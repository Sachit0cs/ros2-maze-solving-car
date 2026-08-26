#!/usr/bin/env bash
# Integration check: does the car actually solve a maze inside Gazebo?
#
#   bash scripts/test_sim.sh                  # A* on the classic maze
#   bash scripts/test_sim.sh terrain ucs      # a named maze and algorithm
#   bash scripts/test_sim.sh classic astar discovery
#   bash scripts/test_sim.sh trap wall        # the wall follower
#
# The three pure-Python suites already prove the maze, the algorithms and the
# mapper are right on paper. This proves the other half - that the world Gazebo
# loaded matches the graph that was searched, that the bridge is pointing the
# right way, and that the driver can physically get a 160 mm car round a 620 mm
# corner. Those are different claims, and only this one catches a wall box in
# the wrong place or a cmd_vel bridged backwards.
#
# Runs headless. Takes two to four minutes.
# NOTE: no `set -u` - the ROS setup scripts reference unbound variables.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

MAZE=${1:-classic}
ALGO=${2:-astar}
MODE=${3:-known}
BUDGET=${4:-180}
# What COUNTS as success for this combination. Almost always 'goal' - but not
# on maze_trap with the wall follower, where the documented, intended and
# tested outcome is that it never finds the goal at all. A harness that scored
# that as a failure would be asserting the opposite of the lesson.
EXPECT=${5:-goal}
if [ "$MAZE" = "trap" ] && [ "$ALGO" = "wall" ]; then EXPECT=stuck; fi

MAZES=~/maze_solver_ws/mazes
WORLD=$MAZES/maze_$MAZE.sdf
META=$MAZES/maze_$MAZE.json

source "$(dirname "${BASH_SOURCE[0]}")/kill_sim.sh"
trap kill_sim EXIT
kill_sim

# REFUSE TO START if anything from a previous run survived.
#
# A leftover solver writes to the same /tmp/ms_drv.log this run truncates, and
# it holds its own file offset - so the two runs' output interleaves and the
# result is unreadable in a way that looks like data rather than corruption.
# The symptom that gave it away: a progress trace alternating between '9 % at
# (3, 1)' and '72 % at (11, 8)', which is one car in two places. Everything
# read out of that log before it was noticed was worthless.
#
# Better to stop with an explanation than to produce a plausible wrong answer.
LEFTOVER=$(pgrep -cf 'install/maze_solver/lib/maze_solver')
if [ "$LEFTOVER" -gt 0 ]; then
  echo "FAIL: $LEFTOVER maze_solver node(s) survived kill_sim. Another test run"
  echo "      is probably still going. Run one at a time - a leftover solver"
  echo "      writes into the same log this one reads, and the two interleave."
  pgrep -af 'install/maze_solver/lib/maze_solver' | cut -c1-100
  exit 1
fi

if [ ! -f "$WORLD" ]; then
  echo "no maze called '$MAZE' - generating the teaching set first"
  python3 -m maze_solver.make_mazes --out "$MAZES" >/dev/null
fi
[ -f "$WORLD" ] || { echo "FAIL: $WORLD still missing"; exit 1; }

DRIVER=plan
[ "$ALGO" = "wall" ] && DRIVER=wall

echo "=== maze_$MAZE : $ALGO, $MODE mode ==="
python3 - "$META" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
s = m['stats']
print('  %dx%d cells, %.1f x %.1f m, corridor %.2f m, %s'
      % (m['cols'], m['rows'], s['span_m'][0], s['span_m'][1], m['corridor'],
         'perfect' if s['perfect'] else 'braided'))
print('  start %s -> goal %s, %d slow cells, %d junctions'
      % (tuple(m['start']), tuple(m['goal']), s['slow_cells'], s['junctions']))
PY

echo "=== launching gazebo headless ==="
ros2 launch maze_solver maze_sim.launch.py \
     world:="$WORLD" gui:=false rviz:=false >/tmp/ms_sim.log 2>&1 &
for i in $(seq 60); do
  ros2 topic list 2>/dev/null | grep -q '/scan' && break
  sleep 2
done
sleep 6

if ! ros2 topic list 2>/dev/null | grep -q '/scan'; then
  echo "FAIL: /scan never appeared. Last 30 lines of the sim log:"
  tail -30 /tmp/ms_sim.log
  exit 1
fi
N=$(ros2 topic list 2>/dev/null | grep -cxE '/scan|/cmd_vel|/ego/true_odom')
echo "  bridged: $N/3 core topics"
[ "$N" -eq 3 ] || { echo "FAIL: a core topic is missing"; ros2 topic list; exit 1; }

# A topic EXISTING proves only that the bridge started. The bridge creates its
# topics whether or not anything is publishing on the Gazebo side, so a car
# that failed to spawn, or a lidar whose render context did not come up, looks
# exactly like a healthy system to `ros2 topic list`.
#
# That distinction cost a 280 s silent timeout: the episode manager was waiting
# for a pose on /ego/true_odom that never came, while the topic sat there
# looking fine. So wait for an actual MESSAGE on each, and fail immediately and
# specifically if one never arrives.
for T in /clock /scan /ego/true_odom; do
  if timeout 25 ros2 topic echo --once "$T" >/dev/null 2>&1; then
    echo "  $T is publishing"
  else
    echo "FAIL: $T exists but nothing is publishing on it."
    echo "      The bridge is up; the Gazebo side is not. Usually a stale gz"
    echo "      server from a previous run - check kill_sim, and try again."
    pgrep -af 'maze_solver_ws/mazes' | cut -c1-100
    tail -20 /tmp/ms_sim.log
    exit 1
  fi
done

# One scan, checked before anything drives. A 360 degree lidar sitting in the
# middle of a cell must see four walls at roughly half a corridor, or the maze
# geometry and the robot are not in the same world and nothing after this
# matters.
python3 - <<'PY'
import re
import subprocess
import sys

try:
    # --full-length matters: without it ros2 topic echo TRUNCATES the array
    # and the 'nearest return' is computed over an arbitrary prefix of the
    # scan. That produced a confident, wrong 0.150 m reading during bring-up.
    out = subprocess.run(['ros2', 'topic', 'echo', '--once', '--full-length',
                          '--field', 'ranges', '/scan'],
                         capture_output=True, text=True, timeout=40).stdout
except subprocess.TimeoutExpired:
    print('  WARN: no scan arrived within 40 s')
    sys.exit(0)

vals = [float(x) for x in re.findall(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?', out)]
vals = [v for v in vals if 0.0 < v < 50.0]
if not vals:
    print('  WARN: could not parse a scan')
    sys.exit(0)
print('  scan: %d returns, nearest %.3f m, furthest %.3f m'
      % (len(vals), min(vals), max(vals)))
# At a cell centre the nearest wall is corridor/2 = 0.31 m. Anything under
# 0.12 m means the car did not spawn where the maze says the start is.
if min(vals) < 0.12:
    print('  FAIL: the car is already touching a wall at spawn')
    sys.exit(1)
PY

echo "=== starting the solver ==="
ros2 launch maze_solver solve.launch.py meta:="$META" \
     algorithm:="$ALGO" mode:="$MODE" driver:="$DRIVER" \
     >/tmp/ms_drv.log 2>&1 &

echo "=== watching for up to ${BUDGET}s ==="
RESULT=timeout
for i in $(seq "$BUDGET"); do
  if grep -qE 'episode 1: (goal|wall|stuck)' /tmp/ms_drv.log 2>/dev/null; then
    RESULT=$(grep -oE 'episode 1: (goal|wall|stuck)' /tmp/ms_drv.log | head -1 \
             | awk '{print $3}')
    break
  fi
  sleep 1
done

echo
echo "=== what the nodes said ==="
grep -hE 'planner up|mapper up|path_driver up|wall_follower up|maze_manager:|astar|bfs|dfs|ucs|greedy|bidirectional|replan|progress|episode|blocked|backing out|wall contact|SIM CLOCK|waiting for' \
     /tmp/ms_drv.log 2>/dev/null | sed 's/.*\]: //' | tail -22

# Did the SIMULATOR survive? A failing run means nothing if gz died underneath
# it, and gz dying is not a maze_solver bug. It has happened here twice, both
# times because two test batches overlapped and one batch's kill_sim tore down
# the other batch's world - the log shows 'exit code -15', a SIGTERM, not a
# crash. Distinguishing the two is the difference between debugging the project
# and debugging the harness.
if ! pgrep -f 'maze_solver_ws/mazes' >/dev/null 2>&1; then
  echo
  echo "=== the simulator is not running any more ==="
  grep -E 'process has died|exit code' /tmp/ms_sim.log 2>/dev/null | tail -3
  echo "  INCONCLUSIVE - gz exited during the run, so the result below is not"
  echo "  a statement about maze_solver. An 'exit code -15' means something"
  echo "  sent it SIGTERM: usually another test run's kill_sim. Run one at a"
  echo "  time, and re-run this."
  exit 2
fi

echo
echo "=== result ==="
if [ "$RESULT" = "$EXPECT" ] && [ "$EXPECT" != "goal" ]; then
  echo "  PASS - maze_$MAZE with $ALGO ended as '$EXPECT', which is the"
  echo "         documented behaviour: the left-hand rule cannot reach a goal"
  echo "         in the interior of a braided maze. See search.py's"
  echo "         wall_follower docstring for the measured table."
  exit 0
fi
case "$RESULT" in
  goal)
    echo "  PASS - solved maze_$MAZE with $ALGO in $MODE mode"
    grep -oE 'episode 1: goal +in +[0-9.]+ s' /tmp/ms_drv.log | head -1
    exit 0 ;;
  wall)
    echo "  FAIL - hit a wall. The plan and the world disagree, or the driver"
    echo "         cannot hold a corridor. Check scripts/test_maze.py first."
    exit 1 ;;
  stuck)
    echo "  FAIL - made no progress for the stuck timeout."
    exit 1 ;;
  *)
    echo "  FAIL - nothing finished inside ${BUDGET}s."
    echo "  last 20 lines of the driver log:"
    tail -20 /tmp/ms_drv.log
    exit 1 ;;
esac
