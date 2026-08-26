#!/usr/bin/env bash
# Shared teardown. Sourced by the test scripts and usable on its own:
#
#   bash scripts/kill_sim.sh
#
# SCOPED TO THIS PROJECT ON PURPOSE. The obvious version pkills 'gz sim',
# 'parameter_bridge' and 'robot_state_publisher' by name - which also kills
# whatever road_follower or traffic_dodger happen to be running on the same
# machine, from a script the user thought only touched mazes. Everything here
# matches either this package's install path or this project's world files.
#
# IT WAITS FOR THE PROCESSES TO ACTUALLY DIE, and that is the whole point.
#
# The first version sent the signals and slept 2 s. Gazebo takes longer than
# that to shut down, so in a back-to-back run - exactly what the test batch
# does - the next `ros2 launch` came up alongside a server that had not
# finished dying. Two gz servers then share one ROS graph and one world name,
# both bridges publish /scan and /ego/true_odom, and the episode manager can
# end up waiting forever for a pose from a car in a world that is being torn
# down. The symptom was a run that sat in silence for 280 s and reported
# 'nothing finished', intermittently, and only inside a batch. Single runs of
# the same command always passed.
#
# So: signal, then POLL until nothing matches, then escalate to SIGKILL.
kill_sim() {
  # '__node:=maze_' is the one that matters most. ros_gz_bridge,
  # robot_state_publisher and rviz2 are shared executables - the other projects
  # run the same binaries - so they cannot be matched on process name without
  # killing those too. maze_sim.launch.py names them maze_bridge / maze_rsp /
  # maze_rviz, which puts a unique string in their argv.
  #
  # Leaving this out is not a small leak. After a morning of runs there were
  # thirteen surviving ros_gz_bridge processes, all still bridging gz topics
  # into this project's ROS graph, and the car's reported pose alternated
  # between two cells because several of them were publishing it.
  local pats=('ros2 launch maze_solver'
              'install/maze_solver/lib/maze_solver'
              'maze_solver_ws/mazes'
              '__node:=maze_')

  # the launch trees first: ros2 launch shuts its own children down cleanly
  pkill -f "${pats[0]}" >/dev/null 2>&1
  sleep 1
  for p in "${pats[@]}"; do pkill -f "$p" >/dev/null 2>&1; done

  # up to 20 s of polite waiting, then stop asking
  local i alive
  for i in $(seq 40); do
    alive=0
    for p in "${pats[@]}"; do
      pgrep -f "$p" >/dev/null 2>&1 && alive=1
    done
    [ "$alive" -eq 0 ] && break
    if [ "$i" -eq 20 ]; then
      for p in "${pats[@]}"; do pkill -9 -f "$p" >/dev/null 2>&1; done
    fi
    sleep 0.5
  done

  rm -f /tmp/ms_ego.json /tmp/ms_plan.json /tmp/ms_map.json
  # a moment for the DDS discovery graph to forget the dead participants;
  # without it the next run can still see the previous run's endpoints
  sleep 2
  if [ "$alive" -ne 0 ]; then
    echo "  WARNING: something survived kill_sim:"
    for p in "${pats[@]}"; do pgrep -af "$p" | cut -c1-100; done
  fi
}
# when executed rather than sourced, just do it
(return 0 2>/dev/null) || kill_sim
