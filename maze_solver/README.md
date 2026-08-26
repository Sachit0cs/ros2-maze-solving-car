# maze_solver

A car that drives out of a maze using classical graph search — breadth-first,
depth-first, bidirectional, uniform-cost, greedy best-first and A\* — with a
2D lidar, in Gazebo, with a browser panel that lets you build the maze, watch
the frontier expand, and then watch the car drive the answer.

**Nothing here learns anything.** Its siblings
[`road_follower`](https://github.com/Sachit0cs/ros2-curvy-road-follower) and
[`traffic_dodger`](https://github.com/Sachit0cs/ros2-traffic-dodging-car) train
a network to imitate an expert. This one is the other half of the syllabus: the
problems where you do not need a network, because the problem is a graph and
graphs have algorithms with proofs. The interesting question stops being "does
it generalise" and becomes **"what does each algorithm actually guarantee, and
what does it cost you"**.

It is built to be taught from. Every algorithm runs on the same interface, so
switching from BFS to A\* changes one dictionary lookup and nothing else, and
the numbers that come back are directly comparable.

## The idea in one picture

```
   maze_gen  ──>  a grid maze: walls, mud, a start and a goal
                              │
             ┌────────────────┴────────────────┐
             │                                 │
      known map                          discovery
   planner searches the maze      planner searches what the
                                  lidar has worked out so far
             │                                 │
             └────────────────┬────────────────┘
                              │
                   one of 7 algorithms  ──>  a path
                              │
                   path_driver: pure pursuit + lidar
                              │
                maze_manager scores it: solved / hit a wall / gave up
```

## Does it work?

Every algorithm on every maze in the teaching set, scored against uniform-cost
search — which is optimal on any graph with non-negative edges and therefore
the ground truth:

| algorithm | total expanded | worst frontier | optimal on | total ms |
|---|---|---|---|---|
| Breadth-first | 1347 | 20 | 6/7 | 6.3 |
| Depth-first | 1306 | 52 | **2/7** | 6.6 |
| Bidirectional BFS | **1294** | 23 | 6/7 | 8.7 |
| Uniform-cost (Dijkstra) | 1355 | 20 | **7/7** | 9.4 |
| Greedy best-first | **815** | 22 | 5/7 | 5.1 |
| **A\*** | 1072 | 29 | **7/7** | 7.9 |
| Wall-follower (left hand) | 20889 | **1** | 0/7 | 132.3 |

Read it as a set of trade-offs rather than a leaderboard:

- **A\* is the only row that is both always optimal and cheaper than the thing
  it is optimal against** — 1072 expansions against UCS's 1355 for identical
  answers, 21 % less work for free. That is what a heuristic buys.
- **Greedy is the cheapest search by a distance** — 815 expansions, 40 % under
  UCS — **and it is wrong on two of seven mazes.** Both halves of that sentence
  are the lesson.
- **Bidirectional expands the least of the uninformed three**, which is the
  b^(d/2) argument showing up as a number.
- **DFS is optimal twice out of seven** and holds the largest frontier of
  anything here, which is not what the textbook bound predicts. See below.
- **The wall follower never once returns an optimal path**, and on `trap` it
  never returns a path at all — it gave up after 20 000 cells. It also uses a
  frontier of exactly 1, because it has no frontier. That is the trade.

Regenerate all of it, in about two seconds:

```bash
python3 scripts/compare_algos.py
```

### And in the simulator

The table above is arithmetic. This is a 160 mm car actually driving out of a
maze, headless, scored by the episode manager:

| maze | algorithm | mode | predicted | measured | |
|---|---|---|---|---|---|
| `tiny` 6×6 | A\* | known | 20.4 s | **21.4 s** | solved |
| `classic` 15×15 | A\* | known | 116 s | **121.6 s** | solved |
| `terrain` 15×15 | UCS | known | 204 s | **196.0 s** | solved |

"Predicted" is `cost × pitch / v_max` — the planner's own number, computed
before the car moved. **The cost model is not a scoring convention, it is a
prediction of the clock**, and on the maze specifically built to make cost and
distance disagree it lands within 4 %.

That is also why the `terrain` row is the interesting one. UCS chose the
80-cell route over the 66-cell route, and the 80-cell route really was faster
on the stopwatch.

### The one maze that separates them

`terrain` — 15×15, braided so there is more than one route, with mud on some
of them:

| algorithm | expanded | peak | cells | cost | seconds | optimal |
|---|---|---|---|---|---|---|
| Breadth-first | 197 | 7 | **66** | 144 | 245 | +24 |
| Depth-first | 193 | 22 | 88 | 179 | 304 | +59 |
| Bidirectional BFS | 184 | 9 | **66** | 144 | 245 | +24 |
| Uniform-cost | 213 | 6 | 80 | **120** | **204** | yes |
| Greedy best-first | **99** | 10 | 80 | **120** | **204** | yes |
| A\* | 204 | 7 | 80 | **120** | **204** | yes |
| Wall-follower | 115 | 1 | 114 | 238 | 405 | +118 |

**BFS finds a route 14 cells shorter that takes 41 seconds longer.** That is
the whole distinction between "shortest" and "cheapest" in one row, and it is
not a rhetorical example — the car really does take 245 s on BFS's route and
204 s on A\*'s, because the cost model and the throttle are the same number
(see below).

Bidirectional BFS is wrong here in exactly the same way BFS is, and for exactly
the same reason: it is two breadth-first searches, and terrain is invisible to
both of them. It minimises *steps*. That is not a defect, it is what the
algorithm is, and having a row where the fast clever algorithm gives the wrong
answer is worth more than another row where everything agrees.

> On a **perfect** maze there is exactly one route, so every algorithm returns
> the same path and only the work differs. That is true, and it teaches
> nothing. The first version of `terrain` was a perfect maze with mud on it and
> every row of this table was identical. Terrain only matters where there is a
> choice.

## The maze

A grid of cells. Cells are vertices, open passages are edges, and the `.sdf`
walls are derived from that graph rather than the other way round — so nothing
downstream ever has to infer connectivity from geometry.

- **Two generators.** Recursive backtracker gives long snaking corridors;
  randomised Prim's gives a bushy thing with many short dead ends (measured:
  1.8× as many dead ends on a 15×15). Both produce a spanning tree, so both are
  *perfect* mazes — exactly one route between any two cells.
- **Braiding** knocks walls out of dead ends, creating loops, which is what
  makes several genuinely different routes exist. Slide it from 0 (one
  solution) to 1 (nearly open floor).
- **Terrain.** Cells carry a multiplier: plain ×1, gravel ×2, mud ×3, scattered
  in blobs rather than per-cell — a lone slow cell is noise a planner routes
  around for free, whereas a patch is a decision.
- The outer boundary is **sealed**. There is no exit gap, for the same reason
  `traffic_dodger` caps its start line: a car that can see unbounded free space
  through a gap will eventually drive out of the problem.

### Cost is measured in seconds, and that is not a metaphor

An edge costs `(terrain[a] + terrain[b]) / 2` — you drive out of half of one
cell and into half of the next. Two things follow, and both matter:

- The graph is **undirected**, `cost(a,b) == cost(b,a)`. Bidirectional search is
  only correct on a graph where that holds; taking the destination cell's cost
  alone — the obvious shortcut — breaks it silently.
- The minimum edge cost is exactly **1.0**, so Manhattan distance is admissible
  with no scaling. `test_search.py` checks that numerically over random terrain
  instead of trusting this paragraph.

The unit is a *cell-time*: how long the car takes to cross one plain cell at
full speed. `path_driver` drives terrain `t` at `v_max / t`. So a cost of 120
really is 120 cells' worth of driving, and

```
seconds = cost * pitch / v_max
```

is an identity, not a correlation. **What the planner minimises is what the
stopwatch measures.** Without that, "UCS finds the faster route" would be a
claim about a friction model rather than about search.

### The heuristic is admissible, and that is not the same as good

A\* saves less as the ground gets rougher, and the reason is visible in the
heuristic's own numbers — `h(start)` as a fraction of the true cost (20×20
mazes, 40 seeds each):

| roughness | UCS expanded | A\* expanded | saving | h(start) / true cost |
|---|---|---|---|---|
| 0.0 | 12348 | 10427 | 16 % | 0.46 |
| 0.4 | 12268 | 10988 | 10 % | 0.33 |
| 0.8 | 12104 | 11090 | 8 % | 0.27 |
| 1.0 | 12086 | 11142 | 8 % | 0.25 |

An admissible heuristic is only worth as much as it is *tight*, and terrain
variance loosens this one monotonically: every extra unit of cost that mud adds
is cost Manhattan distance cannot see. Note also that even on flat ground the
ratio is only 0.46 — in a maze, walls make the true path roughly twice the
straight-line distance, which is why A\* saves 16 % here and not 90 % as it
would on an open plane. **This is the honest reason maze solvers are not
usually A\*-shaped success stories.**

## The seven algorithms

All in [`search.py`](maze_solver/search.py), all with the same signature, all
returning the same `Result`. There is exactly one definition of every number,
applied to all seven — otherwise the comparison table measures the bookkeeping
instead of the algorithms.

| | guarantees | notes |
|---|---|---|
| `bfs` | fewest **steps** | not fewest seconds unless terrain is flat |
| `dfs` | a path | no optimality of any kind |
| `bidirectional` | fewest steps | same answer as BFS from ~half the expansions |
| `ucs` | least **cost** | the ground truth everything is scored against |
| `greedy` | nothing | fast, and often wrong |
| `astar` | least cost | with an admissible heuristic, and this one's is |
| `wall_follower` | a path in a simply-connected maze | and only there |

Three details that are easy to get wrong and are commented where they live:

- **Bidirectional search's stopping rule.** "Halt the moment the frontiers
  touch" is not correct — the first contact can be through a node whose
  combined depth is one more than the best meeting available at that level. It
  expands whole levels and then takes the minimum of `d_forward + d_backward`
  over *every* node both sides have reached. Checked against BFS's step count
  on hundreds of random mazes.
- **DFS marks nodes seen on pop, not on push.** A node can sit in the stack
  several times over; committing to a parent at push time records a parent the
  search did not actually come through.
- **Every priority queue carries a counter as its final tie-break.** Without
  it, `heapq` falls back to comparing the node tuples, which silently makes the
  tie-break "smallest (c, r)" — a different algorithm, with different numbers,
  and one you would never notice was happening.

### Two things the textbook says that this maze disagrees with

Both were written into the docstrings as fact, then measured, then rewritten.

**1. DFS is the memory-efficient one.** Not here. Square perfect mazes, 20
seeds each:

| | bfs peak frontier | dfs peak frontier |
|---|---|---|
| 10×10 | 4.2 | 5.5 |
| 20×20 | 6.0 | 11.2 |
| 40×40 | 9.6 | **29.7** |

Not duplicate stack entries — the deduplicated stack measures the same.
`O(b·m)` beats `O(b^d)` only when `b` is large enough for `b^d` to run away. A
maze has a branching factor of about 1.2 and a depth of hundreds of cells, so
DFS pays for the length of its branch while BFS's ring stays as narrow as the
corridors are. "DFS is the memory-efficient one" is a fact about trees with fat
branching that gets repeated as though it were a fact about search.

**2. The wall follower breaks on a maze with loops.** It does not. 80 mazes per
cell:

| | goal at a corner | goal at the centre |
|---|---|---|
| braid 0.0 | 0/80 trapped | 0/80 trapped |
| braid 0.5 | 0/80 trapped | **46/80 trapped** |
| braid 1.0 | 0/80 trapped | **71/80 trapped** |

The left hand traces one connected component of wall, and reaches the goal if
and only if the goal touches the component it started on. Start and goal are
both corners of a rectangle, so they sit on the outer boundary — the same
component — and loops are irrelevant. It fails when the goal is in the
**interior**, ringed by an island the outer trace never meets.

Which is why the default goal is a corner, so the baseline works out of the
box, and why the control panel lets you drag it to the middle. Drag it there on
a braided maze and the wall follower circles forever while every search still
solves it. That is the argument for building a map, made in ten seconds instead
of a lecture.

## The two modes

The **only** difference between them is which object gets searched.

| | `known` | `discovery` |
|---|---|---|
| planner searches | the real `Maze` | a `Knowledge` — what the lidar has found |
| replans | never, once is the job | when a new wall invalidates the route |
| walls start as | known | unknown, and assumed **open** |
| terrain | known | assumed plain until driven over |

`Knowledge` exposes `.neighbours`, `.edge_cost` and `.heuristic` with exactly
the signatures `search.py` expects, so all six algorithms run unmodified on a
map that is still being drawn. Nothing in `search.py` knows which mode it is
in — which is the cleanest way to show that a search algorithm is a statement
about a graph and not about a robot.

**It cannot leak the answer.** `Knowledge` is built from the maze's dimensions
and never from its passages; the internal geometry object it keeps is
constructed with every passage explicitly closed. A discovery demo whose
planner can secretly see the walls proves nothing, and the bug would be
invisible — a car that cheats looks exactly like a car doing very well.

**Unknown edges are assumed passable.** That optimism is what D\* Lite does and
it has a clean justification: assuming an unknown edge is open can only make
the estimated cost too *low*, which keeps the search admissible, so the car
always drives at the most promising possibility and finds out. Assume the
opposite and a car in a maze it has not seen concludes it is walled in and
never moves.

Offline, against a simulated lidar (10×10 braided mazes with terrain, 8 runs):

```
8/8 reached the goal
mean 16.9 replans, 54.8 cells walked, 77 % of the maze mapped
zero false walls and zero false openings, over every scan of every run
```

## The car

The same ~$70 printable rover as the other three projects — see
[`obstacle_rover`](https://github.com/Sachit0cs/ros2-obstacle-avoidance-rover)
for the BOM. Two deliberate differences:

- **The lidar is 360°, not 270°.** A road car never needs to look behind it. A
  maze car does, twice over: the wall follower's rule is stated in terms of the
  wall on its left and the option of turning *back*, and the discovery mapper
  wants every wall of its current cell in one scan rather than after a
  three-point turn. 180 rays over 360° is 2° per ray — 24 mm sample spacing at
  the far wall of a cell, fine enough to tell a 60 mm wall from a gap.
- **There is no depth camera.** The other projects carry one to catch obstacles
  under the 70 mm scan plane. A maze wall is 350 mm tall and there is nothing
  else in the world, so it would be a sensor no node subscribes to. If you add
  low obstacles to the corridors, `depth_guard.py` from `traffic_dodger` drops
  straight in.

The lidar is mounted **over the wheel axle**, not on the nose. On a 360° scan a
nose mount puts the sensor 40 mm ahead of the turning centre, so every spin
sweeps the whole scan sideways and the mapper has to undo it.

### How it drives

Two steering terms doing two different jobs, the same split `traffic_dodger`
used for lane keeping:

```
w = k_path   * alpha            point at the next waypoint
  + k_centre * (left - right)   stay off the walls
```

Neither can do the other's job. Pure pursuit drives at a waypoint, and a
waypoint is a cell *centre*, so on the inside of a corner the shortest line to
it clips the wall — the car is aiming correctly and still scraping. The lidar
centring term pushes it back out and needs no map at all. It is gated on both
side walls being within half a cell, because at a junction the difference
becomes large and meaningless.

Above 0.7 rad of heading error the car stops and rotates on the spot. A
differential drive can do this and a maze needs it: a 90° turn into a 0.62 m
corridor has no arc that both fits and makes progress.

## The control panel

`start_ui.bat`, then **http://localhost:8090** in Chrome. (8090, not 8088 or
8089 — `road_follower` and `traffic_dodger` hold those.)

- **Build a maze** — size, seed, generator, braiding, roughness, corridor
  width, wall height.
- **Edit it by hand.** Click a wall to knock it through or build one; click
  cells to cycle plain → gravel → mud; drag the start and the goal. The `.sdf`
  is rewritten on every edit, so what is on disk always matches the screen.
- **Solve and animate.** Watch the frontier grow, ring by ring for BFS, in a
  spike at the goal for greedy. This is the picture the project exists for.
- **Compare all** — the full table, on the maze in front of you, instantly.
- **Drive it in Gazebo** — and watch the car on the same map, so you never need
  the Gazebo window open, which on a machine with no GPU is the difference
  between a run you can watch and a run you can afford.

Solving and comparing do **not** start the simulator. They import `search.py`
and answer in single-digit milliseconds, so a student can try seven algorithms
on twenty mazes in the time Gazebo takes to boot once. The algorithm lesson is
there; the robot lesson is in Gazebo; they should not be priced the same.

In discovery mode the panel draws the **car's** map — solid where it has
committed, faint where it is still guessing — not the real one.

## Running it

```bash
cd ~/maze_solver_ws/src/maze_solver
source scripts/env.sh          # ROS, the workspace, and ROS_DOMAIN_ID=42
```

`scripts/env.sh` is not optional — see the domain-collision bug below.

```bash
# 1. generate the teaching set
python3 -m maze_solver.make_mazes --out ~/maze_solver_ws/mazes

# 2. try the algorithms with no simulator at all
python3 scripts/compare_algos.py

# 3. bring up Gazebo on one of them
ros2 launch maze_solver maze_sim.launch.py \
     world:=$HOME/maze_solver_ws/mazes/maze_classic.sdf rviz:=true

# 4. and solve it, in another terminal
ros2 launch maze_solver solve.launch.py \
     meta:=$HOME/maze_solver_ws/mazes/maze_classic.json algorithm:=astar

# other things to try
ros2 launch maze_solver solve.launch.py meta:=... algorithm:=bfs
ros2 launch maze_solver solve.launch.py meta:=... mode:=discovery
ros2 launch maze_solver solve.launch.py meta:=... driver:=wall
```

`driver:=wall` launches **no planner and no mapper**. The whole claim about the
wall follower is that it needs neither, and a launch file that started them
anyway — even harmlessly — would undercut the demonstration every time somebody
ran `ros2 node list`.

### Tests

```bash
python3 scripts/test_search.py      # the algorithms          ~2 s
python3 scripts/test_maze.py        # geometry and generation ~1 s
python3 scripts/test_discovery.py   # the mapper, offline     ~3 s
bash    scripts/test_sim.sh         # and what Gazebo does    ~3 min
```

Those are different claims. The first three prove the maze, the algorithms and
the mapper are right on paper. Only the last one catches a wall box in the
wrong place, a `cmd_vel` bridged backwards, or a car that physically cannot get
round a corner.

```bash
bash scripts/test_sim.sh terrain ucs             # a named maze and algorithm
bash scripts/test_sim.sh classic astar discovery
bash scripts/test_sim.sh trap wall               # watch it fail, on purpose
```

## The teaching set

Seven mazes, each of which makes one point. They are deterministic, so a
lecturer and a student on different machines get identical mazes and identical
numbers.

| | what it is for |
|---|---|
| `tiny` | every algorithm agrees. Start here, so later disagreement is obviously a property of the maze |
| `classic` | a proper hedge maze. Same path from everything, wildly different work |
| `terrain` | mud and loops. **BFS and UCS stop agreeing** |
| `braided` | several real routes, so DFS's path is visibly worse rather than merely different |
| `trap` | loops **and** an interior goal. The wall follower circles forever; every search solves it |
| `open` | almost open floor. Greedy shines and A\* pays for its optimality |
| `big` | 25×25. Expansion counts separate by thousands rather than tens |

## Layout

```
maze_solver/
├── description/maze_car.urdf.xacro   360-degree lidar, ground-truth pose
├── maze_solver/
│   ├── maze.py           the grid, the walls, the terrain    - no ROS
│   ├── search.py         the seven algorithms                - no ROS
│   ├── knowledge.py      a partially discovered maze         - no ROS
│   ├── maze_gen.py       Maze -> .sdf + .json
│   ├── make_mazes.py     the teaching set
│   ├── lidar_utils.py    scan -> the few numbers that matter
│   ├── qos.py            the two QoS profiles, defined once
│   ├── planner.py        runs one algorithm, publishes a path
│   ├── path_driver.py    pure pursuit + lidar centring + recovery
│   ├── wall_follower.py  the left-hand rule, no map at all
│   ├── mapper.py         lidar -> Knowledge, for discovery mode
│   └── maze_manager.py   respawn, scoring, progress in cells
├── launch/{maze_sim,solve}.launch.py
└── scripts/{test_search,test_maze,test_discovery}.py
    scripts/{test_sim.sh,compare_algos.py,env.sh,kill_sim.sh}
```

## Topics

| Topic | Type | From |
|---|---|---|
| `/scan` | LaserScan | Gazebo lidar, 360°, 180 rays |
| `/ego/true_odom` | Odometry | Gazebo — ground-truth pose |
| `/car/world_pose` | PoseStamped | `maze_manager` |
| `/car/terrain` | Float32 | `path_driver` — what the ground costs |
| `/plan` | Path | `planner` — cell centres in world coordinates |
| `/plan/stats` | String | `planner` — expansions, frontier, cost, ms |
| `/maze/known` | String | `mapper` — the discovered map, as JSON |
| `/episode/active` | Bool | `maze_manager` — **transient local**, see below |
| `/episode/event` | String | `goal` / `wall` / `stuck` |
| `/cmd_vel` | Twist | the driver |

## Seven bring-up bugs, and what they teach

Each of these produced plausible-looking behaviour while being completely
wrong, and each took measurement rather than reasoning to find.

**1. The car slid away from the start line.** Spawned at `z = 0.06`, but
`base_footprint` is *defined* to sit at ground contact, so it was dropped 55 mm
onto free-spinning wheels with a frictionless caster. Measured: **1.0 m of
travel in 4 s with nothing commanding it**, ending a full cell east of the
start. The manager scored the episode "stuck", respawned, and the planner
correctly reported a replan in known mode — so it presented as a *planner* bug.
Spawn at `z = 0.005`, and publish zero twist throughout the settle period,
because DiffDrive does not regulate a joint until it has been given a command.

**2. The settle period was being skipped.** `hold_until` was computed as
`now + settle` in the constructor — where the node runs on sim time and
`/clock` has not arrived yet, so `now` was 0 while sim time was already 40. The
countdown now starts on the first tick that has a real clock.

**3. Another project's node was driving this project's planner.** `maze_solver`
was modelled on `traffic_dodger` and inherited its topic names, and ROS 2 puts
every node on domain 0 unless told otherwise. A `road_manager` left running in
another terminal publishes `/episode/active` False every time *it* respawns
*its* car in *its* world, and this planner treated that as a new episode and
replanned — three or four times in known mode, where one search is the whole
job. **Nothing in this repository was wrong.** `ros2 topic info -v
/episode/active` reporting `Node name: road_manager` was the entire diagnosis.
Hence `ROS_DOMAIN_ID=42`, the same way this panel is on 8090 and theirs are on
8088 and 8089.

**4. `/episode/active` is state, not an event.** With the default volatile QoS
the manager published `active = True` at 5 Hz for twenty seconds and the
planner never received one of them — it had come up first, its subscription
matched late, and everything published before the match was simply gone. The
run then failed as "stuck" with the car at the start line and the planner,
correctly, never planning. Intermittent, because it is a discovery race. It is
now **transient local, depth 1**, so a node that joins mid-episode learns the
truth immediately instead of waiting for a transition that has already
happened.

**5. One bad lidar return sealed a passage forever.** The mapper marked a wall
the instant a ray stopped near a lattice line, and never downgraded it.
Offline, with a stationary car and an exact pose, that was flawless — thousands
of scans, zero false walls. In Gazebo the car is *turning* while it scans, at
up to 2.2 rad/s, fused with a pose up to a frame old; 30 ms at 2 rad/s is 3.4°,
which at a 5 m sight line puts the return 0.30 m sideways, most of the way to
the next lattice line. Measured result: **the car reached (3, 1) of a 15×15
maze, concluded it was walled in on all four sides, and A\* expanded three
nodes and reported no path** until the episode timed out. Now each edge keeps a
score — +1 for a hit, −1 for a ray passing through — commits at ±2 and clamps
at ±6, so a belief is revisable. Scans taken while spinning faster than
0.6 rad/s are discarded outright, because a correlated error is the kind an
evidence filter is worst at rejecting.

*The offline test could not see it because the offline test had no lag.* Same
lesson as `traffic_dodger`'s bug 2: a test that cannot observe the axis the
error is on will pass throughout.

**6. The driver deadlocked whenever it was blocked.** Nosing into a wall while
pointed at the next waypoint means `alpha` is near zero, so the centring term
is near zero, so `w` is near zero — and `v` is zero because something is close
ahead. It published `(0, 0)` until the episode timed out: `holding: 0.13 m
ahead, 26 cells of plan left`, over and over. Being blocked is now a *state
with an escape* — hold and steer for a second, then reverse while turning
toward whichever side has more room.

**7. Teardown did not wait, so runs contaminated each other.** `kill_sim` sent
the signals and slept 2 s; Gazebo takes longer than that to shut down. In a
back-to-back batch the next `ros2 launch` came up alongside a server that had
not finished dying — two `gz` servers sharing one ROS graph and one world name,
both bridges publishing `/scan` and `/ego/true_odom`. The symptom was a run
that sat in silence for 280 s and reported "nothing finished", intermittently,
and **only inside a batch**; single runs of the same command always passed.
Teardown now polls until nothing matches and escalates to SIGKILL.

That one also produced a diagnostic lesson. The manager's "no pose" warning
lived in its *active* branch, which is unreachable when the cause is that no
pose ever arrived — no pose means never active means the warning never prints.
**A diagnostic that cannot fire in the case it describes is worse than none,
because its absence reads as reassurance.** And `test_sim.sh` now waits for an
actual *message* on `/scan` and `/ego/true_odom` rather than for the topics to
exist, because the bridge creates its topics whether or not anything is
publishing on the Gazebo side.

## Limitations

- **Four-connected, never diagonal.** A diagonal step between two cells whose
  shared corner has walls on both sides drives through a wall post, and a
  0.62 m corridor against a 0.16 m car has no room to cut a corner.
- **Ground-truth pose.** The car is told where it is. This project is about
  search, not localisation — everything the car does not know is withheld
  *deliberately* (discovery mode hides the walls), and nothing is withheld by
  accident. Adding SLAM would make it a different project, and a much longer
  one.
- **Terrain is painted, not simulated.** A mud tile is 8 mm of visual geometry
  with no collision; the slow-down is applied by `path_driver` dividing its
  speed. Real soft ground would be more interesting physics and completely
  wrong here — the number the planner minimises has to be the number the
  stopwatch measures, exactly.
- **Bidirectional search minimises steps, not cost.** Documented, tested, and
  visible in the `terrain` table. Making it cost-optimal means bidirectional
  Dijkstra with a different termination rule; silently swapping one for the
  other would be worse teaching than the honest caveat.
- **Discovery mode learns terrain by driving on it.** A lidar cannot see paint.
  Unknown cells are assumed plain, which is the same free-space optimism used
  for walls.

## WSL notes

No GPU is needed or used. Run headless (`gui:=false`) for anything scripted —
the Gazebo window is the single most expensive thing on a machine without a
GPU, and the physics and sensors do not need it. The control panel's live map
exists precisely so that turning it off costs you nothing.

If Gazebo fails to start with a rendering error, `scripts/env.sh` already
exports `LIBGL_ALWAYS_SOFTWARE=1`.

Measured real-time factor headless on this machine: **1.00** — sim seconds and
wall seconds are the same thing, so the timings above are directly comparable
to a stopwatch.
