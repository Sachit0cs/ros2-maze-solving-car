#!/usr/bin/env python3
"""What the car knows in discovery mode. No ROS, no Gazebo, no true maze.

In known-map mode the planner searches a Maze. In discovery mode it searches
one of these instead - and that is the ONLY difference between the two modes.
Knowledge exposes .neighbours, .edge_cost and .heuristic with exactly the
signatures search.py expects, so all six algorithms run unmodified on a map
that is still being drawn. Nothing in search.py knows which mode it is in, and
that is the cleanest way to show students that a search algorithm is a
statement about a graph and not about a robot.

IT CANNOT LEAK THE ANSWER

Knowledge is built from the maze's DIMENSIONS - cols, rows, pitch, start, goal
- and never from its passages. The internal Maze it keeps for geometry is
constructed with every passage explicitly closed, so the true wall layout is
not present in this object in any form. That is deliberate: a discovery-mode
demo where the planner can secretly see the walls proves nothing, and the bug
would be invisible, because a car that cheats simply looks like a car that is
doing very well.

THE FREE-SPACE ASSUMPTION

An edge is UNKNOWN, OPEN or WALL. neighbours() treats UNKNOWN as passable.
That optimism is the standard choice - it is what D* Lite does, and what makes
repeated A* behave sensibly - and it has a clean justification: assuming an
unknown edge is open can only ever make the estimated cost too LOW, which
keeps the search admissible, so the car always drives toward the most
promising possibility and finds out. Assume the opposite and a car in a maze
it has not seen concludes it is walled in and never moves.

The same optimism applies to terrain, which a lidar cannot see at all - it is
paint. An unvisited cell is assumed plain, and its true cost is learned by
driving onto it. So the discovery car can be surprised by mud, replan, and
still be following the same algorithm it started with.
"""
UNKNOWN, OPEN, WALL = 0, 1, 2

# Evidence, not assertion. Each edge keeps a running score: a lidar return
# lands on it, +1; a ray passes cleanly through it, -1. The state is only
# committed once the score reaches +/- COMMIT, and the score is clamped so that
# a wall observed two hundred times can still be argued out of existence by a
# handful of contrary readings.
#
# WHY THIS IS NOT OPTIONAL. The first version marked a wall the instant one ray
# stopped near a lattice line, and refused to ever downgrade it. Offline, with
# a stationary car and an exact pose, that was flawless - scripts/test_discovery
# asserted no false walls over thousands of scans and passed. In Gazebo the car
# is TURNING while it scans, at up to 2.2 rad/s, and the pose it is fused with
# is up to a frame old. A 30 ms lag at 2 rad/s is 3.4 degrees, which at a 5 m
# sight line down a corridor puts the return 0.30 m sideways - most of the way
# to the next lattice line.
#
# One such reading was permanent. The measured result: the car reached (3, 1)
# of a 15x15 maze, decided it was sealed in on every side, and A* expanded
# three nodes and reported no path, over and over, until the episode timed out.
#
# The offline test could not see it because the offline test had no lag. That
# is the same lesson as traffic_dodger's bug 2 - a test that cannot observe the
# axis the error is on will pass throughout.
WALL_VOTE, OPEN_VOTE = 1, -1
COMMIT = 2          # |score| needed before the state is believed
CLAMP = 6           # so a long-held belief is still revisable

from maze_solver.maze import MIN_EDGE_COST, Maze     # noqa: E402


def _score_for(state):
    """The evidence a received state implies.

    The mapper owns the scores; the planner only receives committed states over
    the topic. Reconstituting a score at the commit threshold rather than at
    the clamp keeps the two consistent if a receiver ever starts voting too.
    """
    return COMMIT if state == WALL else (-COMMIT if state == OPEN else 0)


class Knowledge:
    """A partially discovered maze, shaped like a Maze so search.py can run on it."""

    def __init__(self, meta):
        # Geometry only. Every passage is forced closed so that not one bit of
        # the real layout reaches this object - see the module docstring.
        blank = dict(meta)
        blank['passages'] = {'h': [], 'v': []}
        blank['terrain'] = []
        self.geom = Maze.from_meta(blank)

        self.cols, self.rows = self.geom.cols, self.geom.rows
        self.pitch = self.geom.pitch
        self.wall_thickness = self.geom.wall_thickness
        self.start = tuple(meta['start'])
        self.goal = tuple(meta['goal'])

        self.h_state = [[UNKNOWN] * (self.cols - 1) for _ in range(self.rows)]
        self.v_state = [[UNKNOWN] * self.cols for _ in range(self.rows - 1)]
        # the evidence behind each state, in the same shape
        self.h_score = [[0] * (self.cols - 1) for _ in range(self.rows)]
        self.v_score = [[0] * self.cols for _ in range(self.rows - 1)]
        self.terrain = [[1.0] * self.cols for _ in range(self.rows)]
        self.visited = [[False] * self.cols for _ in range(self.rows)]
        self.revision = 0            # bumped whenever anything is learned

    # ------------------------------------------------------------ the graph

    def _state(self, a, b):
        (ac, ar), (bc, br) = a, b
        if ar == br:
            return self.h_state[ar][min(ac, bc)]
        return self.v_state[min(ar, br)][ac]

    def _vote(self, a, b, delta):
        """Add evidence for or against a wall. Returns True if the state moved."""
        (ac, ar), (bc, br) = a, b
        if ar == br:
            score, state, i, j = self.h_score, self.h_state, ar, min(ac, bc)
        else:
            score, state, i, j = self.v_score, self.v_state, min(ar, br), ac
        s = max(-CLAMP, min(CLAMP, score[i][j] + delta))
        score[i][j] = s
        was = state[i][j]
        now = WALL if s >= COMMIT else (OPEN if s <= -COMMIT else UNKNOWN)
        if now != was:
            state[i][j] = now
            self.revision += 1
            return True
        return False

    def neighbours(self, cell):
        """Everything not KNOWN to be walled. Optimism - see the docstring."""
        return [n for n in self.geom._grid_neighbours(cell[0], cell[1])
                if self._state(cell, n) != WALL]

    def known_neighbours(self, cell):
        """Only passages actually confirmed open. Used for reporting, not planning."""
        return [n for n in self.geom._grid_neighbours(cell[0], cell[1])
                if self._state(cell, n) == OPEN]

    def edge_cost(self, a, b):
        return (self.terrain[a[1]][a[0]] + self.terrain[b[1]][b[0]]) / 2.0

    def heuristic(self, cell, goal=None):
        g = goal or self.goal
        return (abs(cell[0] - g[0]) + abs(cell[1] - g[1])) * MIN_EDGE_COST

    # ------------------------------------------------------------- learning

    def learn_wall(self, a, b):
        return self._vote(a, b, WALL_VOTE)

    def learn_open(self, a, b):
        # This used to refuse to downgrade a confirmed wall, reasoning that
        # walls do not move and one stray ray should not re-open one. The
        # reasoning was right and the conclusion was wrong: it also made every
        # WRONG wall permanent, and wrong walls are exactly what a moving,
        # turning car produces. Evidence in both directions, with a commit
        # threshold, gets the intended robustness without the trap.
        return self._vote(a, b, OPEN_VOTE)

    def learn_terrain(self, cell, cost):
        c, r = cell
        self.visited[r][c] = True
        if abs(self.terrain[r][c] - cost) > 1e-9:
            self.terrain[r][c] = cost
            self.revision += 1
            return True
        return False

    def edge_between(self, px, py, tol=None):
        """Which lattice edge a world point sits on, or None.

        The lattice lines are at integer cell coordinates. A point is on a
        vertical wall if its u coordinate is near an integer, on a horizontal
        wall if its v is. A corner post is near BOTH, and is deliberately
        returned as None: a return off a corner cannot be attributed to one
        wall or the other, and guessing marks a wall that may not exist, in a
        map the planner is about to trust.
        """
        if tol is None:
            # half the wall, plus a few sigma of lidar noise
            tol = (self.wall_thickness / 2.0 + 0.03) / self.pitch
        u = px / self.pitch + self.cols / 2.0
        v = py / self.pitch + self.rows / 2.0
        du = abs(u - round(u))
        dv = abs(v - round(v))
        near_u, near_v = du <= tol, dv <= tol
        if near_u == near_v:
            return None                       # neither, or a corner post
        if near_u:
            c = int(round(u))
            r = int(v // 1)
            if not (0 < c < self.cols and 0 <= r < self.rows):
                return None                   # outer boundary: nothing to learn
            return ((c - 1, r), (c, r))
        c = int(u // 1)
        r = int(round(v))
        if not (0 <= c < self.cols and 0 < r < self.rows):
            return None
        return ((c, r - 1), (c, r))

    def cell_of(self, px, py):
        return self.geom.world_to_cell(px, py)

    def integrate(self, origin, hits, misses, wall_margin=0.02, step=0.05):
        """Fold one scan in. Returns how many edges changed state.

        Two passes, because a scan carries two different pieces of information
        and they are learned in opposite ways:

          hits    a ray that stopped. The stopping point is ON a wall, so the
                  lattice edge under it is marked WALL.
          misses  a ray that ran to its limit without stopping, plus the part
                  of every hit ray BEFORE the wall. Every lattice edge such a
                  ray crossed is marked OPEN.

        The free pass is truncated half a wall-thickness short of the return,
        which is the detail that makes this work at all. Sample right up to the
        hit point and the last samples land INSIDE the wall box; the far half
        of that box belongs to the next cell, so the walk sees a cell
        transition and cheerfully marks the wall it just hit as an open
        passage.
        """
        changed = 0
        ox, oy = origin
        for hx, hy in hits:
            e = self.edge_between(hx, hy)
            if e and self.learn_wall(*e):
                changed += 1
        for ex, ey, stop_short in misses:
            dx, dy = ex - ox, ey - oy
            d = (dx * dx + dy * dy) ** 0.5
            if d < 1e-6:
                continue
            if stop_short:
                back = min(d, self.wall_thickness / 2.0 + wall_margin)
                ex, ey = ex - dx / d * back, ey - dy / d * back
                d -= back
            n = max(1, int(d / step))
            prev = self.cell_of(ox, oy)
            for i in range(1, n + 1):
                px = ox + (ex - ox) * i / n
                py = oy + (ey - oy) * i / n
                cur = self.cell_of(px, py)
                if cur != prev:
                    if abs(cur[0] - prev[0]) + abs(cur[1] - prev[1]) == 1:
                        if self.learn_open(prev, cur):
                            changed += 1
                    prev = cur
        return changed

    # ------------------------------------------------------------ reporting

    def coverage(self):
        total = len(self.h_state) * len(self.h_state[0]) if self.h_state else 0
        total += len(self.v_state) * len(self.v_state[0]) if self.v_state else 0
        known = sum(1 for row in self.h_state for s in row if s != UNKNOWN)
        known += sum(1 for row in self.v_state for s in row if s != UNKNOWN)
        return (known / total) if total else 1.0

    def as_dict(self):
        """For the control panel, which draws known walls solid and unknown faint."""
        return {'h': self.h_state, 'v': self.v_state,
                'visited': [[int(b) for b in row] for row in self.visited],
                'terrain': self.terrain,
                'coverage': round(self.coverage(), 4),
                'revision': self.revision}

    def load_state(self, d):
        """Adopt a map published by the mapper node.

        The mapper owns the Knowledge and the planner reads it, so the two
        cross a ROS topic as JSON. Copying the arrays in place rather than
        rebuilding the object keeps the planner's `self.known` identity stable,
        which matters because a search may be running against it.
        """
        h, v = d.get('h'), d.get('v')
        if h and len(h) == len(self.h_state):
            self.h_state = [list(row) for row in h]
            self.h_score = [[_score_for(s) for s in row] for row in h]
        if v and len(v) == len(self.v_state):
            self.v_state = [list(row) for row in v]
            self.v_score = [[_score_for(s) for s in row] for row in v]
        t = d.get('terrain')
        if t and len(t) == len(self.terrain):
            self.terrain = [[float(x) for x in row] for row in t]
        vis = d.get('visited')
        if vis and len(vis) == len(self.visited):
            self.visited = [[bool(x) for x in row] for row in vis]
        self.revision = int(d.get('revision', self.revision))

    def path_is_still_valid(self, path):
        """Has anything learned since planning put a wall across this path?"""
        for i in range(len(path) - 1):
            if self._state(tuple(path[i]), tuple(path[i + 1])) == WALL:
                return False
        return True
