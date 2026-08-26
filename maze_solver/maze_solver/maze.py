#!/usr/bin/env python3
"""The maze itself: a grid of cells, the walls between them, and what each
cell costs to drive across. No ROS, no Gazebo - this is arithmetic, so the
whole thing can be checked in milliseconds by scripts/test_maze.py.

WHY A GRID AND NOT A POLYLINE

The road projects generated a centreline and offset it into walls, because a
road is one path. A maze is a *graph*, and the entire point of this project is
that students can watch a search algorithm expand over that graph. So the maze
is stored as the graph directly - cells are vertices, open passages are edges -
and the .sdf walls are derived from it rather than the other way round. Nothing
downstream ever has to infer connectivity from geometry.

COORDINATES

Cells are indexed (c, r) with c across and r up, both from zero. The maze is
centred on the world origin so that a bigger maze grows in every direction
instead of running away from the camera:

    pitch = corridor + wall_thickness
    x = (c + 0.5 - cols/2) * pitch
    y = (r + 0.5 - rows/2) * pitch

Walls live on the lattice *between* cells. A vertical wall has an index
c in [0, cols] - c = 0 is the left outer boundary, c = cols the right one - so
outer boundary and interior walls are the same code path and cannot disagree.

WHY THE OUTER BOUNDARY IS SEALED

A real maze has an entrance gap and an exit gap. This one does not, for the
same reason traffic_dodger caps its start line: a car that can see unbounded
free space through a gap will sometimes find more room out there than in the
maze, and drive out of the problem. The goal is marked with a green disc
instead, which is what the episode manager actually scores against.

EDGE COST

Every cell carries a terrain multiplier t >= 1.0, and the cost of stepping from
a to b is (t_a + t_b) / 2 - you drive out of half of one cell and into half of
the next. Two things follow, and both matter:

  * The graph is UNDIRECTED. cost(a,b) == cost(b,a). Bidirectional search is
    only correct on a graph where that holds, or where you have a reverse
    graph to search backwards over; taking the cost of the destination cell
    alone (the obvious shortcut) breaks it silently.
  * The minimum possible edge cost is exactly 1.0, so Manhattan distance is an
    admissible heuristic without any scaling. test_search.py checks that
    numerically rather than trusting this paragraph.

The unit is "cell-times": how long the car takes to cross one plain cell at
full speed. path_driver.py drives terrain t at v_max / t, so a cost of 12.0
really is twelve cells' worth of driving time, and the number the planner
minimises is the number the stopwatch measures.
"""
import math
import random

# Terrain tiers. The multiplier is also the divisor applied to the car's top
# speed, so these are not decoration - a mud cell genuinely takes 3x as long.
TERRAIN = [
    ('plain',  1.0, '0.30 0.32 0.36 1', '0.38 0.40 0.45 1'),
    ('gravel', 2.0, '0.44 0.40 0.30 1', '0.56 0.51 0.38 1'),
    ('mud',    3.0, '0.32 0.24 0.16 1', '0.42 0.31 0.20 1'),
]
TERRAIN_COST = [t[1] for t in TERRAIN]
MIN_EDGE_COST = min(TERRAIN_COST)

GENERATORS = ('backtracker', 'prim')

DEFAULTS = {
    'cols': 12,
    'rows': 12,
    'seed': 7,
    'generator': 'backtracker',
    'braid': 0.0,
    'rough': 0.0,
    'corridor': 0.62,
    'wall_thickness': 0.06,
    'wall_height': 0.35,
}


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Maze:
    """A rectangular grid maze: passages, terrain, start and goal."""

    def __init__(self, cols=12, rows=12, seed=0, generator='backtracker',
                 braid=0.0, rough=0.0, corridor=0.62, wall_thickness=0.06,
                 wall_height=0.35, start=None, goal=None, passages=None,
                 terrain=None):
        self.cols = int(_clamp(int(cols), 2, 40))
        self.rows = int(_clamp(int(rows), 2, 40))
        self.seed = int(seed)
        self.generator = generator if generator in GENERATORS else 'backtracker'
        self.braid = float(_clamp(braid, 0.0, 1.0))
        self.rough = float(_clamp(rough, 0.0, 1.0))
        self.corridor = float(_clamp(corridor, 0.34, 1.60))
        self.wall_thickness = float(_clamp(wall_thickness, 0.02, 0.30))
        self.wall_height = float(_clamp(wall_height, 0.12, 1.20))
        self.pitch = self.corridor + self.wall_thickness

        # open_h[r][c] is the passage between (c, r) and (c+1, r)
        # open_v[r][c] is the passage between (c, r) and (c, r+1)
        self.open_h = [[False] * (self.cols - 1) for _ in range(self.rows)]
        self.open_v = [[False] * self.cols for _ in range(self.rows - 1)]
        self.terrain = [[1.0] * self.cols for _ in range(self.rows)]

        rng = random.Random(seed)
        if passages is not None:
            # a hand-edited maze from the control panel: take the walls as
            # given and carve nothing
            self._load_passages(passages)
        else:
            if self.generator == 'prim':
                self._carve_prim(rng)
            else:
                self._carve_backtracker(rng)
            if self.braid > 0.0:
                self._braid(rng)

        if terrain is not None:
            self._load_terrain(terrain)
        elif self.rough > 0.0:
            self._scatter_terrain(rng)

        self.start = tuple(start) if start else (0, 0)
        self.goal = tuple(goal) if goal else (self.cols - 1, self.rows - 1)
        self.start = self._sane_cell(self.start, (0, 0))
        self.goal = self._sane_cell(self.goal, (self.cols - 1, self.rows - 1))

    # ------------------------------------------------------------ carving

    def _carve_backtracker(self, rng):
        """Recursive backtracker: carve a depth-first spanning tree.

        Produces long, snaking corridors with few junctions - the classic
        'hedge maze' texture. Because it is a spanning tree the result is a
        PERFECT maze: exactly one path between any two cells, so every search
        algorithm here returns the same path and only the *work* differs. That
        is the right first lesson; turn up braid to break it.

        Iterative rather than recursive on purpose: a 40x40 maze is 1600 cells
        deep in the worst case and CPython's recursion limit is 1000.
        """
        seen = [[False] * self.cols for _ in range(self.rows)]
        c, r = rng.randrange(self.cols), rng.randrange(self.rows)
        seen[r][c] = True
        stack = [(c, r)]
        while stack:
            c, r = stack[-1]
            nbr = [(nc, nr) for nc, nr in self._grid_neighbours(c, r)
                   if not seen[nr][nc]]
            if not nbr:
                stack.pop()
                continue
            nc, nr = rng.choice(nbr)
            self._open(c, r, nc, nr)
            seen[nr][nc] = True
            stack.append((nc, nr))

    def _carve_prim(self, rng):
        """Randomised Prim's: grow a spanning tree from a random frontier.

        Same guarantee (a spanning tree, so a perfect maze), completely
        different texture - bushy, with many short dead ends branching off a
        stubby trunk. Worth having precisely because it changes which
        algorithms look good: DFS gets lucky on long corridors and looks
        terrible in a bush.
        """
        seen = [[False] * self.cols for _ in range(self.rows)]
        c, r = rng.randrange(self.cols), rng.randrange(self.rows)
        seen[r][c] = True
        frontier = [((c, r), n) for n in self._grid_neighbours(c, r)]
        while frontier:
            i = rng.randrange(len(frontier))
            (ac, ar), (bc, br) = frontier.pop(i)
            if seen[br][bc]:
                continue
            self._open(ac, ar, bc, br)
            seen[br][bc] = True
            frontier.extend(((bc, br), n) for n in self._grid_neighbours(bc, br)
                            if not seen[n[1]][n[0]])

    def _braid(self, rng):
        """Knock a wall out of some dead ends, creating loops.

        This is the switch that makes the algorithm comparison interesting. A
        perfect maze has one solution, so BFS, DFS, UCS and A* all return the
        SAME path and differ only in how much they expanded. Once there are
        loops there are genuinely different routes of different cost, and DFS's
        first-found path is visibly worse than A*'s.
        """
        dead = [(c, r) for r in range(self.rows) for c in range(self.cols)
                if len(self.neighbours((c, r))) == 1]
        rng.shuffle(dead)
        for c, r in dead[:int(round(len(dead) * self.braid))]:
            if len(self.neighbours((c, r))) != 1:
                continue          # an earlier braid already opened this one
            shut = [(nc, nr) for nc, nr in self._grid_neighbours(c, r)
                    if not self._is_open(c, r, nc, nr)]
            if shut:
                nc, nr = rng.choice(shut)
                self._open(c, r, nc, nr)

    def _scatter_terrain(self, rng):
        """Sprinkle patches of slow ground.

        Blobs rather than independent per-cell draws: scattered single slow
        cells are noise a planner routes around for free, whereas a patch is a
        real decision - go through the mud or take the long way round. That
        decision is the whole reason UCS and A* exist, so the terrain has to be
        shaped like one.
        """
        area = self.cols * self.rows
        n_blobs = max(1, int(round(area * self.rough / 9.0)))
        for _ in range(n_blobs):
            tier = rng.randrange(1, len(TERRAIN))
            cost = TERRAIN_COST[tier]
            c0, r0 = rng.randrange(self.cols), rng.randrange(self.rows)
            radius = rng.uniform(0.8, 2.4)
            for r in range(self.rows):
                for c in range(self.cols):
                    if math.hypot(c - c0, r - r0) <= radius:
                        self.terrain[r][c] = max(self.terrain[r][c], cost)
        # start and goal stay plain: a start buried in mud makes every reported
        # cost larger by a constant that has nothing to do with the search
        for c, r in ((0, 0), (self.cols - 1, self.rows - 1)):
            self.terrain[r][c] = 1.0

    # ------------------------------------------------------------ topology

    def _grid_neighbours(self, c, r):
        """The four grid neighbours, walls ignored."""
        out = []
        if c > 0:
            out.append((c - 1, r))
        if c < self.cols - 1:
            out.append((c + 1, r))
        if r > 0:
            out.append((c, r - 1))
        if r < self.rows - 1:
            out.append((c, r + 1))
        return out

    def _open(self, ac, ar, bc, br):
        if ar == br:
            self.open_h[ar][min(ac, bc)] = True
        else:
            self.open_v[min(ar, br)][ac] = True

    def _is_open(self, ac, ar, bc, br):
        if ar == br:
            if abs(ac - bc) != 1:
                return False
            return self.open_h[ar][min(ac, bc)]
        if ac != bc or abs(ar - br) != 1:
            return False
        return self.open_v[min(ar, br)][ac]

    def neighbours(self, cell):
        """Cells reachable from `cell` in one move. THE graph edge relation.

        Four-connected, never diagonal. A diagonal step between two cells whose
        shared corner has walls on both sides would drive the car straight
        through a wall post, and a maze corridor is 0.62 m wide against a
        0.16 m car - there is no room to cut a corner.
        """
        c, r = cell
        return [n for n in self._grid_neighbours(c, r)
                if self._is_open(c, r, n[0], n[1])]

    def edge_cost(self, a, b):
        """Cost of one step, in cell-times. Symmetric - see the module docstring."""
        return (self.terrain[a[1]][a[0]] + self.terrain[b[1]][b[0]]) / 2.0

    def manhattan(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def heuristic(self, cell, goal=None):
        """Admissible, consistent estimate of remaining cost.

        Every step costs at least MIN_EDGE_COST (1.0) and at least Manhattan
        many steps remain, so this never overestimates. Scaling it up is what
        Weighted A* would do; this project keeps A* optimal and lets Greedy
        Best-First be the cautionary tale instead.
        """
        return self.manhattan(cell, goal or self.goal) * MIN_EDGE_COST

    def in_bounds(self, cell):
        c, r = cell
        return 0 <= c < self.cols and 0 <= r < self.rows

    def is_perfect(self):
        """True if the maze is a spanning tree: connected, and no loops."""
        n = self.cols * self.rows
        edges = sum(row.count(True) for row in self.open_h)
        edges += sum(row.count(True) for row in self.open_v)
        return edges == n - 1 and self.connected_count() == n

    def connected_count(self):
        """How many cells are reachable from the start."""
        seen = {self.start}
        stack = [self.start]
        while stack:
            for nb in self.neighbours(stack.pop()):
                if nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        return len(seen)

    def _sane_cell(self, cell, fallback):
        return tuple(cell) if self.in_bounds(cell) else fallback

    # ------------------------------------------------------------ geometry

    def cell_centre(self, c, r):
        """World (x, y) of a cell centre."""
        return ((c + 0.5 - self.cols / 2.0) * self.pitch,
                (r + 0.5 - self.rows / 2.0) * self.pitch)

    def world_to_cell(self, x, y):
        """Which cell a world point is in. Clamped, never None.

        Clamping rather than returning None is deliberate: the caller is always
        the driver or the mapper asking 'where am I', and a car nudged 2 cm
        outside the boundary by a collision still wants an answer it can plan
        from, not an exception.
        """
        c = int(math.floor(x / self.pitch + self.cols / 2.0))
        r = int(math.floor(y / self.pitch + self.rows / 2.0))
        return (int(_clamp(c, 0, self.cols - 1)), int(_clamp(r, 0, self.rows - 1)))

    def path_points(self, path):
        """A cell path as world waypoints."""
        return [self.cell_centre(c, r) for c, r in path]

    def path_cost(self, path):
        return sum(self.edge_cost(path[i], path[i + 1])
                   for i in range(len(path) - 1))

    def wall_segments(self):
        """Every wall as (x, y, yaw, length, thickness).

        Each segment is exactly `pitch` long, which is corridor + thickness, so
        it reaches half a wall-thickness into the corner post at each end. Two
        perpendicular walls meeting at a corner therefore both cover that
        corner and no gap is ever left for the lidar to see through. The road
        generator needed an explicit 6 percent overlap to get the same
        property; on a lattice it falls out of the geometry for free.
        """
        segs = []
        half_c, half_r = self.cols / 2.0, self.rows / 2.0
        # vertical walls: index c in [0, cols], blocking movement in x
        for r in range(self.rows):
            y = (r + 0.5 - half_r) * self.pitch
            for c in range(self.cols + 1):
                if c == 0 or c == self.cols:
                    present = True
                else:
                    present = not self.open_h[r][c - 1]
                if present:
                    x = (c - half_c) * self.pitch
                    segs.append((x, y, math.pi / 2.0, self.pitch,
                                 self.wall_thickness))
        # horizontal walls: index r in [0, rows], blocking movement in y
        for c in range(self.cols):
            x = (c + 0.5 - half_c) * self.pitch
            for r in range(self.rows + 1):
                if r == 0 or r == self.rows:
                    present = True
                else:
                    present = not self.open_v[r - 1][c]
                if present:
                    y = (r - half_r) * self.pitch
                    segs.append((x, y, 0.0, self.pitch, self.wall_thickness))
        return segs

    def start_pose(self):
        """Where the car spawns: the start cell, facing an open neighbour.

        Facing an arbitrary direction would have the car begin every episode by
        reversing out of a wall 0.3 m away, which the driver can do but which
        wastes the first two seconds of every run and looks broken.
        """
        x, y = self.cell_centre(*self.start)
        nb = self.neighbours(self.start)
        if nb:
            nx, ny = self.cell_centre(*nb[0])
            return x, y, math.atan2(ny - y, nx - x)
        return x, y, 0.0

    def goal_pose(self):
        return self.cell_centre(*self.goal)

    def goal_radius(self):
        # Just inside the cell, so 'reached the goal' means the car is actually
        # in the goal cell and not merely leaning into it from the corridor.
        return round(self.corridor * 0.45, 3)

    # ------------------------------------------------------- serialisation

    def passage_bits(self):
        """Passages as two lists of 0/1 rows - JSON-friendly and exact."""
        return {'h': [[int(v) for v in row] for row in self.open_h],
                'v': [[int(v) for v in row] for row in self.open_v]}

    def _load_passages(self, p):
        h, v = p.get('h', []), p.get('v', [])
        for r in range(min(len(h), self.rows)):
            for c in range(min(len(h[r]), self.cols - 1)):
                self.open_h[r][c] = bool(h[r][c])
        for r in range(min(len(v), self.rows - 1)):
            for c in range(min(len(v[r]), self.cols)):
                self.open_v[r][c] = bool(v[r][c])

    def _load_terrain(self, t):
        for r in range(min(len(t), self.rows)):
            for c in range(min(len(t[r]), self.cols)):
                self.terrain[r][c] = max(1.0, float(t[r][c]))

    def to_meta(self):
        """Everything needed to rebuild this exact maze.

        The passages and the terrain are stored EXPLICITLY rather than as
        (seed, generator) for from_meta to re-carve. The control panel can
        hand-edit a maze, and a hand-edited maze has no seed that produces it -
        so storing the recipe instead of the result would quietly discard every
        edit the moment anything reloaded the file.
        """
        sx, sy, sth = self.start_pose()
        gx, gy = self.goal_pose()
        return {
            'cols': self.cols, 'rows': self.rows, 'seed': self.seed,
            'generator': self.generator, 'braid': self.braid,
            'rough': self.rough,
            'corridor': self.corridor, 'wall_thickness': self.wall_thickness,
            'wall_height': self.wall_height, 'pitch': round(self.pitch, 4),
            'start': list(self.start), 'goal': list(self.goal),
            'start_pose': [round(sx, 4), round(sy, 4), round(sth, 4)],
            'goal_pose': [round(gx, 4), round(gy, 4)],
            'goal_radius': self.goal_radius(),
            'passages': self.passage_bits(),
            'terrain': self.terrain,
            'perfect': self.is_perfect(),
        }

    @classmethod
    def from_meta(cls, m):
        return cls(cols=m['cols'], rows=m['rows'], seed=m.get('seed', 0),
                   generator=m.get('generator', 'backtracker'),
                   braid=m.get('braid', 0.0), rough=m.get('rough', 0.0),
                   corridor=m.get('corridor', 0.62),
                   wall_thickness=m.get('wall_thickness', 0.06),
                   wall_height=m.get('wall_height', 0.35),
                   start=m.get('start'), goal=m.get('goal'),
                   passages=m.get('passages'), terrain=m.get('terrain'))

    def stats(self):
        junctions = sum(1 for r in range(self.rows) for c in range(self.cols)
                        if len(self.neighbours((c, r))) > 2)
        dead_ends = sum(1 for r in range(self.rows) for c in range(self.cols)
                        if len(self.neighbours((c, r))) == 1)
        slow = sum(1 for row in self.terrain for t in row if t > 1.0)
        return {'cells': self.cols * self.rows, 'junctions': junctions,
                'dead_ends': dead_ends, 'slow_cells': slow,
                'perfect': self.is_perfect(),
                'reachable': self.connected_count(),
                'span_m': (round(self.cols * self.pitch, 2),
                           round(self.rows * self.pitch, 2))}
