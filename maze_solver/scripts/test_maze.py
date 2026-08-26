#!/usr/bin/env python3
"""Checks on the maze itself: topology, terrain, and the geometry Gazebo gets.

    python3 scripts/test_maze.py [--mazes 30]

No ROS, no simulator, under a second.

The test that matters most is test_walls_match_the_graph. Everything in this
project depends on one thing being true: the walls the lidar hits are exactly
the walls the planner searched. If a single wall box is missing where the graph
says a wall is, the search returns a path through it and the car drives into a
wall it was told was open - and the failure looks like a control bug, not a
geometry bug, so it costs a day. So the correspondence is checked in both
directions, and checked by RAY CASTING through the actual boxes rather than by
re-reading the same list that generated them.
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from maze_solver.maze import GENERATORS, Maze                    # noqa: E402
from maze_solver.maze_gen import to_sdf, write_maze              # noqa: E402

FAILED = []


def check(name, cond, detail=''):
    if cond:
        print('  ok    %s' % name)
    else:
        print('  FAIL  %s   %s' % (name, detail))
        FAILED.append(name)


def mazes(n, **kw):
    for i in range(n):
        kw.setdefault('braid', 0.0)
        yield Maze(cols=5 + (i % 11), rows=5 + ((i * 5) % 11), seed=700 + i,
                   generator=GENERATORS[i % len(GENERATORS)], **kw)


def point_in_box(px, py, box):
    """Is a point inside a rotated wall box? Used for ray casting below."""
    x, y, yaw, length, thick = box
    dx, dy = px - x, py - y
    c, s = math.cos(yaw), math.sin(yaw)
    u = dx * c + dy * s
    v = -dx * s + dy * c
    return abs(u) <= length / 2.0 + 1e-9 and abs(v) <= thick / 2.0 + 1e-9


def ray_blocked(maze, a, b, samples=200):
    """Does the straight line between two cell centres pass through a wall?"""
    boxes = maze.wall_segments()
    ax, ay = maze.cell_centre(*a)
    bx, by = maze.cell_centre(*b)
    for i in range(samples + 1):
        t = i / samples
        px, py = ax + (bx - ax) * t, ay + (by - ay) * t
        for box in boxes:
            if point_in_box(px, py, box):
                return True
    return False


# ------------------------------------------------------------------ topology

def test_generators_make_perfect_mazes(n):
    print('\nboth generators produce a perfect maze')
    for gen in GENERATORS:
        bad = []
        for i in range(n):
            m = Maze(cols=9, rows=7, seed=i, generator=gen)
            if not m.is_perfect():
                bad.append('seed %d: %d/%d reachable' %
                           (i, m.connected_count(), m.cols * m.rows))
        check('%-12s spanning tree, every cell reachable, no loops' % gen,
              not bad, bad[:3])


def test_generators_differ():
    """The two generators must actually have different texture.

    If they produced statistically identical mazes, offering both would be a
    lie in the UI. Prim's is the bushy one, so it should have decidedly more
    dead ends than the backtracker on the same size of grid.
    """
    print('\nthe two generators are texturally different')
    de = {}
    for gen in GENERATORS:
        tot = 0
        for i in range(30):
            m = Maze(cols=15, rows=15, seed=i, generator=gen)
            tot += m.stats()['dead_ends']
        de[gen] = tot / 30.0
    print('        mean dead ends per 15x15:  %s'
          % '  '.join('%s %.1f' % (g, v) for g, v in de.items()))
    check('prim is bushier than backtracker (%.1f vs %.1f dead ends)'
          % (de['prim'], de['backtracker']),
          de['prim'] > de['backtracker'] * 1.2)


def test_braiding(n):
    print('\nbraiding creates loops and removes dead ends')
    grew = shrank = 0
    for i in range(n):
        a = Maze(cols=13, rows=13, seed=i, braid=0.0)
        b = Maze(cols=13, rows=13, seed=i, braid=1.0)
        ea = sum(r.count(True) for r in a.open_h) + sum(r.count(True) for r in a.open_v)
        eb = sum(r.count(True) for r in b.open_h) + sum(r.count(True) for r in b.open_v)
        if eb > ea:
            grew += 1
        if b.stats()['dead_ends'] < a.stats()['dead_ends']:
            shrank += 1
    check('braid=1.0 adds edges on every maze (%d/%d)' % (grew, n), grew == n)
    check('braid=1.0 reduces dead ends on every maze (%d/%d)' % (shrank, n),
          shrank == n)
    check('a braided maze is no longer perfect',
          not Maze(cols=13, rows=13, seed=3, braid=1.0).is_perfect())
    check('braid=0.0 is still perfect',
          Maze(cols=13, rows=13, seed=3, braid=0.0).is_perfect())
    check('braiding never disconnects a cell',
          Maze(cols=13, rows=13, seed=3, braid=1.0).connected_count() == 169)


def test_terrain(n):
    print('\nterrain')
    flat = Maze(cols=12, rows=12, seed=1, rough=0.0)
    check('rough=0 leaves every cell plain', flat.stats()['slow_cells'] == 0)
    rough = Maze(cols=12, rows=12, seed=1, rough=0.8)
    check('rough=0.8 makes some cells slow (%d/144)'
          % rough.stats()['slow_cells'], rough.stats()['slow_cells'] > 0)
    bad = []
    for m in mazes(n, rough=0.7):
        for r in range(m.rows):
            for c in range(m.cols):
                if m.terrain[r][c] < 1.0:
                    bad.append('seed %d %s = %.2f' % (m.seed, (c, r),
                                                      m.terrain[r][c]))
        if m.terrain[m.start[1]][m.start[0]] != 1.0:
            bad.append('seed %d: start is not plain' % m.seed)
        if m.terrain[m.goal[1]][m.goal[0]] != 1.0:
            bad.append('seed %d: goal is not plain' % m.seed)
    check('no cell is ever cheaper than plain, and start/goal stay plain',
          not bad, bad[:3])
    check('the minimum edge cost really is 1.0 (the heuristic depends on it)',
          abs(min(rough.edge_cost(a, b)
                  for r in range(rough.rows) for c in range(rough.cols)
                  for a in [(c, r)] for b in rough.neighbours(a)) - 1.0) < 1e-9)


# ------------------------------------------------------------------ geometry

def test_cell_coordinates(n):
    print('\nworld <-> cell coordinates round trip')
    bad = []
    for m in mazes(n):
        for r in range(m.rows):
            for c in range(m.cols):
                x, y = m.cell_centre(c, r)
                if m.world_to_cell(x, y) != (c, r):
                    bad.append('seed %d: %s -> (%.3f, %.3f) -> %s'
                               % (m.seed, (c, r), x, y, m.world_to_cell(x, y)))
        # and the corners of a cell must still resolve to that cell
        e = m.corridor / 2.0 * 0.98
        for r in range(m.rows):
            for c in range(m.cols):
                x, y = m.cell_centre(c, r)
                for dx, dy in ((e, e), (-e, -e), (e, -e), (-e, e)):
                    if m.world_to_cell(x + dx, y + dy) != (c, r):
                        bad.append('seed %d: corner of %s escapes the cell'
                                   % (m.seed, (c, r)))
    check('every cell centre and corner maps back to its own cell', not bad,
          bad[:3])
    m = Maze(cols=10, rows=10, seed=1)
    check('the maze is centred on the origin',
          abs(sum(m.cell_centre(c, r)[0] for r in range(10) for c in range(10))) < 1e-6)


def test_walls_match_the_graph(n):
    """The one that protects everything else. Ray cast, both directions."""
    print('\nthe walls Gazebo gets are exactly the walls the planner searched')
    open_but_blocked = []
    shut_but_clear = []
    for m in mazes(n, braid=0.5):
        for r in range(m.rows):
            for c in range(m.cols):
                cell = (c, r)
                for nb in m._grid_neighbours(c, r):
                    is_open = nb in m.neighbours(cell)
                    blocked = ray_blocked(m, cell, nb)
                    if is_open and blocked:
                        open_but_blocked.append('seed %d %s-%s' % (m.seed, cell, nb))
                    if not is_open and not blocked:
                        shut_but_clear.append('seed %d %s-%s' % (m.seed, cell, nb))
    check('every OPEN passage is clear of wall geometry',
          not open_but_blocked, open_but_blocked[:3])
    check('every CLOSED passage is physically blocked',
          not shut_but_clear, shut_but_clear[:3])


def test_boundary_is_sealed(n):
    print('\nthe outer boundary is sealed - the car cannot leave the maze')
    bad = []
    for m in mazes(n, braid=1.0):
        segs = {(round(x, 4), round(y, 4)) for x, y, _, _, _ in m.wall_segments()}
        hc, hr = m.cols / 2.0, m.rows / 2.0
        for r in range(m.rows):
            y = round((r + 0.5 - hr) * m.pitch, 4)
            for x in (round(-hc * m.pitch, 4), round(hc * m.pitch, 4)):
                if (x, y) not in segs:
                    bad.append('seed %d: gap in the side wall at row %d' % (m.seed, r))
        for c in range(m.cols):
            x = round((c + 0.5 - hc) * m.pitch, 4)
            for y in (round(-hr * m.pitch, 4), round(hr * m.pitch, 4)):
                if (x, y) not in segs:
                    bad.append('seed %d: gap in the end wall at col %d' % (m.seed, c))
    check('no gap anywhere in the outer wall, even at braid=1.0', not bad,
          bad[:3])


def test_corridor_clearance():
    """A 0.11 m wide car in a 0.62 m corridor. Check the corridor is real."""
    print('\nthe corridor is as wide as it claims, and the car fits')
    m = Maze(cols=6, rows=6, seed=2, corridor=0.62, wall_thickness=0.06)
    check('pitch = corridor + wall thickness',
          abs(m.pitch - 0.68) < 1e-9, m.pitch)
    # a cell with walls on both sides: the clear gap between the inner faces
    gap = m.pitch - m.wall_thickness
    check('clear gap between two facing walls = corridor (%.3f m)' % gap,
          abs(gap - m.corridor) < 1e-9)
    check('a 0.110 m car has %.0f mm clearance each side' % ((gap - 0.11) / 2 * 1000),
          gap > 0.11 * 2.5)
    narrow = Maze(cols=4, rows=4, seed=1, corridor=0.05)
    check('an absurdly narrow corridor is clamped, not accepted',
          narrow.corridor >= 0.34, narrow.corridor)


def test_start_and_goal():
    print('\nstart and goal')
    bad = []
    for m in mazes(20):
        sx, sy, sth = m.start_pose()
        cx, cy = m.cell_centre(*m.start)
        if abs(sx - cx) > 1e-9 or abs(sy - cy) > 1e-9:
            bad.append('seed %d: spawn is not on the start cell' % m.seed)
        # the spawn heading must point at an OPEN neighbour, not into a wall
        step = (round(math.cos(sth)), round(math.sin(sth)))
        ahead = (m.start[0] + step[0], m.start[1] + step[1])
        if ahead not in m.neighbours(m.start):
            bad.append('seed %d: spawn faces a wall' % m.seed)
    check('the car spawns on the start cell, facing an open passage', not bad,
          bad[:3])
    m = Maze(cols=9, rows=9, seed=1)
    check('goal radius stays inside the cell',
          m.goal_radius() < m.corridor / 2.0)
    check('an out-of-bounds start falls back rather than crashing',
          Maze(cols=5, rows=5, seed=1, start=(99, 99)).start == (0, 0))


# ------------------------------------------------------------- serialisation

def test_meta_round_trip(n):
    print('\nmetadata round trip - the .json must rebuild the exact maze')
    bad = []
    for m in mazes(n, braid=0.4, rough=0.6):
        b = Maze.from_meta(m.to_meta())
        if b.open_h != m.open_h or b.open_v != m.open_v:
            bad.append('seed %d: passages differ' % m.seed)
        if b.terrain != m.terrain:
            bad.append('seed %d: terrain differs' % m.seed)
        if (b.start, b.goal) != (m.start, m.goal):
            bad.append('seed %d: start/goal differ' % m.seed)
        if abs(b.pitch - m.pitch) > 1e-12:
            bad.append('seed %d: pitch differs' % m.seed)
    check('passages, terrain, start, goal and geometry all survive', not bad,
          bad[:3])


def test_hand_edits_survive():
    """A hand-edited maze has no seed that reproduces it.

    to_meta stores the passages themselves rather than (seed, generator) for
    exactly this reason. If it stored the recipe, reloading a maze the student
    drew in the control panel would silently re-carve a random one and throw
    the drawing away.
    """
    print('\na hand-edited maze survives a save and reload')
    m = Maze(cols=8, rows=8, seed=11)
    m.open_h[0][0] = not m.open_h[0][0]        # toggle one wall by hand
    m.open_v[3][4] = not m.open_v[3][4]
    m.terrain[2][2] = 3.0
    m.goal = (4, 4)
    b = Maze.from_meta(m.to_meta())
    check('the toggled walls come back', b.open_h[0][0] == m.open_h[0][0]
          and b.open_v[3][4] == m.open_v[3][4])
    check('the painted mud comes back', b.terrain[2][2] == 3.0)
    check('the moved goal comes back', b.goal == (4, 4))
    check('and it is not silently re-carved from the seed',
          b.open_h == m.open_h and b.open_v == m.open_v)


def test_sdf_is_valid(tmpdir):
    print('\nthe .sdf Gazebo will load')
    m = Maze(cols=10, rows=8, seed=6, braid=0.3, rough=0.7)
    xml = to_sdf(m)
    try:
        root = ET.fromstring(xml)
        ok = True
    except ET.ParseError as e:
        root, ok = None, False
        check('parses as XML', False, str(e))
    if not ok:
        return
    check('parses as XML', True)
    check('is an sdf 1.9 world', root.tag == 'sdf'
          and root.find('world') is not None)
    walls = root.find(".//model[@name='maze_walls']/link")
    n_col = len(walls.findall('collision'))
    n_vis = len(walls.findall('visual'))
    check('one collision box per wall segment (%d)' % n_col,
          n_col == len(m.wall_segments()))
    check('and a visual for every collision', n_col == n_vis)
    tiles = root.find(".//model[@name='terrain']/link")
    check('one visual tile per slow cell (%d)' % len(tiles.findall('visual')),
          len(tiles.findall('visual')) == m.stats()['slow_cells'])
    check('terrain tiles have NO collision - they must not be obstacles',
          len(tiles.findall('collision')) == 0)
    check('the maze walls are static',
          root.find(".//model[@name='maze_walls']/static").text == 'true')
    check('there is a goal marker', root.find(".//model[@name='goal_marker']")
          is not None)

    out = os.path.join(tmpdir, 'maze_test.sdf')
    meta = write_maze(out, maze=m)
    check('write_maze writes the .sdf', os.path.exists(out))
    check('write_maze writes the .json beside it',
          os.path.exists(out[:-4] + '.json'))
    check('the .json rebuilds the same wall count',
          len(Maze.from_meta(meta).wall_segments()) == len(m.wall_segments()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mazes', type=int, default=30)
    ap.add_argument('--tmpdir', default='/tmp')
    a = ap.parse_args()
    n = a.mazes

    print('=' * 68)
    print('maze geometry and generation - %d random mazes per property' % n)
    print('=' * 68)

    test_generators_make_perfect_mazes(n)
    test_generators_differ()
    test_braiding(min(n, 15))
    test_terrain(n)
    test_cell_coordinates(n)
    test_walls_match_the_graph(min(n, 12))
    test_boundary_is_sealed(min(n, 12))
    test_corridor_clearance()
    test_start_and_goal()
    test_meta_round_trip(n)
    test_hand_edits_survive()
    test_sdf_is_valid(a.tmpdir)

    print()
    print('=' * 68)
    if FAILED:
        print('%d FAILED: %s' % (len(FAILED), ', '.join(FAILED)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
