#!/usr/bin/env python3
"""Discovery mode, checked offline against a simulated lidar.

    python3 scripts/test_discovery.py [--mazes 8]

No ROS and no Gazebo, and yet this exercises the entire discovery pipeline:
raycast a 360 degree scan against the real wall boxes, fold it into a
Knowledge, plan on the Knowledge with the same search.py every other mode uses,
drive the plan one cell, repeat. If a car in Gazebo cannot solve a maze it has
not seen, this test says whether the mapper or the driver is at fault - and it
answers in two seconds rather than two minutes.

The simulated lidar is a genuine raycast, not a lookup. Every wall segment is
axis aligned (a vertical wall is a box of size thickness x pitch), so a slab
intersection gives the exact range a perfect sensor would report, and Gaussian
noise is added on top at the same 8 mm the URDF specifies.

THE TWO PROPERTIES THAT MATTER

  no false walls    an edge marked WALL that is really a passage makes the
                    planner route the long way round forever. Recoverable, but
                    it silently destroys every optimality claim.
  no false openings an edge marked OPEN that is really a wall makes the planner
                    hand the driver a path through solid geometry. The car then
                    grinds into a wall while faithfully following its plan, and
                    it looks exactly like a control bug.

Both are asserted over every scan of every run, not just at the end.
"""
import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from maze_solver.knowledge import (CLAMP, COMMIT, OPEN, UNKNOWN,   # noqa: E402
                                   WALL, Knowledge)
from maze_solver.maze import Maze                                  # noqa: E402
from maze_solver.search import astar, bfs, ucs                     # noqa: E402

FAILED = []


def check(name, cond, detail=''):
    if cond:
        print('  ok    %s' % name)
    else:
        print('  FAIL  %s   %s' % (name, detail))
        FAILED.append(name)


# ------------------------------------------------------------ a fake lidar

def aabbs(maze):
    """Wall segments as axis-aligned boxes (x, y, half_x, half_y)."""
    out = []
    for x, y, yaw, length, thick in maze.wall_segments():
        if abs(yaw) < 1e-6:
            out.append((x, y, length / 2.0, thick / 2.0))
        else:
            out.append((x, y, thick / 2.0, length / 2.0))
    return out


def ray_range(boxes, ox, oy, ang, far):
    """Exact distance to the first box along a ray. Slab method."""
    dx, dy = math.cos(ang), math.sin(ang)
    best = far
    for bx, by, hx, hy in boxes:
        tmin, tmax = 0.0, best
        for o, d, b, h in ((ox, dx, bx, hx), (oy, dy, by, hy)):
            if abs(d) < 1e-12:
                if abs(o - b) > h:
                    tmin = tmax + 1.0        # parallel and outside: no hit
                    break
            else:
                t1 = (b - h - o) / d
                t2 = (b + h - o) / d
                if t1 > t2:
                    t1, t2 = t2, t1
                tmin = max(tmin, t1)
                tmax = min(tmax, t2)
        if tmin <= tmax and tmin < best:
            best = tmin
    return best


def fake_scan(maze, boxes, x, y, yaw, n=180, far=6.0, noise=0.008, rng=None):
    """One 360 degree scan: (hits, misses) in the form Knowledge.integrate wants."""
    hits, misses = [], []
    for i in range(n):
        a = -math.pi + i * (2 * math.pi / n)
        r = ray_range(boxes, x, y, yaw + a, far)
        if rng is not None and r < far:
            r = max(0.02, r + rng.gauss(0.0, noise))
        b = yaw + a
        px, py = x + math.cos(b) * r, y + math.sin(b) * r
        if r < far - 1e-6:
            hits.append((px, py))
            misses.append((px, py, True))
        else:
            misses.append((px, py, False))
    return hits, misses


def truth_state(maze, a, b):
    return OPEN if b in maze.neighbours(a) else WALL


def audit(maze, k):
    """Every edge the car believes it knows must agree with the real maze."""
    false_wall, false_open = [], []
    for r in range(maze.rows):
        for c in range(maze.cols):
            for nb in maze._grid_neighbours(c, r):
                s = k._state((c, r), nb)
                if s == UNKNOWN:
                    continue
                t = truth_state(maze, (c, r), nb)
                if s == WALL and t == OPEN:
                    false_wall.append(((c, r), nb))
                if s == OPEN and t == WALL:
                    false_open.append(((c, r), nb))
    return false_wall, false_open


# ------------------------------------------------------------------- tests

def test_single_scan_is_truthful(n):
    print('\none scan, from one cell: everything learned is true')
    fw, fo, learned = [], [], 0
    rng = random.Random(4)
    for i in range(n):
        m = Maze(cols=9, rows=9, seed=90 + i, braid=0.3)
        boxes = aabbs(m)
        k = Knowledge(m.to_meta())
        # TWO passes over the same cells. One vote per edge per scan is the
        # rule now, and COMMIT is 2, so a single scan deliberately commits
        # nothing at all - 'two votes' has to mean two independent looks or it
        # is not evidence, it is one correlated mistake counted twice.
        for _ in range(2):
            for cell in [m.start, (4, 4), (8, 0)]:
                x, y = m.cell_centre(*cell)
                hits, misses = fake_scan(m, boxes, x, y, 0.3, rng=rng)
                learned += k.integrate((x, y), hits, misses)
        a, b = audit(m, k)
        fw += a
        fo += b
    check('no passage was ever mistaken for a wall', not fw, fw[:3])
    check('no wall was ever mistaken for a passage', not fo, fo[:3])
    check('two looks from the same places commit real edges (%d updates)'
          % learned, learned > 0)


def test_knowledge_hides_the_maze():
    """The object must not contain the answer, not merely decline to show it."""
    print('\nKnowledge cannot leak the true maze')
    m = Maze(cols=10, rows=10, seed=3, braid=0.4, rough=0.7)
    k = Knowledge(m.to_meta())
    check('every edge starts UNKNOWN',
          all(s == UNKNOWN for row in k.h_state for s in row)
          and all(s == UNKNOWN for row in k.v_state for s in row))
    check('its geometry copy has no passages at all',
          k.geom.connected_count() == 1,
          '%d cells reachable - the true walls leaked in' % k.geom.connected_count())
    check('every cell starts assumed plain',
          all(t == 1.0 for row in k.terrain for t in row))
    check('but the geometry is right', abs(k.pitch - m.pitch) < 1e-12
          and (k.cols, k.rows) == (m.cols, m.rows))


def test_optimism():
    print('\nunknown edges are treated as open, so the car will explore')
    m = Maze(cols=8, rows=8, seed=12)
    k = Knowledge(m.to_meta())
    check('an untouched map looks fully connected',
          len(k.neighbours((4, 4))) == 4)
    check('and a plan exists on it immediately',
          bool(astar(k, k.start, k.goal).path))


def test_evidence():
    """The commit-and-revise contract, which is what keeps the car unstuck.

    The first version of this asserted 'one hit marks a wall, and nothing ever
    un-marks it'. Both halves are now deliberately false, and the reason is in
    knowledge.py: a turning car produces occasional returns attributed to the
    wrong lattice line, and under that rule ONE of them sealed a passage
    permanently. In Gazebo the car reached (3, 1) of a 15x15 maze, concluded it
    was walled in on all four sides, and A* expanded three nodes and reported
    no path until the episode timed out.

    So the contract is now: evidence accumulates, the state commits at a
    threshold, and a belief can be argued out of - slowly if it is well
    supported, quickly if it is not.
    """
    print('\nwalls are believed on evidence, and can be revised')
    m = Maze(cols=8, rows=8, seed=12)
    k = Knowledge(m.to_meta())
    a, b = (4, 4), (5, 4)

    k.learn_wall(a, b)
    check('ONE hit is not enough to seal a passage', b in k.neighbours(a),
          'a single stray return would box the car in - that was the bug')
    for _ in range(COMMIT - 1):
        k.learn_wall(a, b)
    check('%d hits commit it, and it leaves the graph' % COMMIT,
          b not in k.neighbours(a))

    for _ in range(20):
        k.learn_wall(a, b)          # a real wall, seen from many rays
    k.learn_open(a, b)
    check('a well-supported wall survives one contrary reading',
          b not in k.neighbours(a))

    for _ in range(CLAMP * 2):
        k.learn_open(a, b)
    check('but sustained contrary evidence does re-open it',
          b in k.neighbours(a),
          'a wrong wall must be recoverable or the car stays boxed in')

    # and the reverse direction: a confirmed opening should not be closed by
    # one stray return either
    c, d = (2, 2), (2, 3)
    for _ in range(CLAMP * 2):
        k.learn_open(c, d)
    k.learn_wall(c, d)
    check('a confirmed opening survives one stray hit', d in k.neighbours(c))


def test_full_discovery_run(n, verbose=False):
    """Plan, drive one cell, scan, replan when invalidated. Does it finish?"""
    print('\na complete discovery run - plan, drive, scan, replan')
    rng = random.Random(11)
    unfinished, false_open_seen, false_wall_seen = [], [], []
    replans, steps_taken, coverages, optimal_hits = [], [], [], 0

    for i in range(n):
        m = Maze(cols=10, rows=10, seed=400 + i, braid=0.25, rough=0.5)
        boxes = aabbs(m)
        k = Knowledge(m.to_meta())
        at = m.start
        yaw = 0.0
        walked = [at]
        n_replan = 0
        path = []
        for _ in range(600):
            x, y = m.cell_centre(*at)
            hits, misses = fake_scan(m, boxes, x, y, yaw, rng=rng)
            k.integrate((x, y), hits, misses)
            k.learn_terrain(at, m.terrain[at[1]][at[0]])

            fw, fo = audit(m, k)
            false_wall_seen += fw
            false_open_seen += fo

            if at == m.goal:
                break
            if not path or path[0] != at or not k.path_is_still_valid(path):
                res = astar(k, at, k.goal)
                path = res.path
                n_replan += 1
                if not path:
                    break
            nxt = tuple(path[1])
            # the driver would refuse to cross a wall; so does this
            if nxt not in m.neighbours(at):
                k.learn_wall(at, nxt)
                path = []
                continue
            yaw = math.atan2(nxt[1] - at[1], nxt[0] - at[0])
            at = nxt
            path = path[1:]
            walked.append(at)

        if at != m.goal:
            unfinished.append(m.seed)
        else:
            replans.append(n_replan)
            steps_taken.append(len(walked) - 1)
            coverages.append(k.coverage())
            if len(walked) - 1 == bfs(m, m.start, m.goal).steps:
                optimal_hits += 1
        if verbose:
            print('        seed %d: %d cells walked, %d replans, %.0f%% mapped'
                  % (m.seed, len(walked) - 1, n_replan, k.coverage() * 100))

    check('every run reached the goal (%d/%d)' % (n - len(unfinished), n),
          not unfinished, unfinished[:5])
    check('never marked a real passage as a wall, over every scan of every run',
          not false_wall_seen, false_wall_seen[:3])
    check('never marked a real wall as a passage, over every scan of every run',
          not false_open_seen, false_open_seen[:3])
    if replans:
        print('        mean %.1f replans, %.1f cells walked, %.0f%% of the maze mapped'
              % (sum(replans) / len(replans),
                 sum(steps_taken) / len(steps_taken),
                 100.0 * sum(coverages) / len(coverages)))
        print('        walked the shortest possible route anyway on %d/%d runs'
              % (optimal_hits, len(replans)))
    check('a discovery run costs more steps than knowing the map would',
          sum(steps_taken) >= sum(bfs(Maze(cols=10, rows=10, seed=400 + i,
                                           braid=0.25, rough=0.5),
                                      (0, 0), (9, 9)).steps
                                  for i in range(len(steps_taken))),
          'discovery cannot beat perfect knowledge - if it did, it is cheating')


def test_survives_pose_lag(n):
    """The failure the original offline test could not see.

    Everything here used to fuse each scan with the EXACT pose it was taken
    from, because that is what a fake lidar naturally gives you. Gazebo does
    not: the pose is up to a frame old, and the car is turning while it scans.
    At 2 rad/s, 33 ms is 3.4 degrees, and 3.4 degrees at a 5 m sight line puts
    a return 0.30 m sideways - most of the way to the next lattice line.

    The consequence was severe and invisible to this file: the car sealed
    itself into a 40-cell pocket of maze_classic, A* expanded those 40 nodes
    and reported no path, and the episode timed out. Meanwhile this test passed
    over thousands of scans, because it was measuring a system without the
    error in it.

    So the error goes in. Each scan is integrated against a pose that is wrong
    by a bearing offset, the way a real one is, and the two properties that
    matter are asserted against the truth regardless.
    """
    print('\nfalse walls do not survive a stale pose (the Gazebo failure)')
    rng = random.Random(21)
    fw, fo = [], []
    for i in range(n):
        m = Maze(cols=12, rows=12, seed=600 + i, braid=0.3)
        boxes = aabbs(m)
        k = Knowledge(m.to_meta())
        at, prev = m.start, None
        for _ in range(70):
            x, y = m.cell_centre(*at)
            true_yaw = rng.uniform(-math.pi, math.pi)
            # the scan is taken at true_yaw; the pose we fuse it with is a
            # frame stale, so it is off by up to +/- 4 degrees
            lag = rng.uniform(-0.07, 0.07)
            hits, misses = fake_scan(m, boxes, x, y, true_yaw, rng=rng)
            hits = [_rotate(p, (x, y), lag) for p in hits]
            misses = [_rotate(p[:2], (x, y), lag) + (p[2],) for p in misses]
            k.integrate((x, y), hits, misses)
            a, b = audit(m, k)
            fw += a
            fo += b
            nb = m.neighbours(at)
            fwd = [c for c in nb if c != prev] or nb
            prev, at = at, rng.choice(fwd)
    check('no passage mistaken for a wall, with a stale pose on every scan',
          not fw, fw[:4])
    check('no wall mistaken for a passage, with a stale pose on every scan',
          not fo, fo[:4])


def _rotate(p, about, ang):
    """Rotate a point about another - simulates fusing with a stale heading."""
    c, s = math.cos(ang), math.sin(ang)
    dx, dy = p[0] - about[0], p[1] - about[1]
    return (about[0] + dx * c - dy * s, about[1] + dx * s + dy * c)


def test_coverage_grows():
    print('\nmapping progresses rather than plateauing')
    m = Maze(cols=12, rows=12, seed=77, braid=0.2)
    boxes = aabbs(m)
    k = Knowledge(m.to_meta())
    # The strong version of 'the mapper accumulates': scan from EVERY cell and
    # the map must be complete. A random walk was tried first and asserted
    # nothing useful - 40 random steps revisit so heavily that they reach about
    # 20 of 144 cells, and the 22 percent that produces is a fact about random
    # walks, not about the mapper.
    marks = []
    for r in range(m.rows):
        for c in range(m.cols):
            x, y = m.cell_centre(c, r)
            k.integrate((x, y), *fake_scan(m, boxes, x, y, 0.0))
            marks.append(k.coverage())
    check('coverage after 1 scan is small (%.0f%%)' % (marks[0] * 100),
          marks[0] < 0.35)
    check('a scan from every cell maps every wall (%.1f%%)' % (marks[-1] * 100),
          marks[-1] > 0.995,
          'some edge is never resolved from either side - a blind spot')
    check('coverage never goes down', all(marks[i] <= marks[i + 1] + 1e-12
                                          for i in range(len(marks) - 1)))


def test_search_is_unmodified():
    """The same functions, on the other kind of map. No special cases anywhere."""
    print('\nthe identical search code runs on a Knowledge and on a Maze')
    m = Maze(cols=10, rows=10, seed=8, rough=0.4)
    k = Knowledge(m.to_meta())
    boxes = aabbs(m)
    for cell in [(c, r) for r in range(0, 10, 3) for c in range(0, 10, 3)]:
        x, y = m.cell_centre(*cell)
        k.integrate((x, y), *fake_scan(m, boxes, x, y, 0.0))
    bad = []
    for fn in (bfs, ucs, astar):
        r = fn(k, k.start, k.goal)
        if not r.path:
            bad.append(fn.__name__)
        for i in range(len(r.path) - 1):
            if k._state(r.path[i], r.path[i + 1]) == WALL:
                bad.append('%s routed through a known wall' % fn.__name__)
    check('bfs, ucs and a* all plan on a partial map', not bad, bad[:3])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mazes', type=int, default=8)
    ap.add_argument('--verbose', action='store_true')
    a = ap.parse_args()

    print('=' * 68)
    print('discovery mode - simulated lidar, %d mazes' % a.mazes)
    print('=' * 68)

    test_knowledge_hides_the_maze()
    test_optimism()
    test_evidence()
    test_single_scan_is_truthful(a.mazes)
    test_survives_pose_lag(min(a.mazes, 6))
    test_coverage_grows()
    test_search_is_unmodified()
    test_full_discovery_run(a.mazes, a.verbose)

    print()
    print('=' * 68)
    if FAILED:
        print('%d FAILED: %s' % (len(FAILED), ', '.join(FAILED)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
