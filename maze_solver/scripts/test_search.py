#!/usr/bin/env python3
"""Checks on the search algorithms. No ROS, no Gazebo, no simulator.

    python3 scripts/test_search.py [--mazes 40]

Runs in a couple of seconds, which is the entire reason the algorithms live in
a module that imports nothing. Every claim the README and the docstrings make
about optimality is checked here numerically, against a REFERENCE
implementation written differently on purpose:

  * shortest cost is verified against Bellman-Ford - repeated edge relaxation,
    no priority queue - so a bug in the heap discipline cannot hide by being
    present in both the thing and the check.
  * shortest step count is verified against a plain ring-by-ring flood fill.

The heuristic gets its own section. An inadmissible heuristic does not crash
and does not look wrong; A* simply returns a slightly worse path than UCS while
still being A*. The only way to catch it is to compute the true remaining cost
from every cell and compare, which is what test_heuristic does.
"""
import argparse
import os
import sys
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from maze_solver.maze import Maze                          # noqa: E402
from maze_solver.search import (ALGORITHMS, ORDER, astar, bfs,  # noqa: E402
                                bidirectional, compare, dfs, greedy, run, ucs,
                                wall_follower)

FAILED = []
INF = float('inf')


def check(name, cond, detail=''):
    if cond:
        print('  ok    %s' % name)
    else:
        print('  FAIL  %s   %s' % (name, detail))
        FAILED.append(name)


# ------------------------------------------------------- reference answers

def bellman_ford(m, src):
    """True least cost from src to everywhere. No heap, no visited set."""
    dist = {(c, r): INF for r in range(m.rows) for c in range(m.cols)}
    dist[src] = 0.0
    for _ in range(len(dist)):
        changed = False
        for node, d in list(dist.items()):
            if d == INF:
                continue
            for nb in m.neighbours(node):
                nd = d + m.edge_cost(node, nb)
                if nd < dist[nb] - 1e-12:
                    dist[nb] = nd
                    changed = True
        if not changed:
            break
    return dist


def flood_depth(m, src):
    """True fewest steps from src to everywhere. Plain ring-by-ring flood."""
    depth = {src: 0}
    q = deque([src])
    while q:
        n = q.popleft()
        for nb in m.neighbours(n):
            if nb not in depth:
                depth[nb] = depth[n] + 1
                q.append(nb)
    return depth


def valid_path(m, path, start, goal):
    """Is this a path a car could actually drive?"""
    if not path:
        return False, 'empty'
    if path[0] != start:
        return False, 'starts at %s not %s' % (path[0], start)
    if path[-1] != goal:
        return False, 'ends at %s not %s' % (path[-1], goal)
    for i in range(len(path) - 1):
        if path[i + 1] not in m.neighbours(path[i]):
            return False, 'step %d: %s -> %s crosses a wall' % (
                i, path[i], path[i + 1])
    return True, ''


def mazes(n, rough=0.0, braid=0.0, gens=('backtracker', 'prim')):
    for i in range(n):
        yield Maze(cols=6 + (i % 9), rows=6 + ((i * 3) % 9), seed=1000 + i,
                   generator=gens[i % len(gens)], braid=braid, rough=rough)


# -------------------------------------------------------------------- tests

def test_paths_are_drivable(n):
    print('\nevery algorithm returns a path a car could drive')
    bad = []
    for m in mazes(n, rough=0.5, braid=0.3):
        for name in ORDER:
            r = run(name, m, m.start, m.goal)
            ok, why = valid_path(m, r.path, m.start, m.goal)
            if not ok:
                bad.append('%s on seed %d: %s' % (name, m.seed, why))
    check('all %d algorithms x %d mazes produce legal paths' % (len(ORDER), n),
          not bad, bad[:3])


def test_bfs_is_step_optimal(n):
    print('\nbfs minimises steps')
    bad = []
    for m in mazes(n, rough=0.6, braid=0.4):
        truth = flood_depth(m, m.start).get(m.goal)
        got = bfs(m, m.start, m.goal).steps
        if truth != got:
            bad.append('seed %d: flood says %s, bfs says %s'
                       % (m.seed, truth, got))
    check('bfs step count matches an independent flood fill', not bad, bad[:3])


def test_ucs_is_cost_optimal(n):
    print('\nucs minimises cost')
    bad = []
    for m in mazes(n, rough=0.7, braid=0.4):
        truth = bellman_ford(m, m.start)[m.goal]
        got = ucs(m, m.start, m.goal)
        if abs(truth - got.cost) > 1e-9:
            bad.append('seed %d: bellman-ford %.4f, ucs %.4f'
                       % (m.seed, truth, got.cost))
        # the cost it reports must also be the cost of the path it returns
        if abs(got.cost - m.path_cost(got.path)) > 1e-9:
            bad.append('seed %d: reported cost is not the path cost' % m.seed)
    check('ucs cost matches bellman-ford, and matches its own path', not bad,
          bad[:3])


def test_astar_matches_ucs(n):
    print('\na* is optimal, greedy and dfs are not required to be')
    bad = []
    greedy_worse = dfs_worse = 0
    for m in mazes(n, rough=0.7, braid=0.4):
        best = ucs(m, m.start, m.goal).cost
        a = astar(m, m.start, m.goal).cost
        if abs(a - best) > 1e-9:
            bad.append('seed %d: ucs %.4f, a* %.4f' % (m.seed, best, a))
        if greedy(m, m.start, m.goal).cost > best + 1e-9:
            greedy_worse += 1
        if dfs(m, m.start, m.goal).cost > best + 1e-9:
            dfs_worse += 1
    check('a* returns exactly the ucs cost on every maze', not bad, bad[:3])
    # Not a failure if these are zero on a perfect maze - there is only one
    # path - but with braid and rough turned up it should be common, and if it
    # never happens the terrain is not actually doing anything.
    check('greedy is measurably worse somewhere (%d/%d mazes)'
          % (greedy_worse, n), greedy_worse > 0,
          'terrain or braiding may not be taking effect')
    print('        dfs returned a worse-than-optimal path on %d/%d mazes'
          % (dfs_worse, n))


def test_bidirectional_matches_bfs(n):
    print('\nbidirectional finds the same step count as bfs, for less work')
    bad = []
    saved = []
    for m in mazes(n, rough=0.4, braid=0.35):
        b = bfs(m, m.start, m.goal)
        d = bidirectional(m, m.start, m.goal)
        if b.steps != d.steps:
            bad.append('seed %d: bfs %d steps, bidirectional %d'
                       % (m.seed, b.steps, d.steps))
        if b.n_expanded:
            saved.append(1.0 - d.n_expanded / b.n_expanded)
    check('bidirectional step count equals bfs on every maze', not bad, bad[:3])
    mean = sum(saved) / max(len(saved), 1)
    check('and it expands fewer nodes on average (%.0f%% fewer)' % (mean * 100),
          mean > 0.0, 'mean saving %.3f' % mean)


def test_heuristic(n):
    print('\nthe heuristic is admissible and consistent')
    over = []
    incons = []
    for m in mazes(n, rough=0.8, braid=0.4):
        true_to_goal = bellman_ford(m, m.goal)      # symmetric graph, so this
        for r in range(m.rows):                     # is also cost-to-goal
            for c in range(m.cols):
                cell = (c, r)
                h = m.heuristic(cell)
                t = true_to_goal[cell]
                if t < INF and h > t + 1e-9:
                    over.append('seed %d %s: h=%.3f > true %.3f'
                                % (m.seed, cell, h, t))
                for nb in m.neighbours(cell):
                    if h - m.heuristic(nb) > m.edge_cost(cell, nb) + 1e-9:
                        incons.append('seed %d %s->%s' % (m.seed, cell, nb))
    check('h never overestimates the true remaining cost', not over, over[:3])
    check('h(a) - h(b) <= cost(a, b) for every edge', not incons, incons[:3])
    check('h(goal) == 0', abs(Maze(seed=3).heuristic((0, 0), (0, 0))) < 1e-12)


def test_wall_follower(n):
    """The left-hand rule, and the exact condition under which it fails.

    'It breaks on a maze with loops' is the version everyone repeats and it is
    not true - it breaks when the GOAL is not on the wall component the hand
    started tracing. With start and goal both at corners of a rectangle they
    are on the same component (the outer boundary) and loops change nothing.
    This asserts both halves, because getting only the first half right is how
    you end up with a demo that never demonstrates anything.
    """
    print('\nwall-follower: the failure is about the goal, not about loops')
    unsolved = [m.seed for m in mazes(n, braid=0.0)
                if not wall_follower(m, m.start, m.goal).path]
    check('reaches a corner goal on every perfect maze, with no map at all',
          not unsolved, unsolved[:5])

    corner = sum(1 for m in mazes(60, braid=1.0)
                 if not wall_follower(m, m.start, m.goal).path)
    check('a corner goal is still reached even at braid=1.0 (%d/60 trapped)'
          % corner, corner == 0,
          'start and goal share the outer wall component - it cannot fail')

    def centre_trapped(braid):
        t = 0
        for i in range(60):
            m = Maze(cols=13, rows=13, seed=3000 + i, braid=braid)
            m.goal = (m.cols // 2, m.rows // 2)
            if not wall_follower(m, m.start, m.goal).path:
                t += 1
        return t

    clean, loopy = centre_trapped(0.0), centre_trapped(1.0)
    check('an interior goal in a PERFECT maze is still reached (%d/60 trapped)'
          % clean, clean == 0, 'one wall component, so it must succeed')
    check('an interior goal in a braided maze traps it (%d/60 trapped)' % loopy,
          loopy > 20, 'braiding is not detaching any wall islands')


def test_memory_axis(n):
    """DFS's memory advantage, and the fact that a maze does not grant it.

    The textbook bound is O(b*m) for DFS against O(b^d) for BFS, so the stack
    should be the small one. In a maze it is not: b is about 1.2, so b^d never
    runs away, while m - the length of a branch - is most of the maze. This
    asserts the MEASURED relationship rather than the quoted one, and asserts
    that the gap widens with size, because that is the part that shows it is
    structural and not a fluke of one seed.
    """
    print('\ndfs vs bfs frontier: the textbook bound does not apply here')
    ratios = []
    for m in mazes(n):
        b = bfs(m, m.start, m.goal).max_frontier
        d = dfs(m, m.start, m.goal).max_frontier
        if b:
            ratios.append(d / b)
    mean = sum(ratios) / max(len(ratios), 1)
    check('dfs holds a LARGER frontier than bfs on a maze (ratio %.2f)' % mean,
          mean > 1.0,
          'if this ever drops below 1, the README table needs regenerating')

    grew = []
    for size in (10, 20, 40):
        pb = pd = 0
        for i in range(12):
            m = Maze(cols=size, rows=size, seed=200 + i)
            pb += bfs(m, m.start, m.goal).max_frontier
            pd += dfs(m, m.start, m.goal).max_frontier
        grew.append((size, pb / 12.0, pd / 12.0))
        print('        %2dx%-2d   bfs peak %5.1f   dfs peak %5.1f'
              % (size, size, pb / 12.0, pd / 12.0))
    check('and the gap widens with maze size',
          (grew[-1][2] / grew[-1][1]) > (grew[0][2] / grew[0][1]),
          [(s, round(d / b, 2)) for s, b, d in grew])


def test_informed_beats_uninformed(n):
    print('\ninformed search expands less than uninformed')
    tot_ucs = tot_astar = tot_greedy = 0
    for m in mazes(n, rough=0.5):
        tot_ucs += ucs(m, m.start, m.goal).n_expanded
        tot_astar += astar(m, m.start, m.goal).n_expanded
        tot_greedy += greedy(m, m.start, m.goal).n_expanded
    check('a* expands fewer nodes than ucs in total (%d vs %d)'
          % (tot_astar, tot_ucs), tot_astar < tot_ucs)
    check('greedy expands fewer than a* in total (%d vs %d)'
          % (tot_greedy, tot_astar), tot_greedy < tot_astar)


def test_determinism():
    print('\nthe same maze twice gives the same expansion order')
    m = Maze(cols=14, rows=11, seed=99, braid=0.4, rough=0.6)
    bad = []
    for name in ORDER:
        a = run(name, m, m.start, m.goal)
        b = run(name, m, m.start, m.goal)
        if a.expanded != b.expanded or a.path != b.path:
            bad.append(name)
    check('all %d algorithms are reproducible' % len(ORDER), not bad, bad)


def test_no_path():
    print('\na sealed goal is reported as unreachable, not crashed into')
    m = Maze(cols=8, rows=8, seed=5)
    # brick the goal cell in on all four sides
    gc, gr = m.goal
    for r in range(m.rows):
        for c in range(m.cols - 1):
            if (c, r) == (gc - 1, gr) or (c, r) == (gc, gr):
                m.open_h[r][c] = False
    for r in range(m.rows - 1):
        for c in range(m.cols):
            if c == gc and r in (gr - 1, gr):
                m.open_v[r][c] = False
    bad = []
    for name in ORDER:
        r = run(name, m, m.start, m.goal)
        if r.path:
            bad.append('%s claimed a path to a sealed cell' % name)
    check('every algorithm reports no path rather than raising', not bad, bad)


def test_compare_table():
    print('\nthe comparison table the control panel renders')
    # braided and rough on purpose. On a perfect maze with flat ground every
    # row of this table is identical, which is true, correct, and teaches
    # nothing - the whole table has to be able to disagree with itself.
    m = Maze(cols=15, rows=15, seed=4242, braid=0.7, rough=0.7)
    tbl = compare(m, m.start, m.goal)
    check('one row per algorithm', len(tbl['rows']) == len(ORDER))
    check('ucs and a* are both flagged optimal',
          all(r['optimal'] for r in tbl['rows']
              if r['name'] in ('ucs', 'astar')))
    check('best_cost agrees with the ucs row',
          abs(tbl['best_cost'] -
              [r for r in tbl['rows'] if r['name'] == 'ucs'][0]['cost']) < 1e-9)
    print()
    print('    %-22s %7s %7s %8s %9s %8s  %s'
          % ('algorithm', 'expand', 'peak', 'steps', 'cost', 'ms', 'optimal'))
    for r in tbl['rows']:
        print('    %-22s %7d %7d %8d %9.2f %8.2f  %s'
              % (r['label'], r['n_expanded'], r['max_frontier'], r['steps'],
                 r['cost'], r['ms'], 'yes' if r['optimal'] else '-'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mazes', type=int, default=40)
    a = ap.parse_args()
    n = a.mazes

    print('=' * 68)
    print('search algorithms - %d random mazes per property' % n)
    print('=' * 68)

    test_paths_are_drivable(n)
    test_bfs_is_step_optimal(n)
    test_ucs_is_cost_optimal(n)
    test_astar_matches_ucs(n)
    test_bidirectional_matches_bfs(n)
    test_heuristic(min(n, 20))
    test_wall_follower(n)
    test_memory_axis(n)
    test_informed_beats_uninformed(n)
    test_determinism()
    test_no_path()
    test_compare_table()

    print()
    print('=' * 68)
    if FAILED:
        print('%d FAILED: %s' % (len(FAILED), ', '.join(FAILED)))
        return 1
    print('all checks passed  (%d algorithms)' % len(ALGORITHMS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
