#!/usr/bin/env python3
"""Run every algorithm on every teaching maze and print the tables.

    python3 scripts/compare_algos.py              # the seven teaching mazes
    python3 scripts/compare_algos.py --markdown   # ready to paste into a README
    python3 scripts/compare_algos.py --maze terrain

No ROS, no Gazebo - seconds, not hours. The numbers in the README come from
here, so they can be regenerated after any change rather than being remembered.

The 'seconds' column is not a separate measurement. Cost is in cell-times and
path_driver drives one plain cell at v_max, so seconds = cost * pitch / v_max
exactly. That identity is the whole point of the cost model: what the planner
minimises is what the stopwatch measures.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from maze_solver.make_mazes import LESSONS                    # noqa: E402
from maze_solver.maze import Maze                             # noqa: E402
from maze_solver.search import ALGORITHMS, ORDER, compare     # noqa: E402

V_MAX = 0.40          # must match path_driver's default


def table(maze, name, markdown=False):
    tbl = compare(maze, maze.start, maze.goal)
    rows = []
    for r in tbl['rows']:
        secs = r['cost'] * maze.pitch / V_MAX
        rows.append((ALGORITHMS[r['name']][0], r['n_expanded'], r['max_frontier'],
                     r['steps'] if r['found'] else None, r['cost'] if r['found'] else None,
                     secs if r['found'] else None, r['ms'],
                     'yes' if r['optimal'] else ('+%.0f' % r['excess']
                                                 if r['found'] else 'never')))
    head = ('algorithm', 'expanded', 'peak', 'cells', 'cost', 'seconds', 'ms',
            'optimal')
    if markdown:
        print('\n**%s** — %dx%d, %s, %d slow cells\n'
              % (name, maze.cols, maze.rows,
                 'perfect' if maze.is_perfect() else 'braided',
                 maze.stats()['slow_cells']))
        print('| ' + ' | '.join(head) + ' |')
        print('|' + '---|' * len(head))
        for r in rows:
            print('| %s | %d | %d | %s | %s | %s | %.2f | %s |'
                  % (r[0], r[1], r[2],
                     r[3] if r[3] is not None else '—',
                     '%.0f' % r[4] if r[4] is not None else '—',
                     '%.0f' % r[5] if r[5] is not None else '—', r[6], r[7]))
    else:
        print('\n%s  (%dx%d, %s, %d slow cells)'
              % (name, maze.cols, maze.rows,
                 'perfect' if maze.is_perfect() else 'braided',
                 maze.stats()['slow_cells']))
        print('  %-24s %8s %6s %6s %7s %8s %7s  %s' % head)
        for r in rows:
            print('  %-24s %8d %6d %6s %7s %8s %7.2f  %s'
                  % (r[0], r[1], r[2],
                     r[3] if r[3] is not None else '-',
                     '%.0f' % r[4] if r[4] is not None else '-',
                     '%.0f' % r[5] if r[5] is not None else '-', r[6], r[7]))
    return tbl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--markdown', action='store_true')
    ap.add_argument('--maze', default='', help='just one of the teaching mazes')
    a = ap.parse_args()

    totals = {n: {'exp': 0, 'peak': 0, 'opt': 0, 'runs': 0, 'ms': 0.0}
              for n in ORDER}
    for name, kw, _lesson in LESSONS:
        if a.maze and a.maze != name:
            continue
        m = Maze(**kw)
        tbl = table(m, name, a.markdown)
        for r in tbl['rows']:
            t = totals[r['name']]
            t['exp'] += r['n_expanded']
            t['peak'] = max(t['peak'], r['max_frontier'])
            t['opt'] += 1 if r['optimal'] else 0
            t['ms'] += r['ms']
            t['runs'] += 1

    runs = max(totals['bfs']['runs'], 1)
    print('\n\nAcross all %d mazes' % runs)
    if a.markdown:
        print('\n| algorithm | total expanded | worst frontier | optimal on | total ms |')
        print('|---|---|---|---|---|')
        for n in ORDER:
            t = totals[n]
            print('| %s | %d | %d | %d/%d | %.1f |'
                  % (ALGORITHMS[n][0], t['exp'], t['peak'], t['opt'], runs, t['ms']))
    else:
        print('  %-24s %14s %14s %10s %9s'
              % ('algorithm', 'total expanded', 'worst frontier', 'optimal on', 'total ms'))
        for n in ORDER:
            t = totals[n]
            print('  %-24s %14d %14d %8d/%d %9.1f'
                  % (ALGORITHMS[n][0], t['exp'], t['peak'], t['opt'], runs, t['ms']))


if __name__ == '__main__':
    main()
