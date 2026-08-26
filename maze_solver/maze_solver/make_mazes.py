#!/usr/bin/env python3
"""Generate the teaching set: seven mazes, each of which makes one point.

    python3 -m maze_solver.make_mazes --out ~/maze_solver_ws/mazes

These are not seven random mazes. Random mazes are mostly the same maze, and
the first thing a student sees if you hand them one is that every algorithm
returns an identical path - which is TRUE on a perfect maze with flat ground,
and is the least interesting true thing in the subject.

Each maze here is built so that one specific comparison becomes visible:

  tiny       every algorithm agrees. Start here, so that later disagreement
             is obviously a property of the maze and not of the code.
  classic    a proper hedge maze. BFS and DFS return the same path and do
             wildly different amounts of work getting there.
  terrain    mud. This is where BFS and UCS stop agreeing: the fewest cells
             and the fewest seconds are different routes.
  braided    loops, flat ground. Several genuinely different routes, so DFS's
             path is visibly worse rather than merely differently found.
  trap       loops AND the goal moved to the middle. The wall follower circles
             forever; every search still solves it. This is the one that
             justifies the whole project.
  open       barely a maze - mostly open floor. Greedy best-first looks
             brilliant here and A* looks slow, which is the honest shape of
             that trade-off when the heuristic is well informed.
  big        25x25 with mud. Scale: where the expansion counts separate by
             thousands rather than by tens.

Every one is deterministic. A student and a lecturer running this on different
machines get identical mazes and can compare identical numbers.
"""
import argparse
import json
import os

from maze_solver.maze import Maze
from maze_solver.maze_gen import write_maze

LESSONS = [
    ('tiny', dict(cols=6, rows=6, seed=1, generator='backtracker'),
     'Every algorithm returns the same path. One route exists, so only the '
     'work differs.'),
    ('classic', dict(cols=15, rows=15, seed=12, generator='backtracker'),
     'A proper hedge maze. Same path from everything; compare nodes expanded '
     'and peak frontier.'),
    # BRAIDED, not perfect, and that is the whole point of this one. The first
    # version was a perfect maze with mud in it, which demonstrates nothing:
    # a perfect maze has exactly one route, so BFS and UCS return the same path
    # at the same cost no matter how much mud is on it. Terrain only matters
    # where there is a CHOICE. These parameters were searched for: BFS's route
    # is 14 cells SHORTER and 24 cell-times MORE EXPENSIVE than UCS's.
    ('terrain', dict(cols=15, rows=15, seed=12, generator='backtracker',
                     braid=0.35, rough=0.70),
     'Mud, and loops to route around it. BFS finds a route 14 cells shorter '
     'that takes 41 seconds longer; UCS and A* find the fast one. This is what '
     '"optimal" not meaning "shortest" looks like.'),
    ('braided', dict(cols=15, rows=15, seed=31, generator='prim', braid=0.7),
     'Loops, so several real routes exist. DFS now returns a visibly worse '
     'path, not just a differently-found one.'),
    ('trap', dict(cols=15, rows=15, seed=44, generator='prim', braid=0.9,
                  goal=(7, 7)),
     'Loops and an interior goal. The wall follower circles forever; every '
     'search in the module still solves it.'),
    ('open', dict(cols=14, rows=14, seed=57, generator='prim', braid=1.0),
     'Almost open floor. Greedy best-first shines and A* pays for its '
     'optimality - the honest shape of that trade-off.'),
    ('big', dict(cols=25, rows=25, seed=68, generator='backtracker',
                 braid=0.2, rough=0.5),
     'Scale. Expansion counts separate by thousands here, not by tens.'),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser('~/maze_solver_ws/mazes'))
    ap.add_argument('--only', default='', help='generate just this one')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    index = []
    print('%-10s %-9s %-8s %-6s %-6s  %s' % ('name', 'size', 'kind', 'slow',
                                             'junc', 'lesson'))
    for name, kw, lesson in LESSONS:
        if a.only and a.only != name:
            continue
        path = os.path.join(a.out, 'maze_%s.sdf' % name)
        m = Maze(**kw)
        meta = write_maze(path, maze=m)
        # the lesson travels with the maze so the control panel can show it
        meta['lesson'] = lesson
        meta['preset'] = name
        with open(path[:-4] + '.json', 'w') as f:
            json.dump(meta, f, indent=2)
        st = meta['stats']
        print('%-10s %-9s %-8s %-6d %-6d  %s'
              % (name, '%dx%d' % (m.cols, m.rows),
                 'perfect' if st['perfect'] else 'braided',
                 st['slow_cells'], st['junctions'], lesson[:44]))
        index.append({'name': 'maze_' + name, 'preset': name, 'lesson': lesson})

    with open(os.path.join(a.out, 'index.json'), 'w') as f:
        json.dump(index, f, indent=2)
    print('\n%d mazes in %s' % (len(index), a.out))


if __name__ == '__main__':
    main()
