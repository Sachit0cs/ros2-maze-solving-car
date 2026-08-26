#!/usr/bin/env python3
"""The search algorithms. No ROS, no Gazebo, no maze - just a graph.

Every algorithm here takes the same three things:

    g       anything with .neighbours(cell), .edge_cost(a, b), .heuristic(c, goal)
    start   a hashable node
    goal    a hashable node

and returns the same Result. That uniformity is the point: if BFS counted
expansions one way and A* another, the comparison table in the control panel
would be measuring the bookkeeping instead of the algorithms. So there is
exactly one definition of every number, applied to all seven:

  expanded      a node is EXPANDED when it is popped off the frontier and its
                neighbours are generated. Nodes that are merely pushed are not
                expanded. This is the standard convention and it is the one
                that makes 'A* expands fewer nodes than UCS' a true statement.
  max_frontier  the largest the open list ever got. This is the memory axis,
                and it is where BFS loses to DFS as dramatically as DFS loses
                to BFS on path quality.
  cost          in cell-times, summed with g.edge_cost. What the car actually
                spends.
  steps         number of moves. What BFS minimises.

DETERMINISM

Neighbours are sorted before use and every priority queue carries a
monotonically increasing counter as its final tie-break. Two runs of the same
algorithm on the same maze therefore expand the same nodes in the same order,
which is what makes the animation reproducible and the numbers quotable.
Without the counter, heapq falls back to comparing the node tuples themselves,
which silently makes the tie-break 'smallest (c, r)' - a different algorithm
with different numbers, and one you would never notice was happening.

WHAT EACH ONE GUARANTEES

  bfs             fewest STEPS. Not fewest seconds unless terrain is uniform.
  dfs             a path. No optimality of any kind. Cheap memory.
  bidirectional   fewest steps, same as bfs, from roughly half the expansions.
  ucs             least COST. The ground truth every other result is scored on.
  greedy          nothing. Fast, and often wrong.
  astar           least COST, with an admissible heuristic - and this one's is.
  wall_follower   a path in a simply-connected maze, and only there.

scripts/test_search.py checks each of those claims numerically over hundreds of
random mazes rather than asking you to believe this docstring.
"""
import heapq
import time
from collections import deque


class Result:
    """What a search returns. One shape for all of them."""

    def __init__(self, name, path=None, cost=0.0, expanded=None, sides=None,
                 max_frontier=0, ms=0.0, note=''):
        self.name = name
        self.path = path or []
        self.cost = cost
        self.expanded = expanded or []
        # which front expanded each node: 0 = from the start, 1 = from the goal.
        # Only bidirectional ever uses 1, but every result carries the list so
        # the animation has one code path.
        self.sides = sides if sides is not None else [0] * len(self.expanded)
        self.max_frontier = max_frontier
        self.ms = ms
        self.note = note

    @property
    def found(self):
        return len(self.path) > 1 or (len(self.path) == 1)

    @property
    def steps(self):
        return max(0, len(self.path) - 1)

    @property
    def n_expanded(self):
        return len(self.expanded)

    def as_dict(self):
        return {'name': self.name, 'found': self.found,
                'path': [list(p) for p in self.path],
                'cost': round(self.cost, 3), 'steps': self.steps,
                'expanded': [list(p) for p in self.expanded],
                'sides': self.sides,
                'n_expanded': self.n_expanded,
                'max_frontier': self.max_frontier,
                'ms': round(self.ms, 3), 'note': self.note}


def _reconstruct(came, node):
    path = [node]
    while node in came:
        node = came[node]
        path.append(node)
    path.reverse()
    return path


def _cost_of(g, path):
    return sum(g.edge_cost(path[i], path[i + 1]) for i in range(len(path) - 1))


def _timed(fn):
    """Wrap an algorithm so every one is timed identically."""
    def run(g, start, goal, **kw):
        t0 = time.perf_counter()
        res = fn(g, start, goal, **kw)
        res.ms = (time.perf_counter() - t0) * 1000.0
        return res
    run.__name__ = fn.__name__
    run.__doc__ = fn.__doc__
    return run


# ------------------------------------------------------------------ uninformed

@_timed
def bfs(g, start, goal):
    """Breadth-first search: expand the whole frontier one ring at a time.

    Finds the path with the FEWEST STEPS, always, because it reaches every node
    at its minimum depth before ever looking deeper. It knows nothing about
    cost, so on a maze with mud in it BFS will happily return the short slow
    route while UCS takes the long fast one. Watching those two disagree is the
    clearest demonstration of what 'optimal' does and does not mean.

    Nodes are marked seen when PUSHED, not when popped. On an unweighted graph
    that is safe - the first time BFS reaches a node is already via a shortest
    path - and it stops the queue filling with duplicates of the same node.
    """
    seen = {start}
    came = {}
    q = deque([start])
    expanded, peak = [], 1
    while q:
        node = q.popleft()
        expanded.append(node)
        if node == goal:
            path = _reconstruct(came, node)
            return Result('bfs', path, _cost_of(g, path), expanded,
                          max_frontier=peak)
        for nb in sorted(g.neighbours(node)):
            if nb not in seen:
                seen.add(nb)
                came[nb] = node
                q.append(nb)
        peak = max(peak, len(q))
    return Result('bfs', [], 0.0, expanded, max_frontier=peak,
                  note='no path')


@_timed
def dfs(g, start, goal):
    """Depth-first search: follow one branch to the end before trying another.

    Returns *a* path, and makes no promise whatsoever about its length or cost.
    On a recursive-backtracker maze, whose corridors are long and lonely, DFS
    often stumbles onto the goal almost immediately and looks brilliant;
    regenerate with Prim's and it looks awful. Both are the same algorithm.

    THE MEMORY CLAIM DOES NOT SURVIVE A MAZE. The textbook sells DFS on space:
    O(b*m) against BFS's O(b^d), so the stack should be the small one. Measured
    here, on square perfect mazes, 20 seeds each, it is the large one and the
    gap widens with size:

        10x10   bfs peak  4.2    dfs peak   5.5
        20x20   bfs peak  6.0    dfs peak  11.2
        40x40   bfs peak  9.6    dfs peak  29.7

    Not duplicate entries - the deduplicated stack measures the same. The bound
    is simply the wrong one for this shape of graph. b*m beats b^d only when b
    is big enough for b^d to run away; a maze has b of about 1.2 and a depth m
    of hundreds of cells, so DFS pays for the length of its branch while BFS's
    ring stays as narrow as the corridors are. Students should see this,
    because 'DFS is the memory-efficient one' is a fact about trees with fat
    branching that gets repeated as though it were a fact about search.

    Marked seen on POP rather than on push. A node can sit in the stack several
    times over before it is reached; committing to a parent at push time would
    record the first push's parent and reconstruct a path the search did not
    actually take.
    """
    seen = set()
    came = {}
    stack = [start]
    expanded, peak = [], 1
    while stack:
        node = stack.pop()
        if node in seen:
            continue
        seen.add(node)
        expanded.append(node)
        if node == goal:
            path = _reconstruct(came, node)
            return Result('dfs', path, _cost_of(g, path), expanded,
                          max_frontier=peak)
        # reversed so that the smallest neighbour is popped first and the
        # exploration order matches the direction BFS would have gone
        for nb in sorted(g.neighbours(node), reverse=True):
            if nb not in seen:
                came[nb] = node
                stack.append(nb)
        peak = max(peak, len(stack))
    return Result('dfs', [], 0.0, expanded, max_frontier=peak, note='no path')


@_timed
def bidirectional(g, start, goal):
    """Two breadth-first searches, one from each end, meeting in the middle.

    Why it wins: a breadth-first search out to depth d touches roughly b^d
    nodes. Two searches of depth d/2 touch 2 * b^(d/2), which for any
    interesting d is very much smaller. The control panel's expansion count
    makes that concrete - typically a third to a half of plain BFS on the same
    maze - and the animation shows the two fronts growing toward each other.

    THE OFF-BY-ONE THAT MAKES IT WRONG. The obvious stopping rule is 'halt the
    moment the two frontiers touch, and join through the node that touched'.
    That rule is not correct. The first contact can be through a node whose
    combined depth is one more than the best meeting available at that same
    level. So this expands whole LEVELS, alternating sides, and when a level
    completes with a non-empty intersection it takes the minimum of
    d_forward + d_backward over EVERY node both sides have reached - not over
    the one that happened to touch first. test_search.py compares the step
    count against bfs on several hundred random mazes, which is the only
    reason to believe any of this.

    It minimises STEPS, not cost - it is two breadth-first searches, and the
    terrain is invisible to both of them. On a maze with mud, bidirectional can
    and does return a route slower than the one UCS finds. That is the
    algorithm being what it is, not a defect.
    """
    if start == goal:
        return Result('bidirectional', [start], 0.0, [start], max_frontier=1)

    dist = [{start: 0}, {goal: 0}]
    came = [{}, {}]
    front = [deque([start]), deque([goal])]
    expanded, sides, peak = [], [], 2

    def join(meet):
        fwd = _reconstruct(came[0], meet)
        back = _reconstruct(came[1], meet)
        back.reverse()
        return fwd + back[1:]

    turn = 0
    while front[0] and front[1]:
        # expand exactly one level on this side
        for _ in range(len(front[turn])):
            node = front[turn].popleft()
            expanded.append(node)
            sides.append(turn)
            for nb in sorted(g.neighbours(node)):
                if nb not in dist[turn]:
                    dist[turn][nb] = dist[turn][node] + 1
                    came[turn][nb] = node
                    front[turn].append(nb)
        peak = max(peak, len(front[0]) + len(front[1]))

        both = set(dist[0]) & set(dist[1])
        if both:
            meet = min(both, key=lambda n: (dist[0][n] + dist[1][n], n))
            path = join(meet)
            return Result('bidirectional', path, _cost_of(g, path), expanded,
                          sides, peak, note='met at %s' % (meet,))
        turn ^= 1
    return Result('bidirectional', [], 0.0, expanded, sides, peak,
                  note='no path')


@_timed
def ucs(g, start, goal):
    """Uniform-cost search (Dijkstra): always expand the cheapest node so far.

    This is the ground truth. Every 'optimal?' column in the comparison table
    is a comparison against the cost UCS returned, because UCS is optimal on
    any graph with non-negative edge costs and needs no heuristic to be so.

    On a maze where every cell is plain, UCS expands in exactly the same order
    as BFS and returns exactly the same path - all edges cost 1.0, so 'cheapest
    so far' and 'shallowest so far' are the same question. Turn the roughness
    up and they diverge immediately. That is the cleanest way to show a student
    what the difference between the two algorithms actually is: it is not the
    data structure, it is the key.
    """
    seen = set()
    best = {start: 0.0}
    came = {}
    pq = [(0.0, 0, start)]
    tie = 0
    expanded, peak = [], 1
    while pq:
        cost, _, node = heapq.heappop(pq)
        if node in seen:
            continue                 # a stale copy, already expanded cheaper
        seen.add(node)
        expanded.append(node)
        if node == goal:
            path = _reconstruct(came, node)
            return Result('ucs', path, cost, expanded, max_frontier=peak)
        for nb in sorted(g.neighbours(node)):
            nc = cost + g.edge_cost(node, nb)
            if nc < best.get(nb, float('inf')) - 1e-12:
                best[nb] = nc
                came[nb] = node
                tie += 1
                heapq.heappush(pq, (nc, tie, nb))
        peak = max(peak, len(pq))
    return Result('ucs', [], 0.0, expanded, max_frontier=peak, note='no path')


# -------------------------------------------------------------------- informed

@_timed
def greedy(g, start, goal):
    """Greedy best-first: expand whatever LOOKS closest to the goal.

    Priority is h alone - the cost already paid is thrown away. That is what
    makes it fast and what makes it wrong: it will dive into a dead end that
    points at the goal and refuse to consider the corridor that heads away
    first. Its path is not optimal in cost, and it is not optimal in steps
    either.

    It is in this project as the cautionary half of a pair. Run greedy and A*
    on the same maze and the numbers say it plainly - greedy usually expands
    fewer nodes, and usually returns a worse path. A* is what you get by adding
    the one term greedy discarded.
    """
    seen = set()
    came = {}
    tie = 0
    pq = [(g.heuristic(start, goal), 0, start)]
    expanded, peak = [], 1
    while pq:
        _, _, node = heapq.heappop(pq)
        if node in seen:
            continue
        seen.add(node)
        expanded.append(node)
        if node == goal:
            path = _reconstruct(came, node)
            return Result('greedy', path, _cost_of(g, path), expanded,
                          max_frontier=peak)
        for nb in sorted(g.neighbours(node)):
            if nb not in seen and nb not in came:
                came[nb] = node
                tie += 1
                heapq.heappush(pq, (g.heuristic(nb, goal), tie, nb))
        peak = max(peak, len(pq))
    return Result('greedy', [], 0.0, expanded, max_frontier=peak,
                  note='no path')


@_timed
def astar(g, start, goal):
    """A*: expand by cost-so-far plus estimated cost-to-go.

    f(n) = g(n) + h(n). UCS is this with h = 0; greedy is this with g dropped.
    Seeing all three side by side in one table is the single most useful thing
    in this project, because it shows that they are not three algorithms but
    one algorithm with three choices of key.

    Optimal here because the heuristic is admissible - Manhattan distance times
    the cheapest possible edge, which cannot overestimate - and consistent, so
    a node is never reached more cheaply after it has been expanded and the
    closed set is safe. test_search.py verifies admissibility and consistency
    numerically on random terrain rather than assuming them, because an
    inadmissible heuristic does not crash: it just quietly returns a worse path
    than UCS while still looking like A*.
    """
    seen = set()
    best = {start: 0.0}
    came = {}
    tie = 0
    pq = [(g.heuristic(start, goal), 0, 0.0, start)]
    expanded, peak = [], 1
    while pq:
        _, _, cost, node = heapq.heappop(pq)
        if node in seen:
            continue
        seen.add(node)
        expanded.append(node)
        if node == goal:
            path = _reconstruct(came, node)
            return Result('astar', path, cost, expanded, max_frontier=peak)
        for nb in sorted(g.neighbours(node)):
            nc = cost + g.edge_cost(node, nb)
            if nc < best.get(nb, float('inf')) - 1e-12:
                best[nb] = nc
                came[nb] = node
                tie += 1
                heapq.heappush(pq, (nc + g.heuristic(nb, goal), tie, nc, nb))
        peak = max(peak, len(pq))
    return Result('astar', [], 0.0, expanded, max_frontier=peak,
                  note='no path')


# --------------------------------------------------------------- non-search

@_timed
def wall_follower(g, start, goal, max_steps=20000):
    """Left-hand rule: keep your left hand on the wall and walk.

    Not a search. It holds no frontier, no visited set and no map - at each
    cell it turns as far left as it can and moves, which is a rule a robot can
    execute with nothing but a lidar and no memory at all. maze_solver's
    wall_follower ROS node is exactly this, driven off the live scan.

    It is here because it is the honest baseline, and because the exact shape
    of its failure is more interesting than 'it sometimes fails'.

    The left hand traces the boundary of ONE connected component of wall. It
    reaches the goal if and only if the goal touches the component it started
    on. In a perfect maze every wall is joined to the outer boundary, so there
    is only one component and it always works. Braiding removes wall segments
    and can detach an interior island from that boundary - but that alone is
    not enough, and this is where the obvious statement of the rule is wrong.
    Measured over 80 mazes per cell:

                        goal at a corner      goal at the centre
        braid 0.0            0/80 trapped          0/80 trapped
        braid 0.5            0/80 trapped         46/80 trapped
        braid 1.0            0/80 trapped         71/80 trapped

    Loops are irrelevant when the start and the goal are both corners, because
    both sit on the outer boundary - the same wall component - so the hand is
    already touching the wall that leads to the goal. The rule fails when the
    goal is in the INTERIOR, ringed by an island the outer trace never meets;
    then the car circles that outer wall forever with the goal a metre away
    through a gap it will not take.

    Which is why the default goal is a corner - the baseline should work out of
    the box - and why the control panel lets you drag it to the middle. Drag it
    there on a braided maze and the wall follower fails while every search in
    this module still solves it. That is the argument for building a map,
    made in ten seconds instead of a lecture.

    The path it returns REVISITS cells, so its 'steps' is a distance travelled
    rather than a path length, and it is the only row in the table where those
    two are different numbers.
    """
    heading = 0                       # 0 E, 1 N, 2 W, 3 S
    delta = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    nb = g.neighbours(start)
    if nb:
        d = (nb[0][0] - start[0], nb[0][1] - start[1])
        heading = delta.index(d)

    node = start
    path = [start]
    expanded = [start]
    for _ in range(max_steps):
        if node == goal:
            return Result('wall_follower', path, _cost_of(g, path), expanded,
                          max_frontier=1, note='%d cells walked' % len(path))
        # left, straight, right, back - the first one that is open
        for turn in (1, 0, -1, 2):
            h = (heading + turn) % 4
            step = (node[0] + delta[h][0], node[1] + delta[h][1])
            if step in g.neighbours(node):
                heading = h
                node = step
                path.append(node)
                expanded.append(node)
                break
        else:
            break                     # walled in on all four sides
    return Result('wall_follower', [], 0.0, expanded, max_frontier=1,
                  note='gave up after %d cells - looped forever?' % len(expanded))


ALGORITHMS = {
    'bfs': ('Breadth-first', bfs, 'fewest steps; ignores terrain cost'),
    'dfs': ('Depth-first', dfs, 'any path, tiny memory, no guarantees'),
    'bidirectional': ('Bidirectional BFS', bidirectional,
                      'fewest steps, from about half the expansions'),
    'ucs': ('Uniform-cost (Dijkstra)', ucs, 'least cost; the ground truth'),
    'greedy': ('Greedy best-first', greedy, 'fast, and often wrong'),
    'astar': ('A*', astar, 'least cost, guided; usually the one to ship'),
    'wall_follower': ('Wall-follower (left hand)', wall_follower,
                      'no map at all; fails on loops'),
}

# The order the comparison table is presented in: uninformed first, then
# informed, then the no-map baseline. It is a lesson plan, not an alphabet.
ORDER = ['bfs', 'dfs', 'bidirectional', 'ucs', 'greedy', 'astar',
         'wall_follower']


def run(name, g, start, goal, **kw):
    if name not in ALGORITHMS:
        raise KeyError('no algorithm called %r' % name)
    return ALGORITHMS[name][1](g, start, goal, **kw)


def compare(g, start, goal, names=None):
    """Run every algorithm on one maze and score them against UCS.

    'optimal' means 'returned a path of the same COST as UCS', to a tolerance,
    because that is the only definition under which the informed and the
    uninformed searches can be compared at all. BFS being step-optimal is a
    separate column, not a substitute.
    """
    names = names or ORDER
    out = []
    truth = ucs(g, start, goal)
    for n in names:
        r = run(n, g, start, goal)
        d = r.as_dict()
        d['label'] = ALGORITHMS[n][0]
        d['blurb'] = ALGORITHMS[n][2]
        d['optimal'] = bool(r.found and truth.found
                            and abs(r.cost - truth.cost) < 1e-6)
        d['excess'] = (round(r.cost - truth.cost, 3)
                       if r.found and truth.found else None)
        out.append(d)
    return {'rows': out, 'best_cost': round(truth.cost, 3),
            'best_steps': truth.steps}
