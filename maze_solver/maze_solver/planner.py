#!/usr/bin/env python3
"""The planner: runs a search algorithm and publishes the path to drive.

  ros2 run maze_solver planner --ros-args -p meta_path:=<maze>.json \
       -p algorithm:=astar -p mode:=known

Publishes:
  /plan        nav_msgs/Path    cell centres in world coordinates
  /plan/stats  std_msgs/String  JSON: expansions, frontier peak, cost, ms

THE ONLY DIFFERENCE BETWEEN THE TWO MODES IS WHICH OBJECT IS SEARCHED

  known      searches a Maze - the real one, loaded from the .json
  discovery  searches a Knowledge - what the mapper has worked out so far

Both expose .neighbours, .edge_cost and .heuristic, so search.py is called
identically and has no idea which it is looking at. Nothing in this file
branches on the algorithm either. That is the design: a student swapping
`astar` for `bfs` in the control panel changes one dictionary lookup, and the
robot's behaviour changes completely.

WHEN IT REPLANS

In known mode: once. The map cannot change, so a second search would return
the same answer and the honest expansion count is the one from the single run
that produced the path being driven.

In discovery mode: whenever the map has changed AND the current path is no
longer valid - that is, a wall has been discovered lying across a step the
plan intends to take. Replanning on every map update instead would work, and
would inflate the expansion count by a factor of thirty while teaching nothing;
replanning only on invalidation is what makes 'how many replans did it need'
a number worth putting on the screen.
"""
import json
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from maze_solver.knowledge import Knowledge
from maze_solver.maze import Maze
from maze_solver.qos import EPISODE_QOS
from maze_solver.search import ALGORITHMS, run


class Planner(Node):
    def __init__(self):
        super().__init__('planner')

        self.declare_parameter('meta_path', '')
        self.declare_parameter('algorithm', 'astar')
        self.declare_parameter('mode', 'known')       # known | discovery
        self.declare_parameter('plan_file', '/tmp/ms_plan.json')

        g = self.get_parameter
        meta_path = g('meta_path').value
        self.algo = g('algorithm').value
        self.mode = g('mode').value
        self.plan_file = g('plan_file').value

        if self.algo not in ALGORITHMS:
            raise SystemExit('unknown algorithm %r - try one of %s'
                             % (self.algo, ', '.join(sorted(ALGORITHMS))))
        if not meta_path or not os.path.exists(meta_path):
            raise SystemExit('planner needs meta_path=<maze>.json')
        with open(meta_path) as f:
            self.meta = json.load(f)

        self.maze = Maze.from_meta(self.meta)         # geometry, always
        self.goal = tuple(self.meta['goal'])
        # In discovery mode the graph searched is the Knowledge, which starts
        # empty. self.maze is still loaded because cell_centre and
        # world_to_cell are geometry, not knowledge - the car is told the size
        # of the arena, not the shape of the maze inside it.
        self.known = Knowledge(self.meta) if self.mode == 'discovery' else None

        self.pub_plan = self.create_publisher(Path, 'plan', 10)
        self.pub_stats = self.create_publisher(String, 'plan/stats', 10)
        self.create_subscription(PoseStamped, 'car/world_pose', self.on_pose, 10)
        self.create_subscription(Bool, 'episode/active', self.on_active, EPISODE_QOS)
        if self.mode == 'discovery':
            self.create_subscription(String, 'maze/known', self.on_known, 10)
        self.create_timer(0.5, self.tick)

        self.cell = None
        self.active = False
        self.path = []
        self.replans = 0
        self.total_expanded = 0
        self.total_ms = 0.0
        self.last_revision = -1
        self.last_result = None
        self.planned_for_episode = False

        self.get_logger().info(
            'planner up: %s in %s mode, %dx%d maze, goal %s'
            % (ALGORITHMS[self.algo][0], self.mode, self.maze.cols,
               self.maze.rows, self.goal))
        if self.mode == 'known':
            self.get_logger().info('  %s' % ALGORITHMS[self.algo][2])

    # ------------------------------------------------------------- callbacks

    def on_pose(self, m):
        self.cell = self.maze.world_to_cell(m.pose.position.x,
                                            m.pose.position.y)

    def on_active(self, m):
        # Logged on TRANSITIONS only. It is published at 5 Hz, so logging every
        # message drowns the console - but the transitions are exactly what you
        # need when the planner is replanning and you cannot see why.
        if m.data != self.active:
            self.get_logger().info('episode/active -> %s' % m.data)
        if m.data and not self.active:
            self.planned_for_episode = False          # a new run: plan again
        if not m.data:
            self.path = []
        self.active = m.data

    def on_known(self, m):
        try:
            self.known.load_state(json.loads(m.data))
        except (ValueError, TypeError) as e:
            self.get_logger().warn('bad map update: %s' % e)

    # ------------------------------------------------------------------ plan

    def graph(self):
        return self.known if self.mode == 'discovery' else self.maze

    def needs_plan(self):
        if not self.active or self.cell is None:
            return False
        if self.cell == self.goal:
            return False
        if self.mode == 'known':
            return not self.planned_for_episode
        # discovery: only when something was learned AND it broke the plan
        if not self.path:
            return True
        if self.known.revision == self.last_revision:
            return False
        self.last_revision = self.known.revision
        if not self.known.path_is_still_valid(self.path):
            return True
        # the car left the plan (pushed off line, or a step was refused)
        return self.cell not in [tuple(p) for p in self.path]

    def tick(self):
        if not self.needs_plan():
            return
        if self.mode == 'known' and self.replans:
            # In known mode this must never happen: the map cannot change, so
            # a second search can only mean the episode was restarted. Say so
            # loudly rather than quietly reporting a lower expansion count than
            # the search that actually produced the driven path.
            self.get_logger().warn(
                'replanning in KNOWN mode (#%d) from %s - episode restarted?'
                % (self.replans + 1, self.cell))
        g = self.graph()
        res = run(self.algo, g, self.cell, self.goal)
        self.replans += 1
        self.total_expanded += res.n_expanded
        self.total_ms += res.ms
        self.planned_for_episode = True
        self.last_result = res

        if not res.path:
            self.get_logger().warn(
                'no path from %s to %s - %s expanded %d nodes and found nothing'
                % (self.cell, self.goal, self.algo, res.n_expanded))
            self.path = []
            self.write_plan(res)
            return

        self.path = [tuple(p) for p in res.path]
        self.publish_path(self.path)
        self.publish_stats(res)
        self.write_plan(res)

        if self.mode == 'known':
            self.get_logger().info(
                '%s: %d cells, cost %.1f, expanded %d, peak frontier %d, %.1f ms'
                % (self.algo, res.steps, res.cost, res.n_expanded,
                   res.max_frontier, res.ms))
        else:
            self.get_logger().info(
                'replan #%d from %s: %d cells ahead, expanded %d, %.0f%% mapped'
                % (self.replans, self.cell, res.steps, res.n_expanded,
                   100.0 * self.known.coverage()))

    def publish_path(self, cells):
        msg = Path()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        for c, r in cells:
            x, y = self.maze.cell_centre(c, r)
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.pub_plan.publish(msg)

    def publish_stats(self, res):
        self.pub_stats.publish(String(data=json.dumps({
            'algorithm': self.algo, 'mode': self.mode,
            'steps': res.steps, 'cost': round(res.cost, 3),
            'expanded': res.n_expanded, 'max_frontier': res.max_frontier,
            'ms': round(res.ms, 3), 'replans': self.replans})))

    def write_plan(self, res):
        """Everything the control panel needs to animate the search.

        The expansion ORDER is the payload here, not the path - it is what lets
        the panel replay the frontier growing, which is the part students
        actually learn from. On a 40x40 maze it is at most 1600 pairs, a few
        tens of kilobytes, written once per plan.
        """
        try:
            body = res.as_dict()
            body.update({
                'algorithm': self.algo,
                'label': ALGORITHMS[self.algo][0],
                'mode': self.mode,
                'replans': self.replans,
                'total_expanded': self.total_expanded,
                'total_ms': round(self.total_ms, 2),
                'from': list(self.cell) if self.cell else None,
                'goal': list(self.goal),
            })
            if self.mode == 'discovery':
                body['known'] = self.known.as_dict()
            tmp = self.plan_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(body, f)
            os.replace(tmp, self.plan_file)
        except Exception:                             # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = Planner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
