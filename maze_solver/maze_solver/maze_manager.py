#!/usr/bin/env python3
"""Episode manager: respawn the car, and score what happened.

Publishes:
  /car/world_pose  PoseStamped   ground truth, republished for everyone else
  /episode/active  Bool          False for a moment after a respawn
  /episode/event   String        goal | wall | stuck

PROGRESS IS MEASURED IN COST, NOT METRES AND NOT CELLS

The road projects measured progress as straight-line distance to the goal,
which on a road is very nearly arc length along it. In a maze it is nearly
meaningless: the goal can be two metres away through a wall and forty cells
away by road, so a car making excellent progress reads as making none, and a
car that has blundered into the cell next to the goal reads as nearly finished.

So this builds a distance field once, from the goal over the TRUE maze, and
progress is how far down that field the car has come. The field holds COST, not
cells - see _distance_field for the run that proved cells were wrong. Briefly:
a cost-optimal route detours around mud, a detour is a retreat when you count
hops, and a flawless run got scored 'stuck'.

'stuck' therefore means no reduction in cost-to-goal for stuck_seconds. Cells
are still computed, and still shown, because 'nine cells from the goal' is what
a human wants to read - but nothing is scored on them.

The manager is the one node that is allowed to know the true maze even in
discovery mode. It is the referee, not a player - it never publishes the field
and nothing that drives subscribes to it.
"""
import heapq
import json
import math
import os
import subprocess
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from maze_solver.maze import Maze
from maze_solver.qos import EPISODE_QOS, SENSOR_QOS


class MazeManager(Node):
    def __init__(self):
        super().__init__('maze_manager')

        self.declare_parameter('meta_path', '')
        self.declare_parameter('world_name', 'maze')
        self.declare_parameter('model_name', 'maze_car')
        self.declare_parameter('state_file', '/tmp/ms_ego.json')
        # 0.085 m from the lidar. The car is 160 mm long with the lidar 30 mm
        # ahead of centre, so its nose is 50 mm from the lidar - below the
        # sensor's own 60 mm minimum. There is no threshold that sees the
        # instant of contact; this one fires just before it.
        self.declare_parameter('crash_distance', 0.085)
        self.declare_parameter('stuck_seconds', 30.0)
        self.declare_parameter('settle_seconds', 1.5)
        self.declare_parameter('max_episodes', 0)     # 0 = unlimited

        g = self.get_parameter
        meta_path = g('meta_path').value
        self.world = g('world_name').value
        self.model = g('model_name').value
        self.state_file = g('state_file').value
        self.crash_d = float(g('crash_distance').value)
        self.stuck_t = float(g('stuck_seconds').value)
        self.settle = float(g('settle_seconds').value)
        self.max_ep = int(g('max_episodes').value)

        if not meta_path or not os.path.exists(meta_path):
            raise SystemExit('maze_manager needs meta_path=<maze>.json')
        with open(meta_path) as f:
            meta = json.load(f)
        self.meta = meta
        self.maze = Maze.from_meta(meta)
        self.start_pose = meta['start_pose']
        self.goal_xy = meta['goal_pose']
        self.goal_r = float(meta['goal_radius'])

        self.field = self._distance_field()          # cost, for scoring
        self.hops = self._hop_field()                # cells, for the display
        self.d_start = self.field.get(self.maze.start, 1.0)
        self.hop_start = self.hops.get(self.maze.start, 1)

        self.pub_active = self.create_publisher(Bool, 'episode/active', EPISODE_QOS)
        self.pub_event = self.create_publisher(String, 'episode/event', EPISODE_QOS)
        self.pub_stop = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_pose = self.create_publisher(PoseStamped, 'car/world_pose', 10)
        self.create_subscription(Odometry, 'ego/true_odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, SENSOR_QOS)
        self.create_timer(0.2, self.tick)

        self.pos = None
        self.yaw = 0.0
        self.min_range = 99.0
        self.active = False
        self.settle_from = None
        self.best_d = None
        self.last_progress_t = None
        self.warned_no_odom = False
        self.n_goal = self.n_wall = self.n_stuck = 0
        self.t_started = None
        self.last_report = 0.0
        self.last_log = 0.0
        self.finish_times = []

        self.get_logger().info(
            'maze_manager: %dx%d maze, start %s -> goal %s, %d cells and '
            '%.0f cell-times apart'
            % (self.maze.cols, self.maze.rows, self.maze.start, self.maze.goal,
               self.hop_start, self.d_start))
        self.respawn('init')

    def _distance_field(self):
        """Least COST from every cell to the goal. One Dijkstra, at startup.

        This used to be a breadth-first flood counting CELLS, and that was
        wrong in a way that matters more here than almost anywhere else: it
        made the referee assume that shortest is best, which is the assumption
        this entire project exists to refute.

        The failure was clean. Uniform-cost search on maze_terrain returns an
        80-cell route that detours around the mud, in preference to the 66-cell
        route through it. The car drove that detour perfectly, and its
        cells-to-goal went 66, 64, 62, 61, 58, then 60, 61, 62 - because a
        detour is, in hops, a retreat. The manager watched the number stop
        improving and scored a flawless run as 'stuck' after 45.8 s.

        Cost-to-goal does not have that problem: a car following a cost-optimal
        route reduces it by exactly the cost of every step it takes. And it is
        still planner-independent, which the alternative - progress along the
        published path - would not have been, because the wall follower has no
        published path and still has to be scored.
        """
        d = {self.maze.goal: 0.0}
        pq = [(0.0, self.maze.goal)]
        while pq:
            cost, n = heapq.heappop(pq)
            if cost > d.get(n, float('inf')) + 1e-12:
                continue
            for nb in self.maze.neighbours(n):
                nc = cost + self.maze.edge_cost(n, nb)
                if nc < d.get(nb, float('inf')) - 1e-12:
                    d[nb] = nc
                    heapq.heappush(pq, (nc, nb))
        return d

    def _hop_field(self):
        """Fewest cells to the goal. Display only - never used for scoring."""
        d = {self.maze.goal: 0}
        q = deque([self.maze.goal])
        while q:
            n = q.popleft()
            for nb in self.maze.neighbours(n):
                if nb not in d:
                    d[nb] = d[n] + 1
                    q.append(nb)
        return d

    def on_odom(self, m):
        """Ground-truth world pose - see the URDF for why it is not /odom."""
        q = m.pose.pose.orientation
        p = m.pose.pose.position
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pos = (p.x, p.y)

        ps = PoseStamped()
        ps.header.stamp = m.header.stamp
        ps.header.frame_id = 'world'
        ps.pose.position.x = p.x
        ps.pose.position.y = p.y
        ps.pose.orientation.z = math.sin(self.yaw / 2.0)
        ps.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pub_pose.publish(ps)

    def on_scan(self, m):
        v = [r for r in m.ranges if 0.02 < r < 100.0]
        self.min_range = min(v) if v else 99.0

    def respawn(self, why):
        self.pub_active.publish(Bool(data=False))
        self.pub_stop.publish(Twist())
        x, y, th = self.start_pose
        # z = 0.005 for the same reason the launch file uses it: base_footprint
        # is ground contact, so anything higher drops the car onto free wheels
        # and it skids away from the start line.
        req = ('name: "%s", position: {x: %f, y: %f, z: 0.005}, '
               'orientation: {x: 0, y: 0, z: %f, w: %f}'
               % (self.model, x, y, math.sin(th / 2.0), math.cos(th / 2.0)))
        try:
            subprocess.run(
                ['gz', 'service', '-s', '/world/%s/set_pose' % self.world,
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '3000', '--req', req],
                capture_output=True, timeout=10)
        except Exception as e:                        # noqa: BLE001
            self.get_logger().warn('respawn failed: %s' % e)
        # None means 'start counting on the next tick', NOT 'now + settle'.
        # This node runs on sim time, and at construction /clock has not
        # arrived yet, so get_clock().now() is 0 - which put hold_until at 1.5
        # while sim time was already 40, and the settle period was skipped
        # entirely. The episode then began while the car was still moving from
        # the teleport.
        self.settle_from = None
        self.pos = None
        self.active = False
        self.best_d = None
        self.last_progress_t = None

    def finish(self, why, t):
        self.pub_event.publish(String(data=why))
        took = (t - self.t_started) if self.t_started else 0.0
        if why == 'goal':
            self.n_goal += 1
            self.finish_times.append(took)
        elif why == 'wall':
            self.n_wall += 1
        else:
            self.n_stuck += 1
        tot = self.n_goal + self.n_wall + self.n_stuck
        self.get_logger().info(
            'episode %d: %-5s in %5.1f s | goal %d  wall %d  stuck %d  (%.0f%% solved)'
            % (tot, why, took, self.n_goal, self.n_wall, self.n_stuck,
               100.0 * self.n_goal / max(tot, 1)))
        self.write_state(progress=None)
        if self.max_ep and tot >= self.max_ep:
            self.get_logger().info('EPISODE_BUDGET_DONE')
        self.respawn(why)

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if not self.active:
            if self.settle_from is None:
                # first tick after a respawn - NOW is when the clock is real
                self.settle_from = t
                return
            # Hold the wheels at zero for the whole settle period. The
            # DiffDrive plugin only regulates a joint once it has been given a
            # command; until then the wheels free-spin, and a car that landed
            # with any momentum keeps rolling. Publishing zero repeatedly is
            # what actually stops it - the single zero in respawn() was sent
            # before the bridge had connected.
            self.pub_stop.publish(Twist())
            self.pub_active.publish(Bool(data=False))
            if (t - self.settle_from) < self.settle:
                return
            if self.pos is None:
                # The settle is over and there is still no pose, so the episode
                # cannot start. Say so - loudly and repeatedly.
                #
                # This warning used to live in the ACTIVE branch, where it was
                # unreachable in precisely this situation: no pose means never
                # active means the warning never prints. A run then sat in
                # perfect silence for 280 s with the manager's startup banner
                # as the only clue, and 'nothing finished' as the only symptom.
                # A diagnostic that cannot fire in the case it describes is
                # worse than none, because its absence reads as reassurance.
                if (t - self.settle_from) > 8.0 and (t - self.last_log) > 5.0:
                    self.last_log = t
                    self.get_logger().warn(
                        'waiting for /ego/true_odom - %.0f s and no pose yet. '
                        'Is the car spawned, and is the topic bridged?'
                        % (t - self.settle_from))
                return
            self.active = True
            self.last_progress_t = t
            self.t_started = t
            self.pub_active.publish(Bool(data=True))
            return
        if self.pos is None:
            if (not self.warned_no_odom and self.last_progress_t
                    and (t - self.last_progress_t) > 10.0):
                self.warned_no_odom = True
                self.get_logger().warn(
                    'no pose after 10 s - is /ego/true_odom bridged?')
            return

        self.pub_active.publish(Bool(data=True))

        if math.dist(self.pos, tuple(self.goal_xy)) < self.goal_r:
            self.finish('goal', t)
            return
        if self.min_range < self.crash_d:
            self.finish('wall', t)
            return

        cell = self.maze.world_to_cell(*self.pos)
        d = self.field.get(cell, self.d_start)              # cost to goal
        hops = self.hops.get(cell, self.hop_start)          # cells to goal
        pct = 100.0 * (1.0 - d / max(self.d_start, 1e-6))

        if t - self.last_report > 0.5:
            self.last_report = t
            self.write_state(progress=pct, cells_to_goal=hops,
                             cost_to_goal=round(d, 1), cell=cell,
                             elapsed=t - (self.t_started or t))
        if t - self.last_log > 5.0:
            self.last_log = t
            self.get_logger().info(
                'progress %3.0f%%   %d cells / %.0f cell-times to goal   at %s'
                % (pct, hops, d, cell))

        # 'stuck' is no reduction in CELLS to the goal. A car shuffling about
        # inside one cell is not making progress no matter how far it drives,
        # and a car reversing out of a dead end is - it is on its way to a
        # lower number. Metres travelled cannot tell those apart.
        # 'stuck' is no reduction in COST to the goal. A car detouring around
        # mud is making progress even though it is getting further away in
        # cells; a car shuffling inside one cell is not, however far it drives.
        if self.best_d is None or d < self.best_d - 1e-9:
            self.best_d = d
            self.last_progress_t = t
        elif self.last_progress_t and (t - self.last_progress_t) > self.stuck_t:
            self.finish('stuck', t)

    def write_state(self, progress=None, cells_to_goal=None, cost_to_goal=None,
                    cell=None, elapsed=None):
        """Drop the ego's state where the control panel can read it.

        The panel is a plain http.server rather than a ROS node, so it picks
        the run up from files. Written to a temp file and renamed, so a reader
        never catches half a file.
        """
        try:
            body = {'x': self.pos[0] if self.pos else None,
                    'y': self.pos[1] if self.pos else None,
                    'yaw': self.yaw, 'active': self.active,
                    'progress': progress, 'cells_to_goal': cells_to_goal,
                    'cost_to_goal': cost_to_goal,
                    'cell': list(cell) if cell else None,
                    'elapsed': round(elapsed, 1) if elapsed else None,
                    'goal_n': self.n_goal, 'wall_n': self.n_wall,
                    'stuck_n': self.n_stuck,
                    'times': [round(v, 1) for v in self.finish_times[-8:]]}
            tmp = self.state_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(body, f)
            os.replace(tmp, self.state_file)
        except Exception:                             # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = MazeManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass          # Ctrl-C, or a supervisor sent SIGTERM - both are normal
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
