#!/usr/bin/env python3
"""The left-hand rule, driven off the live lidar. No map, no plan, no memory.

  ros2 run maze_solver wall_follower

Subscribes /scan and /episode/active. Publishes /cmd_vel. That is the entire
interface, and the shortness of that list is the point: this node does not know
where it is, where the goal is, how big the maze is, or what it did a second
ago. It is the honest baseline every search algorithm in this project is
measured against, and it is what a maze robot looked like before anybody put a
graph in one.

THE RULE

Keep your left hand on the wall and walk. Three cases, checked in order,
because the order IS the rule:

  1. blocked ahead        turn right on the spot - the hand stays on the wall
  2. the wall on the left is gone   turn left and drive into the gap, or the
                          hand comes off the wall and the rule is broken
  3. otherwise            hold the left wall at half a corridor

Case 2 is the one people get wrong when they write this from memory. The
tempting version is 'if there is space on the left, keep going straight and
come back to it' - which walks straight past every left turn in the maze and
degenerates into a very slow random walk.

WHAT IT CANNOT DO

It reaches the goal if and only if the goal touches the wall component its hand
started on. In a perfect maze there is only one such component and it always
works. Give the maze loops AND put the goal in the interior and it will circle
the outer wall forever with the goal a metre away. search.py's wall_follower
docstring has the measured table; scripts/test_search.py asserts both halves.

That failure is the argument for the rest of this project.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from maze_solver.lidar_utils import bearings, clean, sector_mean, sector_min
from maze_solver.qos import EPISODE_QOS, SENSOR_QOS


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Defaults suit the default 0.62 m corridor. The node is NOT given the
        # maze metadata - a wall follower that reads the maze file is not a
        # wall follower - so the corridor width is a parameter it is told, the
        # way you would tell a real robot the width of the corridors it is
        # about to be put in.
        self.declare_parameter('corridor', 0.62)
        self.declare_parameter('v_max', 0.32)
        self.declare_parameter('w_max', 2.0)
        self.declare_parameter('k_wall', 3.2)
        self.declare_parameter('front_stop', 0.24)
        self.declare_parameter('hand', 'left')        # left | right

        g = self.get_parameter
        self.corridor = float(g('corridor').value)
        self.v_max = float(g('v_max').value)
        self.w_max = float(g('w_max').value)
        self.k_wall = float(g('k_wall').value)
        self.front_stop = float(g('front_stop').value)
        self.hand = 1.0 if g('hand').value == 'left' else -1.0

        self.target = self.corridor / 2.0
        # The wall is 'lost' when the wedge reads further than the far side of
        # the NEXT cell could be. Held at half a corridor the wall is 0.31 m
        # away; if the passage beside the car is open the nearest return in
        # that wedge is a full cell further. Anything past 0.9 corridors is
        # unambiguously an opening.
        self.lost = self.corridor * 0.90

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, SENSOR_QOS)
        self.create_subscription(Bool, 'episode/active', self.on_active, EPISODE_QOS)
        self.active = False
        self.state = ''
        self.get_logger().info(
            'wall_follower up: %s hand, holding %.2f m, corridor %.2f m'
            % (g('hand').value, self.target, self.corridor))

    def on_active(self, m):
        if not m.data:
            self.pub.publish(Twist())
        self.active = m.data

    def on_scan(self, msg):
        if not self.active:
            return
        rs = clean(msg)
        angs = bearings(msg)
        h = self.hand
        side = sector_mean(rs, angs, h * math.pi / 2.0, 0.35)
        diag = sector_min(rs, angs, h * math.pi / 4.0, 0.26)
        front = sector_min(rs, angs, 0.0, 0.30)

        cmd = Twist()
        if front < self.front_stop:
            # 1. blocked ahead: spin away from the hand, staying on the wall
            state = 'blocked - turning %s' % ('right' if h > 0 else 'left')
            cmd.linear.x = 0.0
            cmd.angular.z = -h * self.w_max * 0.8
        elif side > self.lost and diag > self.lost:
            # 2. the wall fell away: follow it round rather than sail past it
            state = 'opening - turning %s' % ('left' if h > 0 else 'right')
            cmd.linear.x = self.v_max * 0.45
            cmd.angular.z = h * 1.5
        else:
            # 3. hold station against the wall
            state = 'following'
            err = min(0.30, max(-0.30, side - self.target))
            w = h * self.k_wall * err
            # ... but do not let a corner ahead of the hand be run into while
            # correcting: the diagonal ray sees it before the side wedge does
            if diag < self.target:
                w -= h * self.k_wall * (self.target - diag)
            w = max(-self.w_max, min(self.w_max, w))
            cmd.linear.x = self.v_max * max(0.25, 1.0 - 0.6 * abs(w) / self.w_max)
            cmd.angular.z = float(w)

        if state != self.state:
            self.state = state
            self.get_logger().info(
                '%s   (side %.2f  diag %.2f  front %.2f)'
                % (state, side, diag, front))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = WallFollower()
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
