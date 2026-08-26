#!/usr/bin/env python3
"""Discovery mode: build a map of the maze from the lidar, as the car drives.

  ros2 run maze_solver mapper --ros-args -p meta_path:=<maze>.json

Subscribes /scan, /car/world_pose, /car/terrain, /episode/active.
Publishes  /maze/known  std_msgs/String - the map so far, as JSON.

WHAT IT IS TOLD, AND WHAT IT IS NOT

It is handed the maze's .json, and takes exactly four things from it: how many
cells across, how many down, how big a cell is, and where the goal is. The
walls are never read. Knowledge enforces that structurally - it builds its
geometry from a copy of the metadata with every passage blanked - so this node
could not cheat if it wanted to.

That is the right amount of prior knowledge for the demonstration to mean
something. A robot dropped into an arena generally does know how big the arena
is and what it is looking for; it does not know the shape of the walls inside.

WHY A JSON STRING AND NOT A CUSTOM MESSAGE

An ament_python package cannot generate its own messages without a CMake
package alongside it, and adding one to carry a grid of small integers would be
the most complicated part of the project by some distance. A 20x20 maze is 760
edges - a few kilobytes of JSON, published only when something is actually
learned rather than at a fixed rate. The same trick the control panel already
uses to read the running simulation.

RATE

Scans arrive at 10 Hz and each one is 180 raycasts folded into the grid. The
map is republished only when the revision counter moves, which after the first
few seconds is rare - most scans re-observe walls already known. That is what
keeps a node doing real work at 10 Hz from flooding a topic at 10 Hz.
"""
import json
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from maze_solver.knowledge import Knowledge
from maze_solver.lidar_utils import bearings, clean
from maze_solver.qos import EPISODE_QOS, SENSOR_QOS


class Mapper(Node):
    def __init__(self):
        super().__init__('mapper')

        self.declare_parameter('meta_path', '')
        # The lidar sits 30 mm ahead of the pose frame. That is small until you
        # notice a cell is 680 mm and this node's whole job is deciding which
        # side of a lattice line a return fell on - 30 mm is 4 percent of a
        # cell, and it decides which of two walls gets marked.
        self.declare_parameter('lidar_dx', 0.030)
        self.declare_parameter('map_file', '/tmp/ms_map.json')
        self.declare_parameter('min_publish_period', 0.2)
        # Two gates on what a scan is allowed to teach, both there because the
        # error a moving car makes is ANGULAR, and an angular error grows with
        # range.
        #
        # max_yaw_rate: skip the scan entirely while spinning faster than this.
        # The pose fused with a scan can be a frame old; at the driver's 2.2
        # rad/s ceiling that is 3.4 degrees of bearing error on every ray at
        # once - a correlated error, which is the kind an evidence filter is
        # worst at rejecting. Cheaper to throw the scan away: the car turns in
        # place for well under a second and then scans from a standstill.
        #
        # max_range: 5 cells. Beyond that even a correct 1-degree calibration
        # error puts a return on the wrong side of a lattice line, and a
        # corridor five cells long has already been mapped by the near rays.
        self.declare_parameter('max_yaw_rate', 0.6)
        self.declare_parameter('max_range_cells', 5.0)

        g = self.get_parameter
        meta_path = g('meta_path').value
        self.lidar_dx = float(g('lidar_dx').value)
        self.map_file = g('map_file').value
        self.min_period = float(g('min_publish_period').value)
        self.max_yaw_rate = float(g('max_yaw_rate').value)
        self.max_range_cells = float(g('max_range_cells').value)

        if not meta_path or not os.path.exists(meta_path):
            raise SystemExit('mapper needs meta_path=<maze>.json')
        with open(meta_path) as f:
            meta = json.load(f)
        # Knowledge takes dimensions and blanks the passages itself.
        self.known = Knowledge(meta)

        self.pub_map = self.create_publisher(String, 'maze/known', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, SENSOR_QOS)
        self.create_subscription(PoseStamped, 'car/world_pose', self.on_pose, 10)
        self.create_subscription(Float32, 'car/terrain', self.on_terrain, 10)
        self.create_subscription(Bool, 'episode/active', self.on_active, EPISODE_QOS)

        self.pose = None
        self.last_pose_t = None
        self.yaw_rate = 0.0
        self.skipped = 0
        self.active = False
        self.last_rev = -1
        self.last_pub = 0.0
        self.last_log = 0.0
        self.scans = 0
        self.get_logger().info('mapper up: %dx%d cells, everything unknown'
                               % (self.known.cols, self.known.rows))

    def on_pose(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        t = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if self.pose is not None and self.last_pose_t is not None:
            dt = t - self.last_pose_t
            if dt > 1e-4:
                d = math.atan2(math.sin(yaw - self.pose[2]),
                               math.cos(yaw - self.pose[2]))
                # low-passed: a single 33 ms sample of yaw rate is noisy enough
                # to gate scans on and off at random
                self.yaw_rate = 0.7 * self.yaw_rate + 0.3 * abs(d / dt)
        self.last_pose_t = t
        self.pose = (m.pose.position.x, m.pose.position.y, yaw)

    def on_terrain(self, m):
        """The car felt the ground it is on. Terrain is invisible to a lidar."""
        if self.pose is None:
            return
        cell = self.known.cell_of(self.pose[0], self.pose[1])
        self.known.learn_terrain(cell, float(m.data))

    def on_active(self, m):
        if m.data and not self.active:
            # A fresh episode starts from a blank map. Carrying the previous
            # run's map over would make the second attempt at a maze trivially
            # better than the first and quietly ruin every discovery number.
            self.known = Knowledge(self.known_meta())
            self.last_rev = -1
            self.scans = 0
            self.skipped = 0
            self.get_logger().info('episode start - map cleared')
        self.active = m.data

    def known_meta(self):
        """Rebuild the dimension-only metadata Knowledge needs."""
        return {'cols': self.known.cols, 'rows': self.known.rows,
                'corridor': self.known.geom.corridor,
                'wall_thickness': self.known.wall_thickness,
                'wall_height': self.known.geom.wall_height,
                'seed': 0, 'start': list(self.known.start),
                'goal': list(self.known.goal)}

    def on_scan(self, msg):
        if not self.active or self.pose is None:
            return
        if self.yaw_rate > self.max_yaw_rate:
            self.skipped += 1
            return                        # spinning: see max_yaw_rate above
        x, y, yaw = self.pose
        rs = clean(msg)
        angs = bearings(msg)
        far = msg.range_max
        reach = self.max_range_cells * self.known.pitch

        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy = x + c * self.lidar_dx, y + s * self.lidar_dx

        hits, misses = [], []
        for r, a in zip(rs, angs):
            stopped = r < far - 1e-6
            if r > reach:
                # too far to attribute to a lattice line, but the near part of
                # the ray is still good evidence of open space - so keep it as
                # a miss, truncated, rather than discarding the whole ray
                r, stopped = reach, False
            b = yaw + a
            px, py = ox + math.cos(b) * r, oy + math.sin(b) * r
            if stopped:
                hits.append((px, py))
            misses.append((px, py, stopped))
        self.known.integrate((ox, oy), hits, misses)
        self.scans += 1

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.known.revision != self.last_rev and (now - self.last_pub) >= self.min_period:
            self.last_rev = self.known.revision
            self.last_pub = now
            self.pub_map.publish(String(data=json.dumps(self.known.as_dict())))
            self.write_map()
        if now - self.last_log > 5.0:
            self.last_log = now
            self.get_logger().info(
                '%d scans (%d skipped while turning), %.0f%% of the walls resolved'
                % (self.scans, self.skipped, 100.0 * self.known.coverage()))

    def write_map(self):
        try:
            tmp = self.map_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(self.known.as_dict(), f)
            os.replace(tmp, self.map_file)
        except Exception:                             # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = Mapper()
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
