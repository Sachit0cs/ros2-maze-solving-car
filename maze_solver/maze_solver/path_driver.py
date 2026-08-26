#!/usr/bin/env python3
"""Drives the path the planner published. Pure pursuit, plus the lidar.

  ros2 run maze_solver path_driver --ros-args -p meta_path:=<maze>.json

Subscribes /plan, /car/world_pose, /scan, /episode/active.
Publishes  /cmd_vel, and /car/terrain - what the ground under it costs.

TWO STEERING TERMS, DOING TWO DIFFERENT JOBS

    w = k_path   * alpha          point at the next waypoint
      + k_centre * (left - right) stay off the walls

The same split traffic_dodger used for lane keeping, and for the same reason:
neither term can do the other's job. Pure pursuit alone drives at the waypoint,
and a waypoint is a cell CENTRE, so on the inside of a corner the shortest line
to it clips the wall - the car is aiming correctly and still scraping. The
lidar centring term is what pushes it back out, and it needs no map at all: two
wedges of the scan, left and right, and the difference of their means.

It is gated on both walls being within half a cell. At a junction the side
walls open out, the difference becomes large and meaningless, and an ungated
term would yank the car toward whichever corridor happened to be longer at
exactly the moment it is trying to turn.

WHY THE MEAN AND NOT THE MINIMUM

The minimum is the right question for 'am I about to hit something' and is used
for exactly that, ahead. It is the wrong question for 'how far is the wall on
my left': one ray clipping a corner post drags the minimum down by half a metre
for a single frame, and the steering kicks. Averaging a wedge does not care.

TURN IN PLACE, THEN DRIVE

Above align_gate of heading error the car stops and rotates. A differential
drive can do this and a maze needs it: a 90 degree turn into a 0.62 m corridor
has no arc that both fits and makes progress. Below the gate the speed is
scaled down by how far off it is pointing, so the transition is not a cliff.

TERRAIN IS APPLIED HERE, AND IT IS THE WHOLE COST MODEL

v = v_max / terrain(cell). A mud cell is cost 3.0 in the search and takes three
times as long to cross on the clock. That equality is the point of the project:
when UCS routes around mud and BFS goes through it, the stopwatch has to agree
with the planner, or 'optimal' is just a word in a table.
"""
import json
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from maze_solver.lidar_utils import bearings, clean, sector_mean, sector_min, wrap
from maze_solver.maze import Maze
from maze_solver.qos import EPISODE_QOS, SENSOR_QOS


class PathDriver(Node):
    def __init__(self):
        super().__init__('path_driver')

        self.declare_parameter('meta_path', '')
        self.declare_parameter('v_max', 0.40)
        self.declare_parameter('w_max', 2.2)
        self.declare_parameter('k_path', 2.4)
        self.declare_parameter('k_centre', 1.8)
        self.declare_parameter('align_gate', 0.70)     # rad
        self.declare_parameter('turn_penalty', 0.75)
        self.declare_parameter('lookahead_cells', 0.55)
        # 0.14 m, and this number is tightly constrained from both sides.
        #
        # Too small and the car nudges a wall before it stops - the episode
        # manager calls contact at 0.085 m from the lidar. Too LARGE and it
        # cannot corner at all: a 0.62 m corridor means the wall it is about to
        # turn along sits about 0.2 m ahead while it lines the turn up, so a
        # brake at 0.20 m fires exactly when the car needs to creep forward.
        # Measured at 0.20: maze_tiny and maze_classic both cycled 'blocked:
        # 0.17 m ahead' indefinitely and timed out, having solved in 21.4 s and
        # 121.6 s at 0.17.
        #
        # It can sit this close to the wall only because of slow_distance
        # below: the car is already down to roughly 0.05 m/s by the time it
        # gets there, so the brake is a formality rather than an emergency.
        self.declare_parameter('brake_distance', 0.14)
        self.declare_parameter('slow_distance', 0.45)
        self.declare_parameter('v_floor', 0.06)
        # recovery from being blocked - see the comment in tick()
        self.declare_parameter('hold_before_backup', 20)   # ticks at 20 Hz = 1 s
        self.declare_parameter('backup_ticks', 25)
        self.declare_parameter('v_reverse', 0.12)
        self.declare_parameter('reverse_clearance', 0.22)
        # Half the diagonal of a 160 x 110 mm chassis: the radius its corners
        # sweep when it turns on the spot. Plus a margin, this is how much room
        # it needs beside it before spinning is safe.
        self.declare_parameter('corner_radius', 0.097)
        self.declare_parameter('side_margin', 0.055)
        self.declare_parameter('max_evade_ticks', 30)   # 1.5 s at 20 Hz

        g = self.get_parameter
        meta_path = g('meta_path').value
        self.v_max = float(g('v_max').value)
        self.w_max = float(g('w_max').value)
        self.k_path = float(g('k_path').value)
        self.k_centre = float(g('k_centre').value)
        self.gate = float(g('align_gate').value)
        self.turn_pen = float(g('turn_penalty').value)
        self.brake_d = float(g('brake_distance').value)
        self.slow_d = float(g('slow_distance').value)
        self.v_floor = float(g('v_floor').value)
        self.hold_before_backup = int(g('hold_before_backup').value)
        self.backup_ticks = int(g('backup_ticks').value)
        self.v_reverse = float(g('v_reverse').value)
        self.reverse_clearance = float(g('reverse_clearance').value)
        self.corner_radius = float(g('corner_radius').value)
        self.side_margin = float(g('side_margin').value)
        self.max_evade_ticks = int(g('max_evade_ticks').value)

        if not meta_path or not os.path.exists(meta_path):
            raise SystemExit('path_driver needs meta_path=<maze>.json')
        with open(meta_path) as f:
            self.meta = json.load(f)
        self.maze = Maze.from_meta(self.meta)
        self.lookahead = float(g('lookahead_cells').value) * self.maze.pitch
        # Half a cell. Longer and the car cuts corners into walls; shorter and
        # it oscillates about the centreline of a straight run.
        self.side_gate = self.maze.pitch * 0.75

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_terrain = self.create_publisher(Float32, 'car/terrain', 10)
        # 'I tried to go that way and something solid stopped me.' The mapper
        # believes this over any lidar ray - see Knowledge.assert_wall.
        self.pub_blocked = self.create_publisher(String, 'car/blocked', 10)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(PoseStamped, 'car/world_pose', self.on_pose, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, SENSOR_QOS)
        self.create_subscription(Bool, 'episode/active', self.on_active, EPISODE_QOS)
        self.create_timer(0.05, self.tick)

        self.path = []
        self.pose = None
        self.scan = None
        self.active = False
        self.last_cell = None
        self.hold_ticks = 0
        self.evade_ticks = 0
        self.get_logger().info('path_driver up: v_max %.2f m/s, lookahead %.2f m'
                               % (self.v_max, self.lookahead))

    # ------------------------------------------------------------- callbacks

    def on_plan(self, m):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def on_pose(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        self.pose = (m.pose.position.x, m.pose.position.y, yaw)

    def on_scan(self, m):
        self.scan = m

    def on_active(self, m):
        if not m.data:
            self.pub_cmd.publish(Twist())
            self.path = []
        self.active = m.data

    # ------------------------------------------------------------ the target

    def target(self, x, y):
        """The lookahead point: walk forward from the nearest path point."""
        if not self.path:
            return None
        best_i, best_d = 0, float('inf')
        for i, (px, py) in enumerate(self.path):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        acc = 0.0
        for i in range(best_i, len(self.path) - 1):
            acc += math.dist(self.path[i], self.path[i + 1])
            if acc >= self.lookahead:
                return self.path[i + 1]
        return self.path[-1]

    def report_blocked(self, cell):
        """Tell the mapper which edge just stopped the car.

        The next cell on the path, not the lookahead point: the lookahead can
        be two or three cells ahead, and the wall in front of the bumper is on
        the very next edge.
        """
        for px, py in self.path:
            nxt = self.maze.world_to_cell(px, py)
            if nxt == cell:
                continue
            if abs(nxt[0] - cell[0]) + abs(nxt[1] - cell[1]) == 1:
                self.pub_blocked.publish(
                    String(data=json.dumps([list(cell), list(nxt)])))
            return

    # ------------------------------------------------------------------ tick

    def tick(self):
        if not self.active or self.pose is None or not self.path:
            return
        x, y, yaw = self.pose
        tgt = self.target(x, y)
        if tgt is None:
            return

        alpha = wrap(math.atan2(tgt[1] - y, tgt[0] - x) - yaw)

        # terrain under the wheels - physics, not knowledge, so it is the true
        # maze that is consulted here even in discovery mode
        cell = self.maze.world_to_cell(x, y)
        t = self.maze.terrain[cell[1]][cell[0]]
        if cell != self.last_cell:
            self.last_cell = cell
            self.pub_terrain.publish(Float32(data=float(t)))

        left = right = front = None
        rs = angs = []
        if self.scan is not None:
            rs = clean(self.scan)
            angs = bearings(self.scan)
            left = sector_mean(rs, angs, math.pi / 2.0, 0.42)
            right = sector_mean(rs, angs, -math.pi / 2.0, 0.42)
            front = sector_min(rs, angs, 0.0, 0.35)

        # How close is the nearest thing on each side? This is a SAFETY
        # question, so it is the minimum and not the mean - unlike the centring
        # term above, which wants the mean precisely because one ray clipping a
        # corner post should not kick the steering.
        # +/- 0.45 rad, so the wedge spans 64 to 116 degrees: genuinely
        # LATERAL. The first version used +/- 1.0 rad, which reaches to 33
        # degrees off the nose and therefore sees the wall the car is driving
        # TOWARD. Approaching any corner then read as 'no room beside me', the
        # guard below reversed instead of turning, reversing kept the same wall
        # in the wedge, and the car sat at (4, 3) of maze_classic for the whole
        # stuck timeout - silently, because reversing logs nothing and the
        # blocked branch was never reached.
        near_l = near_r = None
        if rs:
            near_l = sector_min(rs, angs, math.pi / 2.0, 0.45)
            near_r = sector_min(rs, angs, -math.pi / 2.0, 0.45)

        cmd = Twist()
        if abs(alpha) > self.gate:
            # Too far off to drive: rotate on the spot.
            #
            # But check there is room to rotate first. This car is 160 x 110 mm,
            # so its corner sweeps a 97 mm radius - in a 620 mm corridor that
            # leaves 213 mm if it is perfectly centred, and much less if it is
            # not. Spinning while pressed against one wall walks a corner into
            # it. Measured: a discovery run scored 'wall' at 48.3 s having
            # driven perfectly well for fifteen replans up to that point.
            #
            # So if one side is tight, ease away from it before turning.
            room = self.corner_radius + self.side_margin
            tight = (near_l is not None and min(near_l, near_r) < room
                     and self.evade_ticks < self.max_evade_ticks)
            if tight:
                # ease away from the closer side before turning
                self.evade_ticks += 1
                cmd.linear.x = -self.v_reverse * 0.5
                cmd.angular.z = -1.0 if near_l < near_r else 1.0
                self.pub_cmd.publish(cmd)
                return
            # Capped, and the cap matters. Backing away can leave the same wall
            # in the wedge, so an uncapped evasion is a car that reverses for
            # ever without logging anything. After max_evade_ticks it turns
            # regardless and lets the episode manager judge the result.
            self.evade_ticks = 0
            cmd.angular.z = max(-self.w_max, min(self.w_max, 3.0 * alpha))
            cmd.linear.x = 0.0
        else:
            w = self.k_path * alpha
            # only trust the side walls when there ARE side walls
            if (left is not None and right is not None
                    and left < self.side_gate and right < self.side_gate):
                w += self.k_centre * (left - right)
            # a hard push away from a side that is genuinely close, on top of
            # the soft centring term - this one is not gated on being in a
            # corridor, because a wall 0.12 m away matters wherever it is
            room = self.corner_radius + self.side_margin
            if near_l is not None and near_l < room:
                w -= self.k_centre * 2.0 * (room - near_l)
            if near_r is not None and near_r < room:
                w += self.k_centre * 2.0 * (room - near_r)
            w = max(-self.w_max, min(self.w_max, w))

            v = (self.v_max / max(t, 1.0)) * (1.0 - self.turn_pen
                                              * abs(alpha) / self.gate)
            v = max(self.v_floor, v)

            # Ease off as the wall approaches, rather than running at full
            # speed into a cliff-edge brake.
            #
            # This matters most in DISCOVERY mode, where the planner assumes
            # unknown edges are open and therefore routes deliberately at walls
            # it has never seen. The car meets them at v_max. Stopping from
            # 0.40 m/s under the model's 1.5 m/s^2 takes 53 mm, so a brake that
            # fires at 0.17 m halts with 117 mm to spare - against an 85 mm
            # crash threshold. That is not much margin once a corner and some
            # lidar noise are involved, and it was not enough: a discovery run
            # was scored 'wall' after 4.6 s.
            if front is not None and front < self.slow_d:
                span = max(1e-3, self.slow_d - self.brake_d)
                v = min(v, self.v_max * max(0.0, front - self.brake_d) / span)
                v = max(0.0, v)

            if front is not None and front < self.brake_d:
                # Something is close ahead that the plan did not expect. In
                # discovery mode that is routinely a wall the mapper is about
                # to learn about, so stopping is not a failure - it is the
                # moment before a replan.
                #
                # But stopping ALONE deadlocks, and it did. If the car has
                # nosed into a wall while pointing straight at its next
                # waypoint, then alpha is near zero, the centring term is near
                # zero, so w is near zero and v is zero: it sits there
                # publishing (0, 0) until the episode times out. Measured on
                # maze_classic as 'holding: 0.13 m ahead, 26 cells of plan
                # left', logged over and over for the full 20 s stuck timeout.
                #
                # So being blocked is a STATE with an escape, not a veto on
                # motion. Hold and steer for a second; if that has not cleared
                # it, reverse a little while turning toward whichever side has
                # more room, then try again.
                self.hold_ticks += 1
                v = 0.0
                if self.hold_ticks == 1:
                    self.get_logger().info(
                        'blocked: %.2f m ahead, %d cells of plan left'
                        % (front, len(self.path)))
                    self.report_blocked(cell)
                elif self.hold_ticks > self.hold_before_backup:
                    behind = (sector_min(rs, angs, math.pi, 0.35)
                              if self.scan is not None else 0.0)
                    if behind > self.reverse_clearance:
                        v = -self.v_reverse
                        # turn toward the open side while backing out; with no
                        # side information, commit to one direction rather than
                        # dithering
                        w = 1.4 if (left or 0) >= (right or 0) else -1.4
                        if self.hold_ticks == self.hold_before_backup + 1:
                            self.get_logger().info(
                                'backing out (%.2f m behind, turning %s)'
                                % (behind, 'left' if w > 0 else 'right'))
                    else:
                        # boxed in front and back: spin, it is all that is left
                        w = self.w_max * 0.8
                    if self.hold_ticks > self.hold_before_backup + self.backup_ticks:
                        self.hold_ticks = 0        # try the plan again
            else:
                self.hold_ticks = 0

            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = PathDriver()
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
