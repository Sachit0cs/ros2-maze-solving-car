#!/usr/bin/env python3
"""Turning a LaserScan into the few numbers the maze nodes actually want.

Small on purpose. traffic_dodger's version of this file binned a scan down to
a fixed-length observation vector because a neural network needed one; nothing
here learns, so there is no vector to build. What the maze nodes need is:

  clean(msg)         ranges with inf, nan and out-of-range turned into a real
                     number, because 'no return' and 'a wall at 6 m' are the
                     same thing to a controller and only one of them is a float
  sector_min(...)    how close the nearest thing is within a wedge of bearings
  scan_points(...)   the returns as world (x, y), for the discovery mapper

WHY inf BECOMES range_max AND NOT ZERO

A ray that hits nothing comes back as inf (or, on some drivers, 0.0). Feeding
either straight into a min() is how a controller ends up braking for empty
space: 0.0 is the closest possible obstacle. Both are mapped to range_max here,
once, so no caller has to remember. The maze's outer wall is sealed, so in
practice a genuinely infinite ray only happens looking straight down a long
corridor - which is exactly when you least want to brake.
"""
import math


def clean(msg, cap=None):
    """Ranges as floats, with every non-measurement mapped to the far limit."""
    top = cap if cap is not None else msg.range_max
    lo = msg.range_min
    out = []
    for r in msg.ranges:
        if r is None or math.isnan(r) or math.isinf(r) or r <= lo or r > top:
            out.append(top)
        else:
            out.append(float(r))
    return out


def bearings(msg):
    """The bearing of every ray, in the lidar frame. 0 is straight ahead."""
    return [msg.angle_min + i * msg.angle_increment
            for i in range(len(msg.ranges))]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def sector_min(ranges, angs, centre, half_width, default=None):
    """Closest return within +/- half_width of a bearing.

    Bearings are compared after wrapping the difference, so a wedge that
    straddles the +/-pi seam - which on a 360 degree scan is 'directly behind',
    a wedge the wall follower genuinely uses - works without special-casing.
    """
    best = None
    for r, a in zip(ranges, angs):
        if abs(wrap(a - centre)) <= half_width:
            if best is None or r < best:
                best = r
    if best is None:
        return default if default is not None else (max(ranges) if ranges else 0.0)
    return best


def sector_mean(ranges, angs, centre, half_width, default=None):
    """Mean return within a wedge - steadier than the min for a centring term.

    The min is the right answer for 'will I hit something'. It is the wrong
    answer for 'how far is the wall on my left', because a single ray clipping
    the corner post of a junction drags the min down half a metre for one frame
    and kicks the steering. The mean over a wedge does not care.
    """
    vals = [r for r, a in zip(ranges, angs) if abs(wrap(a - centre)) <= half_width]
    if not vals:
        return default if default is not None else (max(ranges) if ranges else 0.0)
    return sum(vals) / len(vals)


def scan_points(ranges, angs, x, y, yaw, lidar_dx=0.0, max_range=None):
    """Scan returns as world (x, y), skipping the non-measurements.

    lidar_dx is the lidar's forward offset from the pose frame. On this robot
    it is 0.03 m, which is small - but a maze cell is 0.68 m, so a 30 mm error
    is 4 percent of a cell, and the mapper decides which SIDE of a wall lattice
    line a return fell on. Getting it wrong marks the wrong wall.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = x + c * lidar_dx, y + s * lidar_dx
    out = []
    for r, a in zip(ranges, angs):
        if max_range is not None and r >= max_range - 1e-6:
            continue                       # nothing was hit along this ray
        b = yaw + a
        out.append((ox + math.cos(b) * r, oy + math.sin(b) * r))
    return out


def free_ray(x0, y0, x1, y1, step):
    """Sample points along a ray, for marking what it passed THROUGH.

    Marking only where a ray stopped tells you where walls are. Marking what
    each ray crossed on the way tells you where they are NOT, which is the
    half that lets a planner commit to a route instead of creeping. Both halves
    are needed and they are different loops, so the sampling lives here.
    """
    d = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(d / step))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]
