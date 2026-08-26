#!/usr/bin/env python3
"""The two QoS profiles this project uses, in one place so they cannot drift.

QoS has to MATCH between a publisher and a subscriber or they simply never
connect - no error, no warning, just a topic with a publisher and a subscriber
that never speak. Defining each profile once and importing it is the only
reliable way to keep four nodes agreeing.

SENSOR_QOS   best effort, volatile, depth 5.
             The right profile for a 10 Hz lidar: if a scan is lost, another
             one is 100 ms away, and blocking to redeliver a stale scan is
             worse than dropping it.

EPISODE_QOS  reliable, TRANSIENT LOCAL, depth 1.
             /episode/active is STATE, not an event stream, and this
             distinction cost a debugging session.

             With the default volatile profile the episode manager published
             `active = True` at 5 Hz for twenty seconds and the planner never
             received one of them - it had come up first, its subscription
             matched late, and every message published before the match was
             simply gone. The run then failed as 'stuck' with the car sitting
             at the start line and the planner, correctly, never planning
             because as far as it knew no episode had begun. Intermittent,
             because it is a discovery race: the same command succeeded the
             next time it was run.

             Transient local keeps the last value and delivers it to a
             subscriber the moment it matches, however late. A node that joins
             mid-episode learns the truth immediately instead of waiting for a
             transition that has already happened.
"""
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

EPISODE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
