#!/usr/bin/env bash
# Source this before running anything in this project:
#
#   source scripts/env.sh
#
# ROS_DOMAIN_ID IS NOT OPTIONAL HERE, AND THIS COST AN AFTERNOON.
#
# maze_solver was modelled on traffic_dodger, so it inherited that project's
# topic names - /episode/active, /cmd_vel, /scan. Both projects live on this
# machine, and ROS 2 puts every node on domain 0 unless told otherwise. So a
# traffic_dodger road_manager left running in another terminal publishes
# /episode/active False every time IT respawns ITS car, in ITS world, and this
# project's planner dutifully treats that as the start of a new episode and
# replans.
#
# What that looked like from the inside: a planner replanning three or four
# times in KNOWN mode, where the map cannot change and one search is the whole
# job. Nothing in this repository was wrong. `ros2 topic info -v
# /episode/active` reporting `Node name: road_manager` was the entire
# diagnosis, and it is the first thing to check if the numbers ever look like
# that again.
#
# Domain 42 is this project's. road_follower and traffic_dodger are on the
# default 0, the same way this panel is on port 8090 and theirs are on 8088
# and 8089.
#
# AND GZ_PARTITION, WHICH IS THE HALF THAT ROS_DOMAIN_ID DOES NOT COVER.
#
# ROS_DOMAIN_ID isolates the ROS 2 graph. It does nothing whatsoever to
# Gazebo's own transport, which is a separate bus with its own namespace - and
# gz publishes /clock on it, ungated by world name. So a second gz server
# anywhere on the machine puts a SECOND /clock on the same bus, and
# ros_gz_bridge dutifully bridges both into this project's ROS graph.
#
# What that looks like from the inside is genuinely strange, and it wasted
# hours:
#
#   SIM CLOCK IS NOT ADVANCING (5.0 s of wall time, -35.512 s of sim time)
#
# Sim time going BACKWARDS by 35 s, the car's pose alternating between (3.2,
# -0.5) and (-17.7, -6.9) - both outside a 4 m maze - and ten episodes scored
# 'stuck' in a few seconds. The other pose was traffic_dodger's car, on
# traffic_dodger's road, published by a gz server its control panel had left
# running for half an hour. Every test in this project was quietly measuring
# two simulations at once.
#
# Set both, always. Neither one is sufficient.
source /opt/ros/jazzy/setup.bash
source ~/maze_solver_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export GZ_PARTITION=maze_solver
export LIBGL_ALWAYS_SOFTWARE=1
