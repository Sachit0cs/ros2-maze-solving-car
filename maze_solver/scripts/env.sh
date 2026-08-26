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
source /opt/ros/jazzy/setup.bash
source ~/maze_solver_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export LIBGL_ALWAYS_SOFTWARE=1
