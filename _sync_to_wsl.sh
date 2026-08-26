#!/usr/bin/env bash
WIN=/mnt/c/Users/sachi/Desktop/ROS/maze_solver/maze_solver
DST=/home/sachit/maze_solver_ws/src/maze_solver
[ -d "$WIN" ] || { echo "missing $WIN"; exit 1; }
rm -rf "$DST"; mkdir -p /home/sachit/maze_solver_ws/src; cp -r "$WIN" "$DST"
find "$DST" -type f \( -name '*.py' -o -name '*.xml' -o -name '*.sdf' -o -name '*.xacro' \
     -o -name '*.md' -o -name '*.cfg' -o -name '*.rviz' -o -name '*.sh' \) -exec sed -i 's/\r$//' {} +
find "$DST" -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null
source /opt/ros/jazzy/setup.bash
cd /home/sachit/maze_solver_ws && colcon build --packages-select maze_solver --symlink-install
