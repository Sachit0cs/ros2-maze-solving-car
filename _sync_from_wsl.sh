#!/usr/bin/env bash
WIN=/mnt/c/Users/sachi/Desktop/ROS/maze_solver
rm -rf "$WIN/maze_solver"
cp -r /home/sachit/maze_solver_ws/src/maze_solver "$WIN/maze_solver"
find "$WIN/maze_solver" -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null
echo "pulled WSL -> Windows"
