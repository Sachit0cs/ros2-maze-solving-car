@echo off
echo Syncing Windows -^> WSL and rebuilding...
wsl -d Ubuntu -- bash /mnt/c/Users/sachi/Desktop/ROS/maze_solver/_sync_to_wsl.sh
echo.
pause
