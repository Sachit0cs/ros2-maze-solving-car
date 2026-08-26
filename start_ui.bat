@echo off
title maze_solver control panel
echo.
echo   Starting the maze_solver control panel...
echo.
echo   It will open automatically at  http://localhost:8090
echo   If it does not, open that address in Chrome yourself.
echo.
echo   Keep THIS WINDOW OPEN while you use the panel.
echo   Press Ctrl-C here to shut the server down.
echo.
start "" /b cmd /c "timeout /t 7 /nobreak >nul && start http://localhost:8090"
wsl -d Ubuntu -- bash -lc "source /opt/ros/jazzy/setup.bash && source ~/maze_solver_ws/install/setup.bash && cd ~/maze_solver_ws && python3 -m maze_solver.ui.server"
echo.
echo   Server stopped.
pause
