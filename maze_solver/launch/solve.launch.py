"""Solve a maze: plan with one algorithm, drive the plan, and score it.

  ros2 launch maze_solver solve.launch.py meta:=$HOME/maze_solver_ws/mazes/maze_classic.json
  ros2 launch maze_solver solve.launch.py meta:=... algorithm:=bfs
  ros2 launch maze_solver solve.launch.py meta:=... mode:=discovery
  ros2 launch maze_solver solve.launch.py meta:=... driver:=wall

Four arguments, and between them they cover every experiment in the project:

  algorithm  bfs | dfs | bidirectional | ucs | greedy | astar
  mode       known      the planner searches the real maze, once
             discovery  the planner searches what the lidar has found, and
                        replans whenever a new wall invalidates the route
  driver     plan       follow the planner's path
             wall       the left-hand rule, with no planner at all
  gui        passed through to nothing here - see maze_sim.launch.py

driver:=wall deliberately launches NO planner and NO mapper. The whole claim
about the wall follower is that it needs neither, and a launch file that
started them anyway - even harmlessly - would undercut the demonstration every
time somebody ran `ros2 node list`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    meta = LaunchConfiguration('meta')
    algorithm = LaunchConfiguration('algorithm')
    mode = LaunchConfiguration('mode')
    driver = LaunchConfiguration('driver')
    corridor = LaunchConfiguration('corridor')

    # PythonExpression rather than EqualsSubstitution/AndSubstitution: those
    # are newer additions to launch and this has to run on whatever Jazzy
    # shipped. A string comparison in a PythonExpression has worked since
    # Foxy and reads the same.
    planning = PythonExpression(["'", driver, "' == 'plan'"])
    discovering = PythonExpression(
        ["'", driver, "' == 'plan' and '", mode, "' == 'discovery'"])

    return LaunchDescription([
        DeclareLaunchArgument('meta'),
        DeclareLaunchArgument('algorithm', default_value='astar'),
        DeclareLaunchArgument('mode', default_value='known'),
        DeclareLaunchArgument('driver', default_value='plan'),
        # The wall follower is TOLD the corridor width rather than reading it
        # out of the maze file, because a node that opens the maze file is not
        # a node that has no map. Telling a real robot the width of the
        # corridors it is about to be placed in is fair; letting it read the
        # floor plan is not.
        DeclareLaunchArgument('corridor', default_value='0.62'),

        # The referee. Runs in every configuration - it is what turns 'it
        # looked like it worked' into a time and a completion rate.
        Node(package='maze_solver', executable='maze_manager', output='screen',
             parameters=[{'meta_path': ParameterValue(meta, value_type=str),
                          'use_sim_time': True}]),

        Node(package='maze_solver', executable='planner', output='screen',
             condition=IfCondition(planning),
             parameters=[{'meta_path': ParameterValue(meta, value_type=str),
                          'algorithm': ParameterValue(algorithm, value_type=str),
                          'mode': ParameterValue(mode, value_type=str),
                          'use_sim_time': True}]),

        Node(package='maze_solver', executable='path_driver', output='screen',
             condition=IfCondition(planning),
             parameters=[{'meta_path': ParameterValue(meta, value_type=str),
                          'use_sim_time': True}]),

        # Only in discovery mode. In known mode there is nothing to map: the
        # planner already has the answer, and a mapper would be burning 180
        # raycasts every 100 ms to rediscover it.
        Node(package='maze_solver', executable='mapper', output='screen',
             condition=IfCondition(discovering),
             parameters=[{'meta_path': ParameterValue(meta, value_type=str),
                          'use_sim_time': True}]),

        Node(package='maze_solver', executable='wall_follower', output='screen',
             condition=IfCondition(PythonExpression(["'", driver, "' == 'wall'"])),
             parameters=[{'corridor': ParameterValue(corridor, value_type=float),
                          'use_sim_time': True}]),
    ])
