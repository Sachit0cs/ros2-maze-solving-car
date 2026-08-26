"""Bring up Gazebo on a generated maze and bridge the car to ROS 2.

  ros2 launch maze_solver maze_sim.launch.py world:=$HOME/maze_solver_ws/mazes/maze_classic.sdf
  ros2 launch maze_solver maze_sim.launch.py world:=... gui:=false
  ros2 launch maze_solver maze_sim.launch.py world:=... rviz:=true

Mazes are generated, not shipped - make the teaching set first:

  python3 -m maze_solver.make_mazes --out ~/maze_solver_ws/mazes

The spawn pose is read from the maze's .json rather than passed in. A maze is
centred on the world origin, and the origin of a maze is the middle of it -
spawning there would drop the car in a random cell somewhere in the interior,
which is not the start, and nothing downstream would notice until the scores
came out wrong.

This is an OpaqueFunction because that .json can only be read once `world` has
an actual value, which is after launch argument substitution.
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def read_meta(world_path):
    """The maze's metadata, if it is sitting beside the .sdf where it belongs."""
    meta_path = os.path.splitext(world_path)[0] + '.json'
    try:
        with open(meta_path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('maze_solver')
    xacro_file = os.path.join(pkg, 'description', 'maze_car.urdf.xacro')

    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    meta = read_meta(world)

    start = meta.get('start_pose', [0.0, 0.0, 0.0])
    x0 = LaunchConfiguration('x').perform(context) or ('%.4f' % start[0])
    y0 = LaunchConfiguration('y').perform(context) or ('%.4f' % start[1])
    yaw0 = LaunchConfiguration('yaw').perform(context) or ('%.4f' % start[2])

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/ego/true_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    ]

    # ParameterValue(..., value_type=str) is required: without it launch tries
    # to parse the URDF XML as YAML and dies.
    robot_description = ParameterValue(Command(['xacro ', xacro_file]),
                                       value_type=str)

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': '-r -v1 ' + world}.items(),
            condition=IfCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': '-r -s -v1 ' + world}.items(),
            condition=UnlessCondition(gui)),

        # Every support node gets a maze_ prefixed NAME, and that is not
        # cosmetic - it is how scripts/kill_sim.sh finds them.
        #
        # ros_gz_bridge, robot_state_publisher and rviz2 are shared executables:
        # road_follower and traffic_dodger run the very same binaries on this
        # machine, so teardown cannot match on the process name without killing
        # their simulations too. When kill_sim was scoped to stop doing that, it
        # stopped killing these at all - and they accumulated. Measured, after a
        # morning of test runs: ONE maze_manager, ONE path_driver, ONE planner,
        # and THIRTEEN ros_gz_bridge nodes, every one of them still bridging
        # gz topics into this graph. The renaming puts `__node:=maze_bridge`
        # into the process command line, which is unique to this project and
        # safe to pkill.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='maze_rsp', output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': True}]),

        # z = 0.005, not 0.06.
        #
        # base_footprint is DEFINED to sit at ground contact - the wheel
        # bottoms and the caster bottom are both derived to land on it - so
        # spawning at 0.06 drops the car 55 mm. Measured consequence: it landed
        # on free-spinning wheels with a frictionless caster and skidded 1.0 m
        # in 4 s with nothing commanding it, ending up a full cell east of the
        # start. The manager then scored the episode 'stuck' and respawned,
        # which the planner correctly reported as a replan in known mode.
        #
        # None of that looked like a spawn height. It looked like a controller
        # bug, and then like a planner bug.
        Node(package='ros_gz_sim', executable='create', name='maze_spawn',
             output='screen',
             arguments=['-topic', 'robot_description', '-name', 'maze_car',
                        '-x', x0, '-y', y0, '-z', '0.005', '-Y', yaw0]),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='maze_bridge', output='screen', arguments=bridge_args,
             parameters=[{'use_sim_time': True}]),

        Node(package='rviz2', executable='rviz2', name='maze_rviz',
             output='screen', condition=IfCondition(rviz),
             arguments=['-d', os.path.join(pkg, 'config', 'maze.rviz')],
             parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    default_world = os.path.expanduser(
        '~/maze_solver_ws/mazes/maze_classic.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # empty means 'take it from the maze metadata'
        DeclareLaunchArgument('x', default_value=''),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
