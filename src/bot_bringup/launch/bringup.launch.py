"""Bringup launch file.

Composes the nodes owned by bot_motor, bot_ultrasonic, bot_safety, and
bot_perception. This package intentionally contains no node
implementations of its own -- see CLAUDE.md for why those live in
separate per-concern packages. Sensor drivers (RPLIDAR, OAK-D) launch
only if their packages are installed; missing drivers don't block launch.

Usage:
    # Run with actual hardware (sensors must be installed)
    ros2 launch bot_bringup bringup.launch.py

    # Run with Gazebo simulation (no real sensors needed)
    ros2 launch bot_bringup bringup.launch.py use_sim:=true
"""
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotEqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bringup = get_package_share_directory('bot_bringup')

    # Try to find sensor driver packages; skip if not installed
    try:
        pkg_rplidar = get_package_share_directory('rplidar_ros')
        rplidar_available = True
    except PackageNotFoundError:
        pkg_rplidar = None
        rplidar_available = False

    try:
        pkg_oak = get_package_share_directory('depthai_ros_driver')
        oak_available = True
    except PackageNotFoundError:
        pkg_oak = None
        oak_available = False

    actions = [
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Launch with Gazebo simulation',
        ),

        # Set use_sim_time parameter when using simulation
        SetEnvironmentVariable(
            name='ROS_DOMAIN_ID',
            value='0'
        ),

        # Conditionally include Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo_sim.launch.py'])
            ),
            condition=IfCondition(LaunchConfiguration('use_sim')),
        ),

        # Custom nodes (always launch)
        Node(
            package="bot_safety",
            executable="watchdog_node",
            name="watchdog_node",
            output="screen",
        ),
        Node(
            package="bot_ultrasonic",
            executable="ultrasonic_node",
            name="ultrasonic_node",
            output="screen",
        ),
        Node(
            package="bot_motor",
            executable="motor_node",
            name="motor_node",
            output="screen",
        ),
        Node(
            package="bot_perception",
            executable="start_trigger_node",
            name="start_trigger_node",
            output="screen",
        ),
        Node(
            package="bot_perception",
            executable="sensor_fusion_node",
            name="sensor_fusion_node",
            output="screen",
        ),
    ]

    # RPLidar driver - only if package is installed and not in sim mode
    if rplidar_available:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_rplidar, 'launch', 'rplidar_a1_launch.py'])
                ),
                condition=IfCondition(NotEqualsSubstitution(
                    LaunchConfiguration('use_sim'), 'true'
                )),
                launch_arguments={
                    'serial_port': '/dev/ttyUSB0',
                    'frame_id': 'lidar_link',
                }.items(),
            )
        )

    # OAK-D S2 driver - only if package is installed and not in sim mode
    if oak_available:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_oak, 'launch', 'oak_d_lite_launch.py'])
                ),
                condition=IfCondition(NotEqualsSubstitution(
                    LaunchConfiguration('use_sim'), 'true'
                )),
            )
        )

    # TODO: add once available --
    # IncludeLaunchDescription(robot_localization launch/ekf.launch.py)
    # IncludeLaunchDescription(nav2_bringup navigation_launch.py,
    #     launch_arguments={'params_file': path to config/nav2_params.yaml})

    return LaunchDescription(actions)
