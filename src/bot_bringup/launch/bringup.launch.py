"""Bringup launch file.

Composes the nodes owned by bot_motor, bot_ultrasonic, bot_safety, and
bot_perception. This package intentionally contains no node
implementations of its own -- see CLAUDE.md for why those live in
separate per-concern packages. Sensor drivers with existing ROS 2 packages
(RPLIDAR, OAK-D via depthai_ros_driver) and Nav2 itself are intentionally
NOT included here yet -- add IncludeLaunchDescription entries for
rplidar_ros, depthai_ros_driver, and nav2_bringup once those packages are
confirmed installed and their launch args are known (they each have their
own launch files worth including rather than reimplementing).

Usage:
    # Run with actual hardware
    ros2 launch bot_bringup bringup.launch.py
    
    # Run with Gazebo simulation
    ros2 launch bot_bringup bringup.launch.py use_sim:=true
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bringup = get_package_share_directory('bot_bringup')
    pkg_rplidar = get_package_share_directory('rplidar_ros')
    pkg_oak = get_package_share_directory('depthai_ros_driver')

    return LaunchDescription([
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

        # RPLidar driver - publishes to /scan (LaserScan)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_rplidar, 'launch', 'rplidar_a1_launch.py'])
            ),
            condition=IfCondition(LaunchConfiguration('use_sim').to_string() + ' == false'),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'lidar_link',
            }.items(),
        ),

        # OAK-D S2 driver - publishes RGB, depth, spatial detections
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_oak, 'launch', 'oak_d_lite_launch.py'])
            ),
            condition=IfCondition(LaunchConfiguration('use_sim').to_string() + ' == false'),
        ),

        # TODO: add once available --
        # IncludeLaunchDescription(robot_localization launch/ekf.launch.py)
        # IncludeLaunchDescription(nav2_bringup navigation_launch.py,
        #     launch_arguments={'params_file': path to config/nav2_params.yaml})
    ])
