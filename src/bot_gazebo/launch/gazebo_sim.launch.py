#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """Launch Gazebo Sim (ros-gz) and spawn the robot."""

    pkg_gazebo = get_package_share_directory('bot_gazebo')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    # File paths
    urdf_file = os.path.join(pkg_gazebo, 'urdf', 'bot.urdf.xacro')

    # LaunchConfiguration references
    world_arg = LaunchConfiguration('world')
    headless_arg = LaunchConfiguration('headless')

    # Generate URDF from Xacro
    robot_description_content = Command(['xacro ', urdf_file])

    # Set environment for Gazebo to find models
    gz_resource_path = os.path.join(pkg_gazebo, 'models')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            gz_resource_path,
        ),
        DeclareLaunchArgument(
            'world',
            default_value='obstacle_course_cfr',
            description='World to load: obstacle_course_cfr, speed_course_cfr, obstacle_course, speed_course, or onshape_course',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo headless',
        ),

        # Publish robot description (URDF) - must be before spawn
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='log',
            parameters=[
                {
                    'robot_description': robot_description_content,
                    'use_sim_time': True,
                }
            ],
        ),

        # Gazebo Sim (ros-gz) - includes server and optional GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': Command([
                    'echo ',
                    os.path.join(pkg_gazebo, 'worlds'),
                    '/',
                    world_arg,
                    '.world -v 4'
                ]),
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # Spawn the robot from the published robot_description, at the
        # course start pose (previously occupied by the placeholder "slash" model)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'bot',
                '-x', '-0.7',
                '-y', '0.0',
                '-z', '0.0',
            ],
            output='screen',
        ),

        # Publish static transform: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='log',
        ),

        # Create a bridge for joint states and transforms
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            ],
            output='screen',
        ),
    ])
