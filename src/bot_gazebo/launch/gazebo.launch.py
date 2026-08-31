#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Launch Gazebo with the robot model in the obstacle course world."""

    pkg_gazebo = get_package_share_directory('bot_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Set to "true" to run Gazebo headless (no GUI)',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [pkg_gazebo, 'worlds', 'obstacle_course.world']
            ),
            description='Full path to the Gazebo world file',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo_sim.launch.py'])
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'headless': LaunchConfiguration('headless'),
            }.items(),
        ),
    ])
