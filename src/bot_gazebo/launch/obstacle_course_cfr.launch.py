#!/usr/bin/env python3
"""Launch the CFR obstacle course world."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch the obstacle course world with all robot stack."""
    pkg_gazebo = get_package_share_directory('bot_gazebo')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo_sim.launch.py'])
            ),
            launch_arguments={
                'world': 'obstacle_course_cfr',
            }.items(),
        ),
    ])
