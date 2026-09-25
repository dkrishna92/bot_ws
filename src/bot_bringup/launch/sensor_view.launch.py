"""Bench view of the real sensors: hardware.launch.py + RViz, no Nav2/EKF.

For a Pi with a monitor attached (setup_pi.sh doesn't install RViz --
`sudo apt install ros-jazzy-rviz2` first). The view is anchored at
base_link, so it works without odometry or a map: lidar /scan, the OAK-D
/oak/points cloud, the URDF and TF.

Motor/watchdog nodes still start (hardware.launch.py owns them), but with
no cmd_vel publisher the motors stay stopped.

Usage:
    ros2 launch bot_bringup sensor_view.launch.py
"""
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('bot_bringup')
    # Fail before any hardware node starts -- a missing rviz2 package would
    # otherwise crash the launch midway and orphan the nodes already running.
    try:
        get_package_share_directory('rviz2')
    except PackageNotFoundError:
        raise RuntimeError('rviz2 not installed -- run: sudo apt install ros-jazzy-rviz2')
    return LaunchDescription([
        # The desktop is a Wayland session; RViz's Qt/OGRE is more reliable
        # through XWayland.
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_bringup, 'launch', 'hardware.launch.py'])
            ),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_bringup, 'launch', 'sensors.rviz'])],
            output='screen',
        ),
    ])
