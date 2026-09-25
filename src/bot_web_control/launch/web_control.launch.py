"""Launches the localhost web control dashboard (see web_control_node.py's
module docstring for what it does and how to use it).

Usage:
    ros2 launch bot_web_control web_control.launch.py
    # then open http://localhost:8080

Run this ALONGSIDE whatever else is already up (gazebo_sim, nav2, etc.) --
this launch file does not start those itself; the dashboard's own
Start/Mapping Run buttons do that via `ros2 launch` subprocesses instead,
so the dashboard can be left running across multiple start/stop cycles.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'http_port', default_value='8080',
            description='Port the dashboard listens on',
        ),
        DeclareLaunchArgument(
            'world', default_value='speed_course_cfr',
            description='World forwarded to bringup/mapping launch when started from the dashboard',
        ),
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='use_sim forwarded to bringup/mapping launch when started from the dashboard',
        ),
        DeclareLaunchArgument(
            'workspace_root', default_value='',
            description=(
                'Workspace root (e.g. /path/to/bot_ws) so Stop can also run '
                'scripts/clean_sim.sh for a thorough sweep. Leave blank to '
                'skip that extra step.'
            ),
        ),
        Node(
            package='bot_web_control',
            executable='web_control_node',
            output='screen',
            parameters=[{
                'http_port': LaunchConfiguration('http_port'),
                'world': LaunchConfiguration('world'),
                'use_sim': LaunchConfiguration('use_sim'),
                'workspace_root': LaunchConfiguration('workspace_root'),
            }],
        ),
    ])
