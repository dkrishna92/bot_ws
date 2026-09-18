"""Mapping launch file -- run this once (per course/world) to build the map
that bringup.launch.py localizes against on race day. Not part of the race
launch itself; bringup.launch.py does not build a map, only consumes one.

Fully autonomous: bot_explore's frontier_explore_node drives the robot
around unexplored space via Nav2 until slam_toolbox's live map is fully
covered, then saves it -- no teleop needed.

Usage:
    # Run from the workspace root (bot_ws/) so the default map save path
    # below resolves correctly.
    ros2 launch bot_bringup mapping.launch.py use_sim:=true

    # Or point the save path elsewhere explicitly:
    ros2 launch bot_bringup mapping.launch.py use_sim:=true \
        map_save_path:=/absolute/path/to/bot_ws/src/bot_bringup/config/maps/map

    # Rebuild so the saved map is installed to the share directory, then
    # bringup.launch.py can localize against it on race day:
    colcon build --packages-select bot_bringup
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bringup = get_package_share_directory('bot_bringup')
    pkg_robot_loc = get_package_share_directory('robot_localization')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Map the Gazebo course instead of the real course',
        ),
        DeclareLaunchArgument(
            'map_save_path',
            default_value='src/bot_bringup/config/maps/map',
            description='Path prefix (no extension) slam_toolbox writes the finished map to',
        ),

        # Conditionally include Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo_sim.launch.py'])
            ),
            condition=IfCondition(LaunchConfiguration('use_sim')),
        ),

        # Odometry fusion -- slam_toolbox's scan matching is more accurate
        # with a fused odom input than raw wheel/ackermann odometry alone.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_robot_loc, 'launch', 'ekf.launch.py'])
            ),
            launch_arguments={
                'namespace': '',
                'params_file': PathJoinSubstitution([pkg_bringup, 'config', 'ekf_params.yaml']),
            }.items(),
        ),

        # SLAM: builds the map live from /scan as the robot moves
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim'),
            }.items(),
        ),

        # Nav2 navigation servers (controller/planner/bt_navigator) so the
        # frontier explorer below has something to send goals to. Localizes
        # against slam_toolbox's live /map, not amcl -- see bringup.launch.py
        # for the pre-built-map (amcl + map_server) race-day equivalent.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2, 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={
                'namespace': '',
                'use_sim_time': LaunchConfiguration('use_sim'),
                'params_file': PathJoinSubstitution([pkg_bringup, 'config', 'nav2_params.yaml']),
                'autostart': 'true',
            }.items(),
        ),

        # Drives exploration autonomously, then saves the map when done --
        # see bot_explore/frontier_explore_node.py
        Node(
            package='bot_explore',
            executable='frontier_explore_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim'),
                'save_map_name': LaunchConfiguration('map_save_path'),
            }],
        ),
    ])
