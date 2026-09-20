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
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 with the Nav2 default view',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value='nav2_mapping_params.yaml',
            description=(
                'Filename (relative to bot_bringup/config, not a path) of '
                'the Nav2 params file to load -- swap to A/B test planner/'
                'controller alternatives without editing this launch file, '
                'e.g. nav2_mapping_params_thetastar.yaml, '
                'nav2_mapping_params_smac_lattice.yaml, '
                'nav2_mapping_params_mppi.yaml. See CLAUDE.md\'s Nav2 '
                'planner/controller comparison section.'
            ),
        ),

        # Conditionally include Gazebo -- slam_toolbox owns map->odom here, so tell
        # gazebo_sim.launch.py not to also publish it
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo_sim.launch.py'])
            ),
            condition=IfCondition(LaunchConfiguration('use_sim')),
            launch_arguments={'publish_static_map_odom': 'false'}.items(),
        ),

        # Odometry fusion -- slam_toolbox's scan matching is more accurate
        # with a fused odom input than raw wheel/ackermann odometry alone.
        # robot_localization's own ekf.launch.py ignores 'namespace'/
        # 'params_file' launch arguments entirely -- it hardcodes its
        # package's example params (odom0: example/odom, no use_sim_time),
        # so the node is launched directly with our own config instead.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_bringup, 'config', 'ekf_params.yaml']),
                {'use_sim_time': LaunchConfiguration('use_sim')},
            ],
        ),

        # SLAM: builds the map live from /scan as the robot moves.
        # slam_params_file overrides slam_toolbox's own stock default, whose
        # base_frame ("base_footprint") this robot doesn't have -- without
        # this override slam_toolbox spins forever on "Failed to compute
        # odom pose" and never publishes /map.
        #
        # use_lifecycle_manager: true here disables online_async_launch.py's
        # own built-in auto-configure/activate (a one-shot EmitEvent fired
        # at launch-graph-build time, with no wait/retry for the node's
        # lifecycle service to exist yet). That race loses outright whenever
        # slam_toolbox is slow to come up -- which stack_size_to_use above
        # makes it, since reallocating a 40MB stack for large-map
        # serialization delays when its lifecycle interface is ready. Lost
        # race means slam_toolbox sits in "unconfigured" forever: never
        # subscribes to /scan, never publishes /map, and every downstream
        # consumer (global costmap's static_layer, frontier_explore_node)
        # waits on a map that will never come -- "mapping doesn't map".
        # The lifecycle_manager below owns activation instead, the same
        # proven-reliable retrying pattern nav2_bringup itself uses for
        # amcl/map_server/controller_server in bringup.launch.py.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim'),
                'slam_params_file': PathJoinSubstitution([pkg_bringup, 'config', 'slam_toolbox_params.yaml']),
                'use_lifecycle_manager': 'true',
            }.items(),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim'),
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }],
        ),

        # Nav2 navigation servers (controller/planner/bt_navigator) so the
        # frontier explorer below has something to send goals to. Localizes
        # against slam_toolbox's live /map, not amcl -- see bringup.launch.py
        # for the pre-built-map (amcl + map_server) race-day equivalent.
        #
        # nav2_mapping_params.yaml, not nav2_params.yaml -- see that file's
        # global_costmap comment for why mapping needs its own copy rather
        # than sharing the race-day params file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2, 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={
                'namespace': '',
                'use_sim_time': LaunchConfiguration('use_sim'),
                'params_file': PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('nav2_params_file')]),
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

        # RViz2 with Nav2's default view -- handy to watch the map fill in
        # live as frontier_explore_node drives
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_nav2, 'rviz', 'nav2_default_view.rviz'])],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim')}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
