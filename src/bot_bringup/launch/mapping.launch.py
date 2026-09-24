"""Mapping launch file -- run this once (per course/world) to build the map
that bringup.launch.py localizes against on race day. Not part of the race
launch itself; bringup.launch.py does not build a map, only consumes one.

Two driving modes:
  - Autonomous (default): bot_explore's frontier_explore_node drives the
    robot around unexplored space via Nav2 until slam_toolbox's live map is
    fully covered, then saves it -- no teleop needed.
  - Teleop (teleop:=true): Nav2 and frontier_explore_node are skipped
    entirely; drive manually with teleop_twist_keyboard in a *separate*
    terminal (it needs a real TTY for keyboard capture, which a `ros2
    launch` subprocess can't give it), then save the map yourself once
    you've covered the course -- see Usage below.

Usage:
    # Run from the workspace root (bot_ws/) so the default map save path
    # below resolves correctly.
    ros2 launch bot_bringup mapping.launch.py use_sim:=true

    # Teleop mode instead of autonomous frontier exploration:
    ros2 launch bot_bringup mapping.launch.py use_sim:=true teleop:=true
    # ...then in a separate terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
    # ...and once you've driven the whole course, save the map manually:
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
        "{name: {data: 'src/bot_bringup/config/maps/map'}}"

    # Or point the save path elsewhere explicitly:
    ros2 launch bot_bringup mapping.launch.py use_sim:=true \
        map_save_path:=/absolute/path/to/bot_ws/src/bot_bringup/config/maps/map

    # Resume mapping from a previously-saved (incomplete) map instead of an
    # empty one -- useful for iterating on a stall/bug that only shows up
    # partway through a run, without re-exploring/re-driving the whole
    # course from scratch every time. Save a checkpoint mid-run with:
    #   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    #       "{name: {data: 'src/bot_bringup/config/maps/checkpoint'}}"
    # then resume from it (resume_pose is the robot's actual x,y,yaw in the
    # checkpoint map's frame when it was saved -- get this from `ros2 run
    # tf2_ros tf2_echo map base_link` at save time). In use_sim mode this
    # also spawns the simulated robot at resume_pose (see gazebo_sim
    # include below) so the sim robot's actual location and slam_toolbox's
    # belief about where it's resuming from stay in sync -- on real
    # hardware you're responsible for physically placing the robot there
    # yourself instead:
    ros2 launch bot_bringup mapping.launch.py use_sim:=true teleop:=true \
        resume_map:=src/bot_bringup/config/maps/checkpoint \
        resume_pose:=1.2,3.4,0.5

    # Rebuild so the saved map is installed to the share directory, then
    # bringup.launch.py can localize against it on race day:
    colcon build --packages-select bot_bringup
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotEqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bringup = get_package_share_directory('bot_bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    def _slam_toolbox_node(context, *args, **kwargs):
        x_str, y_str, yaw_str = LaunchConfiguration('resume_pose').perform(context).split(',')
        return [
            LifecycleNode(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                namespace='',
                output='screen',
                parameters=[
                    PathJoinSubstitution([pkg_bringup, 'config', 'slam_toolbox_params.yaml']).perform(context),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim').perform(context) == 'true',
                        'map_file_name': LaunchConfiguration('resume_map').perform(context),
                        'map_start_pose': [float(x_str), float(y_str), float(yaw_str)],
                    },
                ],
            )
        ]

    def _gazebo_include(context, *args, **kwargs):
        # resume_pose must match wherever the robot is actually spawned, not
        # just what slam_toolbox is told to believe -- otherwise the sim
        # robot spawns at the course's normal start while slam_toolbox
        # assumes it's out at the checkpoint, an immediate map/reality
        # mismatch (the same class of "loses position" symptom a stale
        # amcl initial_pose caused elsewhere in this project). Forwarding
        # resume_pose's x/y/yaw here keeps both in sync from one value
        # instead of two that could drift apart. 'nan' (unchanged) when
        # resume_map is empty -- gazebo_sim.launch.py's own per-world
        # default spawn pose applies as normal.
        resume_map = LaunchConfiguration('resume_map').perform(context)
        if resume_map:
            x_str, y_str, yaw_str = LaunchConfiguration('resume_pose').perform(context).split(',')
        else:
            x_str = y_str = yaw_str = 'nan'
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo_sim.launch.py'])
                ),
                condition=IfCondition(LaunchConfiguration('use_sim')),
                launch_arguments={
                    'publish_static_map_odom': 'false',
                    'headless': LaunchConfiguration('headless'),
                    'world': LaunchConfiguration('world'),
                    'spawn_x': x_str,
                    'spawn_y': y_str,
                    'spawn_yaw': yaw_str,
                }.items(),
            )
        ]

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
            'teleop',
            default_value='false',
            description=(
                'Skip Nav2 and frontier_explore_node; drive manually via '
                'teleop_twist_keyboard in a separate terminal instead (see '
                'this file\'s module docstring for the exact commands).'
            ),
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo headless (forwarded to gazebo_sim.launch.py)',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='speed_course_cfr',
            description=(
                'World to load (forwarded to gazebo_sim.launch.py): '
                'obstacle_course_cfr, speed_course_cfr, obstacle_course, '
                'speed_course, or onshape_course'
            ),
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
        DeclareLaunchArgument(
            'resume_map',
            default_value='',
            description=(
                'Path prefix (no extension) of a previously-saved '
                'slam_toolbox map to continue mapping from, instead of '
                'starting empty -- see this file\'s module docstring. '
                'Empty (default) = start a fresh map.'
            ),
        ),
        DeclareLaunchArgument(
            'resume_pose',
            default_value='0.0,0.0,0.0',
            description=(
                'Comma-separated "x,y,yaw" pose to resume at within '
                'resume_map -- ignored when resume_map is empty.'
            ),
        ),

        # Conditionally include Gazebo -- slam_toolbox owns map->odom here, so tell
        # gazebo_sim.launch.py not to also publish it. Spawn pose is resolved
        # in _gazebo_include above (matches resume_pose when resume_map is set).
        OpaqueFunction(function=_gazebo_include),

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
        # Constructed directly (not via slam_toolbox's own
        # online_async_launch.py) so map_file_name/map_start_pose can be
        # overridden per-launch for resume_map/resume_pose above --
        # online_async_launch.py only exposes slam_params_file/
        # use_sim_time/use_lifecycle_manager as launch arguments, with no
        # way to override individual params inside that file without
        # editing it. Package/executable/name/parameters below are an exact
        # copy of that launch file's own node (confirmed against
        # slam_toolbox's upstream source), minus its auto-configure/
        # activate EmitEvent/RegisterEventHandler pair -- those only fire
        # when use_lifecycle_manager is false, and we always run with our
        # own external lifecycle_manager (below) instead, for the same
        # startup-race reason explained there.
        #
        # slam_params_file overrides slam_toolbox's own stock default, whose
        # base_frame ("base_footprint") this robot doesn't have -- without
        # this override slam_toolbox spins forever on "Failed to compute
        # odom pose" and never publishes /map.
        OpaqueFunction(function=_slam_toolbox_node),
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
        # Skipped entirely in teleop mode -- nothing needs it without
        # frontier_explore_node sending goals.
        #
        # nav2_mapping_params.yaml, not nav2_params.yaml -- see that file's
        # global_costmap comment for why mapping needs its own copy rather
        # than sharing the race-day params file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2, 'launch', 'navigation_launch.py'])
            ),
            condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('teleop'), 'true')),
            launch_arguments={
                'namespace': '',
                'use_sim_time': LaunchConfiguration('use_sim'),
                'params_file': PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('nav2_params_file')]),
                'autostart': 'true',
            }.items(),
        ),

        # Drives exploration autonomously, then saves the map when done --
        # see bot_explore/frontier_explore_node.py. Skipped in teleop mode;
        # drive manually instead and save the map yourself (see module
        # docstring) once you've covered the course.
        Node(
            package='bot_explore',
            executable='frontier_explore_node',
            output='screen',
            condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('teleop'), 'true')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim'),
                'save_map_name': LaunchConfiguration('map_save_path'),
            }],
        ),

        # RViz2 with this package's own view -- handy to watch the map fill
        # in live as frontier_explore_node drives, with a ThirdPersonFollower
        # camera that actually tracks base_link and the OAK-D depth point
        # cloud, instead of nav2_bringup's stock nav2_default_view.rviz.
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_bringup, 'launch', 'thirdpersonviewer.rviz'])],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim')}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
