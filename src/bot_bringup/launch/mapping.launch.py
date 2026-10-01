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
    # course from scratch every time. Save a checkpoint mid-run with
    # serialize_map, NOT save_map (found 2026-09-27, was save_map here
    # before): map_file_name below is a slam_toolbox startup parameter
    # that loads a *serialized pose graph* (.posegraph/.data), the same
    # format mapper_params_localization.yaml's own stock example expects
    # -- save_map only ever exports a flattened occupancy-grid image
    # (.pgm/.yaml, what bringup.launch.py's amcl/map_server load for
    # racing), which map_file_name can't load at all. Checkpointing via
    # save_map alone makes this resume silently start a fresh, empty map
    # instead of continuing -- everything the robot doesn't happen to
    # re-scan during the resumed drive reverts to unknown space, which can
    # look like it "worked" (the map's bounds grow to cover new ground)
    # while actually discarding previously-good map data elsewhere:
    #   ros2 service call /slam_toolbox/serialize_map \
    #       slam_toolbox/srv/SerializePoseGraph \
    #       "{filename: 'src/bot_bringup/config/maps/checkpoint'}"
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
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotEqualsSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, LifecycleNode


def _rviz_available():
    # Launching a Node whose package isn't installed crashes the whole launch
    # *after* earlier nodes have started, orphaning them (still holding the
    # lidar port and motor GPIO). The Pi doesn't ship RViz, so check up front.
    # Also skip it with no display: the race Pi boots headless now, and
    # rviz2 aborts (exit -6) without one -- which took the whole launch down
    # with it mid-startup (2026-09-29).
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        return False
    try:
        get_package_share_directory('rviz2')
        return True
    except PackageNotFoundError:
        return False


def _capped_nav2_params(path, max_speed, max_turn):
    """Nav2 params with speeds capped for slow first laps: returns (path to
    use, what changed). Caps the controller (DWB max_vel_x/max_speed_xy/
    max_vel_theta, MPPI vx_max/wz_max, RPP + rotation shim
    desired_linear_vel/rotate_to_heading_angular_vel), the velocity smoother and the spin
    behaviour -- the smoother alone would clamp output while the controller
    still planned for full speed. max_speed/max_turn <= 0 leave that axis
    alone; nothing capped returns the original path."""
    if max_speed <= 0 and max_turn <= 0:
        return path, []
    with open(path) as f:
        params = yaml.safe_load(f) or {}
    changed = []

    def cap(d, key, limit):
        v = d.get(key)
        if limit > 0 and isinstance(v, (int, float)) and abs(v) > limit:
            d[key] = limit if v > 0 else -limit
            changed.append(key)

    ctrl = (params.get('controller_server') or {}).get('ros__parameters') or {}
    for plugin in ctrl.values():
        if isinstance(plugin, dict):
            for key in ('max_vel_x', 'max_speed_xy', 'vx_max', 'desired_linear_vel'):
                cap(plugin, key, max_speed)
            for key in ('max_vel_theta', 'max_speed_theta', 'wz_max', 'rotate_to_heading_angular_vel'):
                cap(plugin, key, max_turn)
    smoother = (params.get('velocity_smoother') or {}).get('ros__parameters') or {}
    for key in ('max_velocity', 'min_velocity'):
        vel = smoother.get(key)
        if isinstance(vel, list) and len(vel) == 3:
            for i, limit in ((0, max_speed), (2, max_turn)):
                if limit > 0 and abs(vel[i]) > limit:
                    vel[i] = limit if vel[i] > 0 else -limit
                    changed.append(f'velocity_smoother.{key}[{i}]')
    cap((params.get('behavior_server') or {}).get('ros__parameters') or {}, 'max_rotational_vel', max_turn)
    fd, out = tempfile.mkstemp(prefix='nav2_capped_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(params, f)
    return out, changed


def _race_route(context):
    """Race mode's route file: the 'route' argument as a name in config/maps
    (<name>.route.yaml) or a path; '' if not given."""
    value = LaunchConfiguration('route').perform(context).strip()
    if not value:
        return ''
    if value.endswith('.yaml') or os.sep in value:
        return os.path.abspath(os.path.expanduser(value))
    return os.path.join(get_package_share_directory('bot_bringup'), 'config', 'maps', f'{value}.route.yaml')


def _nav2_and_race(context, *args, **kwargs):
    """Nav2 params (speed-capped if asked) for the Nav2 include that
    follows, plus -- in race mode -- the start trigger and lap_navigator.

    Race mode (Plan B, see PREP_DAY.md) races on slam_toolbox's LIVE map
    instead of a saved one: every run starts on the marked start spot, so
    the live map's origin is the start and a route recorded (relative to
    that same start) in an earlier mapping run still lines up."""
    pkg_bringup = get_package_share_directory('bot_bringup')
    base = os.path.join(pkg_bringup, 'config', LaunchConfiguration('nav2_params_file').perform(context))
    params_path, changed = _capped_nav2_params(
        base,
        float(LaunchConfiguration('max_speed').perform(context) or 0),
        float(LaunchConfiguration('max_turn').perform(context) or 0))
    actions = [SetLaunchConfiguration('nav2_params_resolved', params_path)]
    if changed:
        actions.append(LogInfo(msg=f"Nav2 speeds capped ({', '.join(changed)}) via {params_path}"))
    if LaunchConfiguration('race').perform(context) != 'true':
        return actions

    use_sim = LaunchConfiguration('use_sim').perform(context) == 'true'
    route_file = _race_route(context)
    if route_file and not os.path.isfile(route_file):
        return actions + [LogInfo(msg=f"FATAL: race route {route_file} not found"),
                          Shutdown(reason='race route missing')]
    actions.append(LogInfo(msg=(
        f"RACE MODE on the live SLAM map, route {route_file}" if route_file else
        "RACE MODE with no route:= -- lap_navigator falls back to its BUILT-IN sim checkpoints "
        "(only meaningful in sim)")))
    actions.append(Node(
        package='bot_perception', executable='start_trigger_node', name='start_trigger_node', output='screen',
        parameters=[{'image_topic': '/camera/image_raw', 'use_sim_time': True}] if use_sim else
                   [{'use_sim_time': False}],
    ))
    actions.append(Node(
        package='bot_navigation', executable='lap_navigator_node', name='lap_navigator_node', output='screen',
        parameters=[{
            'course': LaunchConfiguration('world'),
            'num_laps': LaunchConfiguration('num_laps'),
            'route_file': route_file,
            'use_sim_time': use_sim,
        }],
    ))
    return actions


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bringup = get_package_share_directory('bot_bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    def _slam_toolbox_node(context, *args, **kwargs):
        x_str, y_str, yaw_str = LaunchConfiguration('resume_pose').perform(context).split(',')
        param_files = [PathJoinSubstitution([pkg_bringup, 'config', 'slam_toolbox_params.yaml']).perform(context)]
        # Optional tuning on top of the base file (later files win) -- the web
        # dashboard's SLAM settings write this; see slam_params_overrides.
        overrides = LaunchConfiguration('slam_params_overrides').perform(context)
        if overrides and os.path.isfile(overrides):
            param_files.append(overrides)
        return [
            LifecycleNode(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                namespace='',
                output='screen',
                parameters=[
                    *param_files,
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
            'use_lidar',
            default_value='true',
            description='Start the RPLIDAR driver (see hardware.launch.py). SLAM needs it -- false only for bench tests.',
        ),
        DeclareLaunchArgument(
            'use_oak',
            default_value='true',
            description='Start the OAK-D driver (see hardware.launch.py). Not needed to map; false runs lidar-only.',
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
            'race',
            default_value='false',
            description='Plan B: race on the LIVE slam map (no saved map) -- Nav2 + start trigger + '
                        'lap_navigator instead of the frontier explorer. Start on the marked start spot.',
        ),
        DeclareLaunchArgument(
            'route',
            default_value='',
            description='Race mode route: a name in config/maps (<name>.route.yaml) or a path',
        ),
        DeclareLaunchArgument(
            'num_laps',
            default_value='0',
            description='Race mode laps; 0 = the route file\'s / course default',
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='0.0',
            description='Cap Nav2 forward speed (m/s); 0 = the params file as-is',
        ),
        DeclareLaunchArgument(
            'max_turn',
            default_value='0.0',
            description='Cap Nav2 turn rate (rad/s); 0 = the params file as-is',
        ),
        DeclareLaunchArgument(
            'slam_params_overrides',
            default_value='',
            description='Optional YAML (slam_toolbox: ros__parameters: ...) loaded on top '
                        'of config/slam_toolbox_params.yaml -- the web dashboard\'s SLAM '
                        'settings write one',
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

        # Real-hardware layer (URDF TF, lidar/OAK-D drivers, motor/watchdog/
        # odometry/IMU nodes) -- Gazebo provides all of this in sim.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_bringup, 'launch', 'hardware.launch.py'])
            ),
            condition=UnlessCondition(LaunchConfiguration('use_sim')),
            launch_arguments={
                'use_lidar': LaunchConfiguration('use_lidar'),
                'use_oak': LaunchConfiguration('use_oak'),
            }.items(),
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
        OpaqueFunction(function=_nav2_and_race),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2, 'launch', 'navigation_launch.py'])
            ),
            condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('teleop'), 'true')),
            launch_arguments={
                'namespace': '',
                'use_sim_time': LaunchConfiguration('use_sim'),
                'params_file': LaunchConfiguration('nav2_params_resolved'),  # set by _nav2_and_race
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
            # Not in teleop mode (you drive) nor race mode (lap_navigator drives)
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('teleop'), "' != 'true' and '",
                LaunchConfiguration('race'), "' != 'true'"])),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim'),
                'save_map_name': LaunchConfiguration('map_save_path'),
                'heading_turn_penalty': 3.0,
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
        ) if _rviz_available() else LogInfo(msg='rviz2 not installed or no display -- skipping RViz'),
    ])
