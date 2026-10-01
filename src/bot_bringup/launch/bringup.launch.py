"""Bringup (race) launch file.

Composes the nodes owned by bot_motor, bot_odometry, bot_safety, bot_imu,
and bot_perception (the Pi-only ones via hardware.launch.py). This package intentionally contains no node
implementations of its own -- see CLAUDE.md for why those live in
separate per-concern packages. Sensor drivers (RPLIDAR, OAK-D) launch
only if their packages are installed; missing drivers don't block launch.

Nav2 here localizes against a pre-built map (amcl + map_server) rather
than building one -- run mapping.launch.py first to produce
config/maps/map.yaml for the course you're about to race on.

Usage:
    # Run with actual hardware (headless Pi: no display for RViz)
    ros2 launch bot_bringup bringup.launch.py rviz:=false

    # Run with Gazebo simulation (no real sensors needed)
    ros2 launch bot_bringup bringup.launch.py use_sim:=true
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotEqualsSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


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


def _read_pgm_size(path):
    """Parse just the width/height out of a P5 (binary greyscale) PGM
    header -- stdlib only, no Pillow, same rationale as
    bot_web_control's hand-rolled PNG encoder."""
    with open(path, 'rb') as f:
        data = f.read(256)  # header is always tiny; way more than enough
    if not data.startswith(b'P5'):
        raise ValueError(f"{path} is not a binary (P5) PGM file")
    fields = []
    i = 2
    while len(fields) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j]))
        i = j
    width, height, _maxval = fields
    return width, height


def _resolve_map(context):
    """The 'map' launch argument: a saved map's name in config/maps (e.g.
    'map', 'Test') or a path to a .yaml. Names resolve in the installed
    share dir; maps saved after the last colcon build aren't installed yet,
    so pass their full path instead (the web dashboard does)."""
    value = LaunchConfiguration('map').perform(context).strip()
    if value.endswith('.yaml') or os.sep in value:
        return os.path.abspath(os.path.expanduser(value))
    return os.path.join(get_package_share_directory('bot_bringup'), 'config', 'maps', f'{value}.yaml')


def _capped_nav2_params(path, max_speed, max_turn, initial_pose=None):
    """Nav2 params with speeds capped for slow first laps: returns (path to
    use, what changed). Caps the controller (DWB max_vel_x/max_speed_xy/
    max_vel_theta, MPPI vx_max/wz_max, RPP + rotation shim
    desired_linear_vel/rotate_to_heading_angular_vel), the velocity smoother and the spin
    behaviour -- the smoother alone would clamp output while the controller
    still planned for full speed. max_speed/max_turn <= 0 leave that axis
    alone; nothing capped returns the original path."""
    if max_speed <= 0 and max_turn <= 0 and initial_pose is None:
        return path, []
    with open(path) as f:
        params = yaml.safe_load(f) or {}
    changed = []
    if initial_pose is not None:
        amcl = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
        x, y, yaw = initial_pose
        amcl['set_initial_pose'] = True
        amcl['initial_pose'] = {'x': float(x), 'y': float(y), 'z': 0.0, 'yaw': float(yaw)}
        changed.append('amcl.initial_pose')

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


def _effective_initial_pose(context, map_yaml_path, amcl_params):
    """Where AMCL starts: (x, y, yaw, source, override) or None.

    1. initial_pose:=x,y,yaw launch argument
    2. <map>.start.yaml next to the map ({x, y, yaw}; the dashboard's "Set
       race start pose here" writes it) -- the start pose belongs to the
       map, since every map has its own frame
    3. nav2_params.yaml's amcl.initial_pose (one hardcoded pose for every
       map -- the sim rehearsal 2026-10-01 had AMCL start 20 m / 180 deg
       away from the robot this way, with every node reporting healthy)
    override is True for 1 and 2 (the params file needs rewriting)."""
    arg = LaunchConfiguration('initial_pose').perform(context).strip()
    if arg:
        x, y, yaw = (float(v) for v in arg.split(','))
        return x, y, yaw, 'initial_pose argument', True
    start_file = map_yaml_path[:-len('.yaml')] + '.start.yaml' if map_yaml_path.endswith('.yaml') else ''
    if start_file and os.path.isfile(start_file):
        with open(start_file) as f:
            data = yaml.safe_load(f) or {}
        return float(data['x']), float(data['y']), float(data.get('yaw', 0.0)), start_file, True
    if amcl_params.get('set_initial_pose'):
        pose = amcl_params.get('initial_pose') or {}
        if pose.get('x') is None or pose.get('y') is None:
            raise ValueError("amcl.set_initial_pose is true but initial_pose.x/y is missing")
        return float(pose['x']), float(pose['y']), float(pose.get('yaw', 0.0)), 'nav2 params file (hardcoded)', False
    return None


def _resolve_route(context, map_yaml_path):
    """lap_navigator's route file: the 'route' argument (a name in
    config/maps -> <name>.route.yaml, or a path), else the map's own
    <map>.route.yaml next to it if one was recorded, else '' (lap_navigator
    then falls back to its built-in, sim-derived checkpoints)."""
    value = LaunchConfiguration('route').perform(context).strip()
    if value:
        if value.endswith('.yaml') or os.sep in value:
            return os.path.abspath(os.path.expanduser(value))
        return os.path.join(get_package_share_directory('bot_bringup'), 'config', 'maps', f'{value}.route.yaml')
    paired = map_yaml_path[:-len('.yaml')] + '.route.yaml' if map_yaml_path.endswith('.yaml') else ''
    return paired if paired and os.path.isfile(paired) else ''


def _validate_map(context, *args, **kwargs):
    """Fail fast, with a clear message, instead of letting a bad map surface
    as a cryptic Nav2 failure many steps downstream (amcl silently never
    localizing, or the planner's "Failed to create plan with tolerance of"
    once a run is already underway) -- see CLAUDE.md's "AMCL's initial pose
    is a hardcoded constant" and "A restored/stale map can silently
    mismatch..." open items for the real incident this guards against: a
    saved map whose bounds fell 0.16m short of nav2_params.yaml's hardcoded
    initial_pose, discovered only by manually cross-checking the two files
    after a confusing live failure.

    Checks, in order: the configured map file actually exists and parses,
    and (only when set_initial_pose is true, i.e. there's a fixed pose to
    check against at all) that amcl's initial_pose actually falls within
    the map's bounds with at least map_bounds_margin_m to spare on every
    side -- not just technically inside, since "just barely inside" is
    exactly as fragile as "just barely outside" once amcl's own pose
    covariance and the robot's footprint are accounted for.
    """
    pkg_bringup = get_package_share_directory('bot_bringup')
    map_yaml_path = _resolve_map(context)
    # Hand the resolved path to the nav2_bringup include that follows
    set_map = SetLaunchConfiguration('map_yaml', map_yaml_path)
    nav2_params_path = os.path.join(
        pkg_bringup, 'config', LaunchConfiguration('nav2_params_file').perform(context)
    )
    margin = float(LaunchConfiguration('map_bounds_margin_m').perform(context))

    if not os.path.isfile(map_yaml_path):
        return [LogInfo(msg=(
            f"FATAL: map file not found at {map_yaml_path} -- run "
            "mapping.launch.py for this course first (see that file's "
            "docstring), or check config/maps/map.yaml was actually "
            "installed (colcon build --packages-select bot_bringup)."
        )), Shutdown(reason='map file missing')]

    try:
        with open(map_yaml_path) as f:
            map_meta = yaml.safe_load(f)
        origin_x, origin_y = map_meta['origin'][0], map_meta['origin'][1]
        resolution = map_meta['resolution']
        pgm_path = os.path.join(os.path.dirname(map_yaml_path), map_meta['image'])
        width_px, height_px = _read_pgm_size(pgm_path)
    except Exception as exc:
        return [LogInfo(msg=(
            f"FATAL: couldn't parse map at {map_yaml_path}: {exc!r} -- "
            "the file may be truncated/corrupt, or map.yaml's 'image' "
            "field doesn't match the actual .pgm filename next to it."
        )), Shutdown(reason='map file unreadable')]

    map_min_x, map_max_x = origin_x, origin_x + width_px * resolution
    map_min_y, map_max_y = origin_y, origin_y + height_px * resolution

    if not os.path.isfile(nav2_params_path):
        return [LogInfo(msg=(
            f"FATAL: nav2 params file not found at {nav2_params_path}"
        )), Shutdown(reason='nav2 params file missing')]
    with open(nav2_params_path) as f:
        nav2_params = yaml.safe_load(f)
    amcl_params = (nav2_params.get('amcl') or {}).get('ros__parameters') or {}

    try:
        start = _effective_initial_pose(context, map_yaml_path, amcl_params)
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:
        return [LogInfo(msg=f"FATAL: bad initial pose: {exc!r}"), Shutdown(reason='initial pose unreadable')]
    if start is not None:
        px, py, pyaw, pose_source, pose_override = start
        if not (map_min_x + margin <= px <= map_max_x - margin
                and map_min_y + margin <= py <= map_max_y - margin):
            return [LogInfo(msg=(
                f"FATAL: amcl's initial_pose ({px}, {py}) from "
                f"{pose_source} falls outside {map_yaml_path}'s bounds "
                f"(x: [{map_min_x:.3f}, {map_max_x:.3f}], "
                f"y: [{map_min_y:.3f}, {map_max_y:.3f}]) with the required "
                f"{margin}m margin. This map was very likely saved from an "
                "incomplete mapping run that never covered the actual spawn "
                "point -- see CLAUDE.md's 'A restored/stale map can "
                "silently mismatch...' open item. Regenerate the map (or "
                "resume-and-extend it, using serialize_map for the "
                "checkpoint -- see mapping.launch.py's docstring) so it "
                "actually covers this pose, or override map_bounds_margin_m "
                "if this margin is being overly strict for a known-good map."
            )), Shutdown(reason='initial_pose outside map bounds')]

    actions = [set_map, LogInfo(msg=(
        f"Map bounds check OK: {map_yaml_path} covers amcl's initial_pose "
        f"with >= {margin}m margin (or set_initial_pose is false)."
    ))]
    if start is not None:
        actions.append(LogInfo(msg=(
            f"AMCL starts at x={px:.2f} y={py:.2f} yaw={pyaw:.2f} from {pose_source}"
            + ("" if pose_override else " -- NOT specific to this map: record one (dashboard 'Set race start "
                                        "pose here' -> <map>.start.yaml) or pass initial_pose:=x,y,yaw"))))

    # Speed caps for slow first laps (max_speed/max_turn launch args), plus
    # the map's own start pose when it didn't come from the params file
    params_path, changed = _capped_nav2_params(
        nav2_params_path,
        float(LaunchConfiguration('max_speed').perform(context) or 0),
        float(LaunchConfiguration('max_turn').perform(context) or 0),
        initial_pose=(px, py, pyaw) if start is not None and pose_override else None)
    actions.append(SetLaunchConfiguration('nav2_params_resolved', params_path))
    if changed:
        actions.append(LogInfo(msg=f"Nav2 params adjusted ({', '.join(changed)}) via {params_path}"))

    # Multi-lap course navigator (bot_navigation) -- subscribes to
    # start_trigger_node's start_signal and sends Nav2 the route once it
    # fires. Started here, after the map is resolved, because its route is
    # tied to the map (<map>.route.yaml, recorded on that map).
    route_file = _resolve_route(context, map_yaml_path)
    actions.append(LogInfo(msg=(
        f"Race route: {route_file}" if route_file else
        "No route file for this map -- lap_navigator falls back to its BUILT-IN sim checkpoints, "
        "which are NOT valid on the real course (record one: dashboard 'Add checkpoint here').")))
    actions.append(Node(
        package='bot_navigation',
        executable='lap_navigator_node',
        name='lap_navigator_node',
        output='screen',
        parameters=[{
            'course': LaunchConfiguration('course'),
            'num_laps': LaunchConfiguration('num_laps'),
            'route_file': route_file,
            'use_sim_time': LaunchConfiguration('use_sim'),
        }],
    ))
    return actions


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bringup = get_package_share_directory('bot_bringup')

    # Try to find optional packages; skip if not installed
    try:
        pkg_nav2 = get_package_share_directory('nav2_bringup')
        nav2_available = True
    except PackageNotFoundError:
        pkg_nav2 = None
        nav2_available = False

    try:
        pkg_robot_loc = get_package_share_directory('robot_localization')
        robot_loc_available = True
    except PackageNotFoundError:
        pkg_robot_loc = None
        robot_loc_available = False

    actions = [
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Launch with Gazebo simulation',
        ),
        DeclareLaunchArgument(
            'route',
            default_value='',
            description="lap_navigator's route: a name in config/maps (<name>.route.yaml) or a path. "
                        "Default: the map's own <map>.route.yaml if one was recorded",
        ),
        DeclareLaunchArgument(
            'initial_pose',
            default_value='',
            description="AMCL's start pose 'x,y,yaw' in the map. Default: the map's own <map>.start.yaml, "
                        "else nav2 params' amcl.initial_pose",
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='0.0',
            description='Cap Nav2 forward speed (m/s) for slow first laps; 0 = the params file as-is',
        ),
        DeclareLaunchArgument(
            'max_turn',
            default_value='0.0',
            description='Cap Nav2 turn rate (rad/s); 0 = the params file as-is',
        ),
        DeclareLaunchArgument(
            'map',
            default_value='map',
            description="Map to race on: a saved map's name in config/maps "
                        "(e.g. 'map', 'Test') or a path to its .yaml",
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 with the Nav2 default view',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Start the RPLIDAR driver (see hardware.launch.py). false = no localization, not drivable.',
        ),
        DeclareLaunchArgument(
            'use_oak',
            default_value='true',
            description='Start the OAK-D driver (see hardware.launch.py). false disables the vision start; start manually.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value='nav2_params.yaml',
            description=(
                'Filename (relative to bot_bringup/config, not a path) of '
                'the Nav2 params file to load -- swap to A/B test planner/'
                'controller alternatives without editing this launch file, '
                'e.g. nav2_params_thetastar.yaml, nav2_params_smac_lattice.yaml, '
                'nav2_params_mppi.yaml. See CLAUDE.md\'s Nav2 planner/'
                'controller comparison section.'
            ),
        ),
        DeclareLaunchArgument(
            'map_bounds_margin_m',
            default_value='0.5',
            description=(
                'Fail launch at startup unless the configured map (config/'
                'maps/map.yaml) covers nav2_params_file\'s amcl.initial_pose '
                'with at least this much margin on every side -- catches a '
                'stale/undersized map before it surfaces as a confusing '
                'Nav2 failure mid-run. See CLAUDE.md\'s "A restored/stale '
                'map can silently mismatch..." open item for the incident '
                'this guards against. Override (e.g. to 0.0) only for a '
                'map you\'ve already manually confirmed is fine.'
            ),
        ),

        # Course/lap count for bot_navigation's lap_navigator_node, which
        # sends Nav2 the race route once bot_perception's start_trigger_node
        # detects the vision start signal. Default here must match
        # gazebo_sim.launch.py's own default 'world' arg -- bringup.launch.py
        # doesn't currently forward a 'world' launch arg through to Gazebo,
        # so keep these in sync by hand if that ever changes.
        DeclareLaunchArgument(
            'course',
            default_value='speed_course_cfr',
            description=(
                "Which course's checkpoints to race (see "
                'bot_navigation/lap_navigator_node.py): speed_course_cfr or '
                'obstacle_course_cfr.'
            ),
        ),
        DeclareLaunchArgument(
            'num_laps',
            default_value='0',
            description="Laps to run; 0 uses the course's own default (3 for speed, 2 for obstacle).",
        ),

        # (No ROS_DOMAIN_ID override here -- this used to force domain 0,
        # which silently cut bringup off from the web dashboard and any
        # terminal on another domain, and put the robot on the default
        # domain every other team's ROS 2 robot uses. The domain and
        # discovery range come from the environment; see RACE_DAY.md.)

        # Conditionally include Gazebo -- amcl owns map->odom here, so tell
        # gazebo_sim.launch.py not to also publish it
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo_sim.launch.py'])
            ),
            condition=IfCondition(LaunchConfiguration('use_sim')),
            launch_arguments={'publish_static_map_odom': 'false'}.items(),
        ),

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

        # Vision-based autonomous start trigger (bot_perception) -- runs in
        # both sim and on real hardware, unlike the Pi-only nodes above.
        # Two variants because the image topic differs: sim's generic
        # camera sensor bridges to /camera/image_raw (bot.urdf.xacro's
        # camera_sensor, see gazebo_sim.launch.py's bridge list), while
        # real hardware's depthai_ros_driver publishes on /oak/rgb/image_raw
        # (start_trigger_node's own default, left unset here). It only
        # detects the signal and publishes start_signal -- lap_navigator_node
        # below is what actually sends Nav2 the route.
        Node(
            package='bot_perception',
            executable='start_trigger_node',
            name='start_trigger_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_sim')),
            parameters=[{
                'image_topic': '/camera/image_raw',
                'use_sim_time': True,
            }],
        ),
        Node(
            package='bot_perception',
            executable='start_trigger_node',
            name='start_trigger_node',
            output='screen',
            condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('use_sim'), 'true')),
            parameters=[{
                'use_sim_time': False,
            }],
        ),

        # lap_navigator_node is started by _validate_map (once the map --
        # and so its paired route file -- is resolved).
    ]

    # Robot localization (EKF odometry fusion) - only if package is installed.
    # robot_localization's own ekf.launch.py ignores 'namespace'/'params_file'
    # launch arguments entirely -- it hardcodes its package's example params
    # (odom0: example/odom, no use_sim_time), so we launch the node directly
    # with our own config instead of including that launch file.
    if robot_loc_available:
        actions.append(
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    PathJoinSubstitution([pkg_bringup, 'config', 'ekf_params.yaml']),
                    {'use_sim_time': LaunchConfiguration('use_sim')},
                ],
            )
        )

    # Nav2 full stack (amcl + map_server localization against the map built
    # by mapping.launch.py, plus navigation) - only if package is installed
    if nav2_available:
        # Runs (and can abort the whole launch via Shutdown) before Nav2
        # itself starts -- see _validate_map's own docstring for what this
        # catches and why.
        actions.append(OpaqueFunction(function=_validate_map))

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_nav2, 'launch', 'bringup_launch.py'])
                ),
                launch_arguments={
                    'namespace': '',
                    'use_namespace': 'false',
                    # nav2_bringup's bringup_launch.py builds this into a
                    # PythonExpression string and eval()s it directly, so it
                    # must be a real Python bool literal (capitalized), not
                    # the lowercase 'true'/'false' used elsewhere here.
                    'slam': 'False',
                    'map': LaunchConfiguration('map_yaml'),  # resolved by _validate_map
                    'use_sim_time': LaunchConfiguration('use_sim'),
                    'params_file': LaunchConfiguration('nav2_params_resolved'),  # set by _validate_map
                    'autostart': 'true',
                }.items(),
            )
        )

        # RViz2 with this package's own view (robot model, TF, costmaps, the
        # panel for sending 2D Nav Goals, a ThirdPersonFollower camera that
        # actually tracks base_link, and the OAK-D depth point cloud) rather
        # than nav2_bringup's stock nav2_default_view.rviz.
        rviz_action = LogInfo(msg='rviz2 not installed or no display -- skipping RViz')
        if _rviz_available():
            rviz_action = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', PathJoinSubstitution([pkg_bringup, 'launch', 'thirdpersonviewer.rviz'])],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim')}],
                condition=IfCondition(LaunchConfiguration('rviz')),
                output='screen',
            )
        actions.append(rviz_action)

    return LaunchDescription(actions)
