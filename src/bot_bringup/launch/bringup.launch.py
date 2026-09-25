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
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotEqualsSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


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
            'rviz',
            default_value='true',
            description='Launch RViz2 with the Nav2 default view',
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

        # Set use_sim_time parameter when using simulation
        SetEnvironmentVariable(
            name='ROS_DOMAIN_ID',
            value='0'
        ),

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

        # Multi-lap course navigator (bot_navigation) -- subscribes to
        # start_trigger_node's start_signal and sends Nav2 the checkpoint
        # route for 'course' once it fires. Runs in both sim and on real
        # hardware; no image-topic split needed since it only talks to Nav2.
        Node(
            package='bot_navigation',
            executable='lap_navigator_node',
            name='lap_navigator_node',
            output='screen',
            parameters=[{
                'course': LaunchConfiguration('course'),
                'num_laps': LaunchConfiguration('num_laps'),
                'use_sim_time': LaunchConfiguration('use_sim'),
            }],
        ),
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
                    'map': PathJoinSubstitution([pkg_bringup, 'config', 'maps', 'map.yaml']),
                    'use_sim_time': LaunchConfiguration('use_sim'),
                    'params_file': PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('nav2_params_file')]),
                    'autostart': 'true',
                }.items(),
            )
        )

        # RViz2 with this package's own view (robot model, TF, costmaps, the
        # panel for sending 2D Nav Goals, a ThirdPersonFollower camera that
        # actually tracks base_link, and the OAK-D depth point cloud) rather
        # than nav2_bringup's stock nav2_default_view.rviz.
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', PathJoinSubstitution([pkg_bringup, 'launch', 'thirdpersonviewer.rviz'])],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim')}],
                condition=IfCondition(LaunchConfiguration('rviz')),
                output='screen',
            )
        )

    return LaunchDescription(actions)
