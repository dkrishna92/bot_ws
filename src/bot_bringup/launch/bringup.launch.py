"""Bringup (race) launch file.

Composes the nodes owned by bot_motor, bot_ultrasonic, bot_safety, and
bot_perception. This package intentionally contains no node
implementations of its own -- see CLAUDE.md for why those live in
separate per-concern packages. Sensor drivers (RPLIDAR, OAK-D) launch
only if their packages are installed; missing drivers don't block launch.

Nav2 here localizes against a pre-built map (amcl + map_server) rather
than building one -- run mapping.launch.py first to produce
config/maps/map.yaml for the course you're about to race on.

Usage:
    # Run with actual hardware (sensors must be installed)
    ros2 launch bot_bringup bringup.launch.py

    # Run with Gazebo simulation (no real sensors needed)
    ros2 launch bot_bringup bringup.launch.py use_sim:=true
"""
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotEqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bringup = get_package_share_directory('bot_bringup')

    # Try to find optional packages; skip if not installed
    try:
        pkg_rplidar = get_package_share_directory('rplidar_ros')
        rplidar_available = True
    except PackageNotFoundError:
        pkg_rplidar = None
        rplidar_available = False

    try:
        pkg_oak = get_package_share_directory('depthai_ros_driver')
        oak_available = True
    except PackageNotFoundError:
        pkg_oak = None
        oak_available = False

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

        # Set use_sim_time parameter when using simulation
        SetEnvironmentVariable(
            name='ROS_DOMAIN_ID',
            value='0'
        ),

        # Conditionally include Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo_sim.launch.py'])
            ),
            condition=IfCondition(LaunchConfiguration('use_sim')),
        ),

        # Custom nodes (disabled on laptop, enabled on Pi with real hardware)
        # Uncomment these when running on Raspberry Pi 5
        # Node(
        #     package="bot_safety",
        #     executable="watchdog_node",
        #     name="watchdog_node",
        #     output="screen",
        # ),
        # Node(
        #     package="bot_ultrasonic",
        #     executable="ultrasonic_node",
        #     name="ultrasonic_node",
        #     output="screen",
        # ),
        # Node(
        #     package="bot_motor",
        #     executable="motor_node",
        #     name="motor_node",
        #     output="screen",
        # ),
        # Node(
        #     package="bot_perception",
        #     executable="start_trigger_node",
        #     name="start_trigger_node",
        #     output="screen",
        # ),
        # Node(
        #     package="bot_perception",
        #     executable="sensor_fusion_node",
        #     name="sensor_fusion_node",
        #     output="screen",
        # ),
    ]

    # RPLidar driver - only if package is installed and not in sim mode
    if rplidar_available:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_rplidar, 'launch', 'rplidar_a1_launch.py'])
                ),
                condition=IfCondition(NotEqualsSubstitution(
                    LaunchConfiguration('use_sim'), 'true'
                )),
                launch_arguments={
                    'serial_port': '/dev/ttyUSB0',
                    'frame_id': 'lidar_link',
                }.items(),
            )
        )

    # OAK-D S2 driver - only if package is installed and not in sim mode
    if oak_available:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_oak, 'launch', 'oak_d_lite_launch.py'])
                ),
                condition=IfCondition(NotEqualsSubstitution(
                    LaunchConfiguration('use_sim'), 'true'
                )),
            )
        )

    # Robot localization (EKF odometry fusion) - only if package is installed
    if robot_loc_available:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_robot_loc, 'launch', 'ekf.launch.py'])
                ),
                launch_arguments={
                    'namespace': '',
                    'params_file': PathJoinSubstitution([pkg_bringup, 'config', 'ekf_params.yaml']),
                }.items(),
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
                    'params_file': PathJoinSubstitution([pkg_bringup, 'config', 'nav2_params.yaml']),
                    'autostart': 'true',
                }.items(),
            )
        )

    return LaunchDescription(actions)
