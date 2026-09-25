"""Real-hardware layer shared by bringup.launch.py and mapping.launch.py.

Everything Gazebo provides for free in sim, the real robot needs from
somewhere else: robot_state_publisher for the URDF's static TF
(base_link -> lidar_link/imu_link/camera_link), the RPLIDAR and OAK-D
drivers, and the Pi-only nodes (motor_node, watchdog_node, wheel_odom_node,
bno055_node). Both top-level launches include this with
condition=unless use_sim, so it never runs alongside Gazebo.

Sensor drivers are skipped (with a log line) if their package isn't
installed, same as before this was split out.

Usage (normally included, but runnable alone for bench-testing sensors):
    ros2 launch bot_bringup hardware.launch.py
"""
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def _share(pkg):
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError:
        return None


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('bot_gazebo')
    pkg_bringup = get_package_share_directory('bot_bringup')
    pkg_sllidar = _share('sllidar_ros2')
    pkg_oak = _share('depthai_ros_driver')

    actions = [
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/rplidar',
            description='RPLIDAR serial device (udev symlink from scripts/setup_pi.sh; /dev/ttyUSB0 otherwise)',
        ),
        DeclareLaunchArgument(
            'imu_i2c_bus',
            default_value='3',
            description='I2C bus number of the BNO055 (/dev/i2c-N)',
        ),
        DeclareLaunchArgument(
            'teensy_port',
            default_value='/dev/teensy',
            description='Teensy (encoders + ultrasonics) serial device (udev symlink from scripts/setup_pi.sh; /dev/ttyACM0 otherwise)',
        ),

        # Static TF from the same URDF sim uses. Wheel joints are continuous
        # and nothing publishes joint_states on hardware, so wheel links
        # won't appear in TF -- harmless, nothing in Nav2/EKF needs them.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='log',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', PathJoinSubstitution([pkg_gazebo, 'urdf', 'bot.urdf.xacro'])]),
                    value_type=str,
                ),
                'use_sim_time': False,
            }],
        ),

        Node(package='bot_safety', executable='watchdog_node', name='watchdog_node', output='screen'),
        Node(package='bot_motor', executable='motor_node', name='motor_node', output='screen'),
        # wheel_odom_node also owns the Teensy's ultrasonic reporting (see
        # teensy_ws and bot_odometry's docstring) -- no separate node.
        Node(
            package='bot_odometry',
            executable='wheel_odom_node',
            name='wheel_odom_node',
            output='screen',
            parameters=[{'serial_port': LaunchConfiguration('teensy_port')}],
        ),
        # BNO055 is on I2C3 (GPIO22/23, header pins 15/16), not the default
        # I2C1 on GPIO2/3 -- I2C1 on this Pi times out with nothing attached.
        # Needs `dtoverlay=i2c3-pi5,pins_22_23` in /boot/firmware/config.txt.
        Node(
            package='bot_imu',
            executable='bno055_node',
            name='bno055_node',
            output='screen',
            parameters=[{'i2c_bus': LaunchConfiguration('imu_i2c_bus')}],
        ),
    ]

    # RPLIDAR S2 (identified 2026-09-25: 1 Mbaud, DenseBoost 32 kHz, 30 m).
    # The apt rplidar_ros (2.1.0, Slamtec SDK 1.12) segfaults on scan start
    # with this unit in every scan mode, so this uses Slamtec's sllidar_ros2
    # (SDK 2.1) built from source into src/ -- scripts/setup_pi.sh clones it.
    if pkg_sllidar is not None:
        actions.append(
            Node(
                package='sllidar_ros2',
                executable='sllidar_node',
                name='sllidar_node',
                output='screen',
                parameters=[
                    PathJoinSubstitution([pkg_bringup, 'config', 'rplidar_params.yaml']),
                    {'serial_port': LaunchConfiguration('lidar_port')},
                ],
            )
        )
    else:
        actions.append(LogInfo(msg='sllidar_ros2 not built -- skipping lidar driver (see scripts/setup_pi.sh)'))

    # OAK-D S2 driver. 'oak_d_lite_launch.py' (an earlier filename used
    # here) doesn't exist in depthai_ros_driver -- 'camera.launch.py' is the
    # real generic launch file. parent_frame attaches the driver's own
    # oak-* TF tree under the URDF's camera_link instead of leaving it as a
    # disconnected tree.
    #
    # Wrapped in a scoped GroupAction: camera.launch.py sets launch configs
    # like use_composition='true' (lowercase), name and namespace, which
    # otherwise leak into every later include -- nav2_bringup then evals
    # PythonExpression('not true') and the whole launch dies with
    # "name 'true' is not defined".
    if pkg_oak is not None:
        actions.append(GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_oak, 'launch', 'camera.launch.py'])
                ),
                launch_arguments={
                    'name': 'oak',
                    'camera_model': 'OAK-D-S2',
                    'parent_frame': 'camera_link',
                    'params_file': PathJoinSubstitution([pkg_bringup, 'config', 'oak_params.yaml']),
                    # Skip the RGB rectify pipeline -- not needed for the
                    # depth-only point cloud below, and CLAUDE.md's hardware
                    # notes call for the host to receive depth (and, later,
                    # on-device detections), never raw/rectified RGB frames.
                    'rectify_rgb': 'false',
                }.items(),
            ),
        ]))

        # Depth -> point cloud for the local costmap's voxel_layer (see
        # nav2_params.yaml). depthai_ros_driver's own built-in point cloud
        # path only produces a colorized cloud via PointCloudXyzrgbNode,
        # which requires the RGB stream -- exactly the raw-frame path
        # CLAUDE.md says to avoid. Loads into the component container
        # camera.launch.py just created ('<name>_container').
        actions.append(
            LoadComposableNodes(
                target_container='/oak_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='depth_image_proc',
                        plugin='depth_image_proc::PointCloudXyzNode',
                        name='oak_points_node',
                        remappings=[
                            ('image_rect', 'oak/stereo/image_raw'),
                            ('points', 'oak/points'),
                        ],
                    ),
                ],
            )
        )
    else:
        actions.append(LogInfo(msg='depthai_ros_driver not installed -- skipping OAK-D driver'))

    return LaunchDescription(actions)
