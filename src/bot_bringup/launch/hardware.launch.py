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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
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
    pkg_rplidar = _share('rplidar_ros')
    pkg_oak = _share('depthai_ros_driver')

    actions = [
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/rplidar',
            description='RPLIDAR serial device (udev symlink from scripts/setup_pi.sh; /dev/ttyUSB0 otherwise)',
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
        Node(package='bot_imu', executable='bno055_node', name='bno055_node', output='screen'),
    ]

    if pkg_rplidar is not None:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_rplidar, 'launch', 'rplidar_a1_launch.py'])
                ),
                launch_arguments={
                    'serial_port': LaunchConfiguration('lidar_port'),
                    'frame_id': 'lidar_link',
                }.items(),
            )
        )
    else:
        actions.append(LogInfo(msg='rplidar_ros not installed -- skipping lidar driver'))

    # OAK-D S2 driver. 'oak_d_lite_launch.py' (an earlier filename used
    # here) doesn't exist in depthai_ros_driver -- 'camera.launch.py' is the
    # real generic launch file. parent_frame attaches the driver's own
    # oak-* TF tree under the URDF's camera_link instead of leaving it as a
    # disconnected tree.
    if pkg_oak is not None:
        actions.append(
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
            )
        )

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
