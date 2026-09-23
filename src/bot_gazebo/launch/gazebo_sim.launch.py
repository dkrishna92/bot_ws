#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

# Per-world default spawn pose, near each course's start_signal_frame (the
# only start-line marker still present in these files -- the old "slash"
# placeholder vehicle model these were imported alongside no longer exists
# in either file). These are best-effort estimates from the frame's own
# pose, not visually verified -- override with spawn_x/spawn_y/spawn_yaw
# launch args to nudge live in the GUI rather than editing this file.
_DEFAULT_SPAWN_POSES = {
    'obstacle_course_cfr': (-0.7, 0.0, 0.0),
    'speed_course_cfr': (19.0, 4.75, 3.14159265),
}
_FALLBACK_SPAWN_POSE = (-0.7, 0.0, 0.0)


def _spawn_robot(context, *args, **kwargs):
    """Build the spawn Node with a per-world default pose, resolved at launch
    time since 'world' is only known once substitutions are performed."""
    world_name = LaunchConfiguration('world').perform(context)
    default_x, default_y, default_yaw = _DEFAULT_SPAWN_POSES.get(world_name, _FALLBACK_SPAWN_POSE)

    def _resolve(arg_name, default_value):
        value = LaunchConfiguration(arg_name).perform(context)
        return str(default_value) if value == 'nan' else value

    return [Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bot',
            '-x', _resolve('spawn_x', default_x),
            '-y', _resolve('spawn_y', default_y),
            '-z', _resolve('spawn_z', 0.0),
            '-Y', _resolve('spawn_yaw', default_yaw),
        ],
        output='screen',
    )]


def generate_launch_description():
    """Launch Gazebo Sim (ros-gz) and spawn the robot."""

    pkg_gazebo = get_package_share_directory('bot_gazebo')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    # File paths
    urdf_file = os.path.join(pkg_gazebo, 'urdf', 'bot.urdf.xacro')

    # LaunchConfiguration references
    world_arg = LaunchConfiguration('world')
    headless_arg = LaunchConfiguration('headless')

    # Generate URDF from Xacro
    robot_description_content = Command(['xacro ', urdf_file])

    # Set environment for Gazebo to find models
    gz_resource_path = os.path.join(pkg_gazebo, 'models')

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            gz_resource_path,
        ),
        DeclareLaunchArgument(
            'world',
            default_value='speed_course_cfr',
            description='World to load: obstacle_course_cfr, speed_course_cfr, obstacle_course, speed_course, or onshape_course',
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='nan',
            description='Override the robot spawn X position (default: per-world start-line estimate)',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='nan',
            description='Override the robot spawn Y position (default: per-world start-line estimate)',
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='nan',
            description='Override the robot spawn Z position (default: 0.0)',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='nan',
            description='Override the robot spawn yaw in radians (default: per-world start-line estimate)',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo headless',
        ),
        DeclareLaunchArgument(
            'publish_static_map_odom',
            default_value='true',
            description='Publish a static map->odom identity TF; disable when slam_toolbox or amcl owns that transform instead',
        ),

        # Publish robot description (URDF) - must be before spawn.
        # ParameterValue(..., value_type=str) forces this to be treated as
        # a plain string; without it, launch_ros guesses the type by trying
        # to YAML-parse the XML content, which is fragile -- content as
        # innocuous as a colon inside an XML comment can make it look
        # enough like a YAML mapping to fail that guess.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='log',
            parameters=[
                {
                    'robot_description': ParameterValue(robot_description_content, value_type=str),
                    'use_sim_time': True,
                }
            ],
        ),

        # Gazebo Sim (ros-gz) - includes server and optional GUI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                # -r: start the simulation running instead of paused. Without
                # it, gz sim opens paused and physics never steps until
                # someone clicks Play in the GUI (or a /world/.../control
                # service call unpauses it) -- likely also why DiffDrive's
                # odometry init has been flaky, since plugin Configure()
                # timing relative to the first physics step becomes
                # dependent on exactly when a human (or nothing) unpauses it.
                'gz_args': Command([
                    'echo -r ',
                    os.path.join(pkg_gazebo, 'worlds'),
                    '/',
                    world_arg,
                    '.world -v 4'
                ]),
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # Spawn the robot from the published robot_description, at a
        # per-world start-line pose (see _DEFAULT_SPAWN_POSES above -- the
        # placeholder "slash" model these courses were imported alongside
        # no longer exists in either file, so this is a best estimate from
        # each course's start_signal_frame; override with spawn_x/spawn_y/
        # spawn_yaw to nudge it live instead of editing this file).
        OpaqueFunction(function=_spawn_robot),

        # Publish static transform: map -> odom -- only when nothing else owns it (see publish_static_map_odom arg above)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='log',
            condition=IfCondition(LaunchConfiguration('publish_static_map_odom')),
        ),

        # Create a bridge for joint states and transforms
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # GZ -> ROS only ('[') -- every node launched with
                # use_sim_time:=true (EKF, slam_toolbox, Nav2, RViz) blocks
                # waiting for this; without it nothing using sim time ever
                # actually starts processing.
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                # IMU from bot.urdf.xacro's imu_link -- feeds robot_localization's EKF.
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                # OAK-D depth, simulated via bot.urdf.xacro's oak_depth_sensor
                # (gz-sim depth_camera) -- only the auto-generated point cloud
                # is bridged (remapped to /oak/points below), not the raw
                # depth image itself, since nothing in sim subscribes to that.
                # Feeds local_costmap's voxel_layer in nav2_params.yaml.
                '/oak/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                # Wheel joint positions, from the gz-sim-joint-state-publisher-
                # system plugin on bot.urdf.xacro -- needed for
                # robot_state_publisher to compute base_link -> *_wheel TF.
                # Bridges the plugin's pinned 'joint_states' topic (see its
                # <topic> in bot.urdf.xacro) rather than the world-namespaced
                # auto name, which depends on the world's internal name.
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                # Diagnostic contact sensor from bot.urdf.xacro -- lets real
                # collisions (e.g. clipping a bale) be observed directly via
                # `ros2 topic echo /bot_contacts` instead of inferred from
                # TF/cmd_vel mismatches.
                '/bot_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                # Ground-truth model pose (bot.urdf.xacro's PosePublisher
                # plugin) -- sim-only diagnostic, NOT part of the real robot;
                # used by scripts/compare_odom_to_ground_truth.py to check
                # DiffDrive's kinematic /odom against physics ground truth
                # (see that plugin's comment in bot.urdf.xacro).
                '/model/bot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
            ],
            remappings=[
                ('/oak/depth/points', '/oak/points'),
            ],
            output='screen',
        ),
    ])
