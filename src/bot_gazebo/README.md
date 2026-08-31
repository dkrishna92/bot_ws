# Gazebo Simulation

Complete Gazebo simulation environment for testing the DIY Robot Challenge vehicle without hardware.

## Quick Start

### 1. Launch with Gazebo simulation
```bash
cd /home/krishna/claudebot/bot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bot_bringup bringup.launch.py use_sim:=true
```

This will:
- Start Gazebo server and client (GUI)
- Spawn the robot at position (-4, 0, 0.2)
- Load the obstacle course world
- Start all custom nodes (motor, ultrasonic, watchdog, start trigger)
- Publish lidar scans and camera images from simulation

### 2. Headless mode (for CI/CD or remote servers)
```bash
ros2 launch bot_bringup bringup.launch.py use_sim:=true headless:=true
```

## Features

### Robot Model
- **Differential drive**: Two independent wheels with 0.35m track width
- **Dimensions**: 0.61m L × 0.41m W × 0.41m H (matches 24"×16"×16" competition spec)
- **Mass**: 11.3 kg (25 lbs competition limit)
- **Wheel radius**: 0.1m, suitable for obstacle courses

### Sensors (Simulated)
1. **Lidar (RPLIDAR)**: 
   - 360 laser scans
   - Range: 0.3–12m
   - Resolution: 0.01m
   - Topic: `/scan`

2. **Camera (OAK-D S2)**:
   - RGB: 640×480@30Hz
   - 60° horizontal FoV
   - Topics: `/camera/image_raw`, `/camera/camera_info`

3. **Ultrasonic**: Mocked by watchdog_node

### World
`obstacle_course.world` contains:
- **Start zone** (blue): Position (-5, 0)
- **End zone** (green): Position (15, 0)
- **Obstacles**:
  - Perpendicular walls
  - Box barriers
  - Narrow passage (0.6m width, simulating the competition obstacle)

## ROS 2 Topics and Services

### Published Topics (from Gazebo)
- `/scan` — LaserScan from simulated lidar
- `/camera/image_raw` — RGB image from simulated camera
- `/camera/camera_info` — Camera calibration
- `/cmd_vel` — Twist velocity input (subscribe to control robot)
- `/odom` — Odometry (X, Y, Theta from Gazebo diff-drive plugin)

### TF Frames
- `map` → `odom` (fixed, identity transform)
- `odom` → `base_link` (from diff-drive odometry)
- `base_link` → `lidar_link`, `camera_link` (fixed)

## Test Commands

### Monitor robot movement
```bash
# In one terminal, run the launch
ros2 launch bot_bringup bringup.launch.py use_sim:=true

# In another terminal:
ros2 topic echo /odom
```

### Manually drive the robot (teleop)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

### Inspect Gazebo state
```bash
# List all entities in simulation
ros2 service call /gazebo/get_model_list gazebo_msgs/srv/GetModelList

# Query robot pose
ros2 service call /gazebo/get_model_state gazebo_msgs/srv/GetModelState '{model_name: bot}'
```

### Check TF tree
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf in current directory
```

## Tuning Physics

Edit `worlds/obstacle_course.world` to adjust:
- ODE solver parameters (`<max_step_size>`, `<real_time_factor>`)
- Friction coefficients (`<mu1>`, `<mu2>` on each model)
- Gravity
- Obstacle positions/sizes

## Known Limitations

1. **No true depth sensor**: Camera publishes RGB only, no depth map. For actual stereo depth testing, integrate `gazebo_ros_depth_camera` plugin.
2. **Simplified wheel friction**: Uses ODE approximation; real robot traction may differ.
3. **No motor current/torque feedback**: Simulated motor always achieves commanded velocity; no acceleration/deceleration limits.
4. **No sensor noise**: Camera and lidar produce perfect data; add Gaussian noise with sensor plugins for realism.

## Extending the Simulation

### Add more obstacles
Edit `worlds/obstacle_course.world` and add `<model>` entries following the existing wall/box patterns.

### Improve camera simulation
Replace the generic camera plugin with `gazebo_ros_depth_camera` for depth:
```xml
<sensor type="depth_camera">
  <plugin name='depth_camera_plugin' filename='libgazebo_ros_camera.so'>
    ...
  </plugin>
</sensor>
```

### Add noise
Wrap sensor plugins with `<noise>`:
```xml
<noise>
  <type>gaussian</type>
  <mean>0</mean>
  <stddev>0.01</stddev>
</noise>
```

## Troubleshooting

**Gazebo doesn't launch:**
```bash
# Ensure Gazebo 11+ is installed
gazebo --version

# Install if missing:
sudo apt install gazebo
```

**Robot doesn't respond to `/cmd_vel`:**
- Check diff-drive plugin is loaded: `ros2 topic list | grep cmd_vel`
- Verify namespace in `urdf/bot.urdf.xacro` matches launch config

**Lidar/camera not publishing:**
```bash
ros2 topic list | grep -E 'scan|camera'
# If empty, check Gazebo console for plugin load errors
```

**Simulation runs in slow-motion:**
- Reduce world complexity (remove obstacles)
- Increase `<max_step_size>` in world file (less accurate but faster)
- Decrease `<real_time_factor>` to run slower than real-time

## Next Steps

- Integrate Nav2 for autonomous navigation planning
- Add `robot_localization` EKF for fused odometry (lidar + odometry)
- Implement the vision-based start trigger with simulated marker detection
- Tune obstacle-avoidance behavior tree for the competition course
