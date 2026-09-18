# Gazebo Simulation

Gazebo Sim (gz-sim, the Harmonic release bundled with ROS 2 Jazzy) setup for
testing the DIY Robot Challenge vehicle without hardware. This is **not**
classic Gazebo — commands and plugin names below are gz-sim/`ros_gz`
specific.

## Quick Start

### 1. Race launch (full stack)
```bash
cd /home/krishna/claudebot/bot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bot_bringup bringup.launch.py use_sim:=true
```

This will:
- Start Gazebo Sim (server + GUI)
- Spawn `bot` (from `urdf/bot.urdf.xacro`) at (-0.7, 0, 0), the course start pose
- Load `obstacle_course_cfr.world` (default — pass `world:=<name>` to pick
  another course, see `GAZEBO_WORLDS.md` for the full list)
- Start all custom nodes (motor, ultrasonic, watchdog, start trigger) and,
  if installed, `robot_localization` + the full Nav2 stack (requires a map
  built by `mapping.launch.py` first — see the root `README.md`)
- Publish lidar scans and camera images from simulation

### 2. Mapping launch (autonomous SLAM, no teleop)
```bash
ros2 launch bot_bringup mapping.launch.py use_sim:=true
```
Drives itself via frontier exploration and saves the map when done. See
`bot_bringup/launch/mapping.launch.py`'s docstring for the exact save path
and rebuild step.

### 3. World only (no robot stack)
```bash
ros2 launch bot_gazebo obstacle_course_cfr.launch.py
# or:
ros2 launch bot_gazebo gazebo_sim.launch.py world:=speed_course_cfr
```

### Headless mode
The `headless` launch argument exists on `gazebo_sim.launch.py` but is
currently **not wired up** — passing `headless:=true` has no effect on the
`gz_args` passed to `gz sim`. If you need headless (e.g. CI), run
`gz sim -s <world>.world` directly, or fix the launch file to pass `-s` to
`gz_args` when `headless` is true.

## Features

### Robot Model
- **Differential drive**: two independent wheels, 0.35m track width, 0.1m wheel radius
- **Chassis**: 0.61m L × 0.41m W × 0.15m H box (within the 24"×16"×16" competition envelope — the 0.15m is chassis height only, not overall robot height with lidar/camera mounts)
- **Mass**: 11.3 kg (25 lb competition limit)
- Spawned via `ros_gz_sim create` reading the `robot_description` topic (published by `robot_state_publisher` from `urdf/bot.urdf.xacro` through `xacro`) — not a hardcoded model in the world file

### Sensors (Simulated)
1. **Lidar (stand-in for RPLIDAR)**: `gpu_lidar` sensor, 360 samples, range 0.3–12m, resolution 0.01m, topic `/scan`
2. **Camera (stand-in for OAK-D S2)**: `camera` sensor, RGB 640×480@30Hz, 60° horizontal FoV, topic `/camera/image_raw` only — `/camera/camera_info` is **not** currently bridged, so anything expecting `sensor_msgs/CameraInfo` on the ROS side won't get it yet
3. **Ultrasonic**: **not simulated at all** right now — `bot_ultrasonic`'s `ultrasonic_node` only does anything against real GPIO hardware on the Pi; there's no gz-sim equivalent sensor or mock

### World
Multiple course worlds are available (`obstacle_course_cfr`, `speed_course_cfr`, plus legacy/test worlds) — see `GAZEBO_WORLDS.md` at the workspace root for the full list, launch commands, and directory layout. Don't duplicate course-geometry details here; that file is the source of truth.

## ROS 2 Topics

### Bridged Topics (gz-sim ⇄ ROS, via `ros_gz_bridge`)
- `/cmd_vel` — `geometry_msgs/Twist`, subscribed by the `gz-sim-diff-drive-system` plugin on `bot`
- `/odom` — `nav_msgs/Odometry`, published by the same plugin
- `/scan` — `sensor_msgs/LaserScan` from the simulated lidar
- `/camera/image_raw` — `sensor_msgs/Image` from the simulated camera

That's the complete bridge list in `gazebo_sim.launch.py` — nothing else (no `/tf`, no `/camera/camera_info`) is currently bridged between gz-transport and ROS.

### TF Frames
- `map` → `odom`: static identity, from a `tf2_ros static_transform_publisher` node — always present when `gazebo_sim.launch.py` runs
- `odom` → `base_link`: only published when `robot_localization`'s EKF is running (i.e. via `bringup.launch.py` or `mapping.launch.py`, not `gazebo_sim.launch.py` alone). Even then, note `bot_bringup/config/ekf_params.yaml`'s `odom0` input topic is currently unset (`odom0: ""`), so the EKF isn't actually fusing wheel odometry yet — a pre-existing gap, not something this doc can paper over.
- `base_link` → `lidar_link`, `camera_link`: static, from `robot_state_publisher` (fixed joints in the URDF) — always present

## Test Commands

### Monitor robot movement
```bash
# Terminal 1
ros2 launch bot_bringup bringup.launch.py use_sim:=true

# Terminal 2
ros2 topic echo /odom
```

### Manually drive the robot (teleop)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

### Inspect Gazebo state
```bash
# List gz-transport topics (separate from ROS topics -- these are gz-sim's own)
gz topic -l

# ROS-side equivalent
ros2 topic list
ros2 topic echo /odom
```

### Check TF tree
```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf in current directory
```

## Tuning Physics

Edit the relevant world file (e.g. `worlds/obstacle_course_cfr.world`) to adjust:
- Solver parameters (`<max_step_size>`, `<real_time_factor>` under `<physics>`)
- Friction coefficients (`<mu>`/`<mu2>` under each collision's `<surface><friction><ode>`)
- Gravity
- Obstacle positions/sizes

## Known Limitations

1. **No true depth sensor**: camera publishes RGB only, no depth map. For stereo depth, add a `depth_camera` sensor type to `bot.urdf.xacro`'s `<gazebo reference="camera_link">` block — gz-sim's Sensors system handles it natively, no extra plugin filename needed (unlike classic Gazebo).
2. **`/camera/camera_info` not bridged**: add it to the `ros_gz_bridge` arguments in `gazebo_sim.launch.py` if you need calibration info on the ROS side.
3. **No ultrasonic simulation**: real hardware only, via `bot_ultrasonic`.
4. **`odom → base_link` TF depends on which launch file you use**, and even under the EKF it's not really fusing anything yet (see TF Frames above).
5. **`headless:=true` is a no-op** (see Quick Start above).
6. **Simplified wheel friction**: ODE approximation; real robot traction may differ.
7. **No motor current/torque feedback**: the diff-drive plugin always achieves commanded velocity instantly; no acceleration/deceleration limits.
8. **No sensor noise**: camera and lidar produce perfect data; add `<noise>` blocks to the sensor definitions in `bot.urdf.xacro` for realism.

## Troubleshooting

**Gazebo doesn't launch:**
```bash
gz sim --version
# Install if missing:
sudo apt install ros-jazzy-ros-gz
```

**Robot doesn't respond to `/cmd_vel`:**
- Check something is actually publishing: `ros2 topic info /cmd_vel` — nothing
  does by default outside of teleop or a running Nav2 goal
- Check the bridge is up: `ros2 topic list | grep cmd_vel` should show `/cmd_vel`
- Confirm the `gz-sim-diff-drive-system` plugin loaded without errors in the
  Gazebo console output (look for `[Err]` lines mentioning `DiffDrive`)

**Lidar/camera not publishing:**
```bash
ros2 topic list | grep -E 'scan|camera'
# If empty, check the Gazebo console for plugin load errors, and confirm the
# world file has the gz-sim-sensors-system plugin loaded (required for any
# rendering-based sensor -- camera, gpu_lidar -- to produce data at all)
```

**Simulation runs in slow-motion:**
- Reduce world complexity (fewer obstacle models)
- Increase `<max_step_size>` in the world file (less accurate but faster)
- Decrease `<real_time_factor>` to intentionally run slower than real-time

## Next Steps

- Wire `odom0` in `bot_bringup/config/ekf_params.yaml` to the bridged `/odom` topic so the EKF actually fuses wheel odometry
- Bridge `/camera/camera_info`
- Add a depth-camera sensor for real stereo-depth testing (stand-in for the OAK-D's on-device depth)
- Add ultrasonic simulation, or accept it stays real-hardware-only until field testing
- Tune Nav2 costmap/controller params against the CFR course geometry (see root `CLAUDE.md`'s Open Items)
