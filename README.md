# bot_ws

ROS 2 Jazzy workspace for the DIY Robot Challenge vehicle. See `CLAUDE.md`
at the workspace root for the full architecture context and decision log.

## Working in VS Code / Claude Code

- Open this whole `bot_ws/` folder as your VS Code workspace root, not just
  `src/bot_bringup/` — `CLAUDE.md` lives at the root and Claude Code picks
  it up automatically as project context when the workspace root is opened.
- Install the [Claude Code extension](https://docs.claude.com) if you
  haven't already; it reads `CLAUDE.md` on its own, so you shouldn't need
  to re-explain the architecture when you start a session there.
- The [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
  VS Code extension is worth adding too for `.launch.py` / URDF syntax
  support and a build/debug integration, separate from Claude Code.

## Middleware separation

The project deliberately keeps middleware concerns separate from robot logic:

- The ROS 2 middleware layer is the default RMW (Fast DDS or Cyclone DDS), not a custom message bus or ZeroMQ stack.
- Driver and control code lives in per-concern packages such as `bot_motor`, `bot_ultrasonic`, `bot_safety`, and `bot_perception`.
- Hardware logic stays in those node implementations; launch files only compose the graph and do not contain robot behavior.
- The safety MCU and Linux/ROS stack remain intentionally separate, with the MCU handling the hard real-time e-stop cutoff and ROS nodes handling higher-level monitoring and planning.

This keeps the application code independent from the middleware transport and makes it easier to swap or update the ROS stack without rewriting the robot logic.

## Build

```bash
cd bot_ws
colcon build --symlink-install
source install/setup.bash
source /opt/ros/jazzy/setup.bash
```

`--symlink-install` means edits to Python files take effect without
rebuilding — only re-run `colcon build` when you add a new file or change
`setup.py`/`package.xml`.

## Run

### Hardware (Raspberry Pi 5)
```bash
ros2 launch bot_bringup bringup.launch.py
```

### Gazebo Simulation (Laptop/Development)
```bash
ros2 launch bot_bringup bringup.launch.py use_sim:=true
```

This launches the full stack with Gazebo simulation:
- Simulated robot in obstacle course world
- Lidar scans, camera images from Gazebo
- All custom nodes (motor, ultrasonic, watchdog, start trigger)
- Ready for Nav2 integration and autonomous testing

See [src/bot_gazebo/README.md](src/bot_gazebo/README.md) for detailed simulation setup, topics, and troubleshooting.

The launch file integrates RPLidar and OAK-D drivers, but gracefully skips them if not installed (useful for laptop development). `robot_localization` and the full Nav2 stack (amcl + map_server + navigation) are already wired into `bringup.launch.py` when those packages are installed — but Nav2 needs a map to localize against first. Build one with `ros2 launch bot_bringup mapping.launch.py use_sim:=true` (autonomous frontier exploration, no teleop needed — see that launch file's docstring), then `colcon build --packages-select bot_bringup` before running the race launch below.

**To enable real sensors**, install the driver packages:

```bash
sudo apt install ros-jazzy-rplidar-ros ros-jazzy-depthai-ros-driver
```

## Sensor Integration

### RPLidar

- **Driver package**: `rplidar_ros`
- **Topic**: `/scan` (sensor_msgs/LaserScan)
- **Usage**: Watchdog node monitors scan freshness; Nav2 uses for mapping and
  obstacle detection; sensor_fusion_node converts to 3D points
- **Config**: [config/rplidar_params.yaml](config/rplidar_params.yaml)
- **Serial port**: `/dev/ttyUSB0` (adjust in launch file if different)

### OAK-D S2 (Luxonis DepthAI)

- **Driver package**: `depthai_ros_driver`
- **Topics**:
  - `/oak/rgb/image_raw` (sensor_msgs/Image) — RGB frames
  - `/oak/stereo/depth` (sensor_msgs/Image) — depth map (uint16, mm)
  - Spatial detection output (varies by detection model)
- **Usage**: start_trigger_node detects vision-based race start; sensor_fusion_node
  converts depth to 3D points for obstacle representation
- **Config**: [config/oak_params.yaml](config/oak_params.yaml)

### Sensor Fusion

The `sensor_fusion_node` combines RPLidar (2D planar scans) and OAK-D (RGB/depth)
into a unified 3D obstacle pointcloud (`/perception/obstacles`). This feeds Nav2's
costmap for richer obstacle awareness than lidar alone.

## Dev machine vs. Pi 5

`motor_node.py` and `ultrasonic_node.py` import `pigpio` / `RPi.GPIO`
inside a try/except and fall back to a dry-run mode (logs a warning, does
nothing) when those libraries aren't present — so the full launch file
runs fine on a laptop for testing everything except actual hardware I/O.
See CLAUDE.md's "Development workflow" section for the full laptop → Pi 5
pipeline (including using Gazebo for Nav2 tuning before touching hardware).

## Navigation (Nav2)

Nav2 is integrated for autonomous path planning and obstacle-aware navigation:

- **Planner**: `NavFn` (A*) -- this robot is skid-steer diff-drive (can rotate
  in place), so it doesn't need SmacPlannerHybrid's non-holonomic
  Reeds-Shepp motion model
- **Local controller**: DWB (Dynamic Window Approach) with velocity limiting
- **Costmap layers**:
  - Lidar obstacle layer from `/scan`
  - Range sensor layer from ultrasonic sensors
  - Inflation layer with tunable radius (currently 0.3m)
- **Localization**: `robot_localization` EKF fuses odometry sources
- **Recovery behaviors**: spin, back_up, wait

Configuration files:

- [config/nav2_params.yaml](config/nav2_params.yaml) — planner, controller, costmap params (placeholders, needs tuning)
- [config/ekf_params.yaml](config/ekf_params.yaml) — odometry fusion settings
- Behavior tree: `navigate_w_replanning_and_recovery.xml` (from nav2_bringup)

**To install Nav2:**

```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-robot-localization
```

Both packages are optional—launch proceeds without them if not installed (useful for laptop development).

## Status

Full stack is now integrated and ready for tuning, but:

- Nav2 costmap inflation / controller gains are still placeholders, not tuned against actual course geometry (see `config/nav2_params.yaml`); `robot_radius` has been corrected to match the real robot footprint
- EKF odometry fusion params are placeholder — tune against actual wheel slip and sensor noise
- OAK-D camera intrinsics in sensor_fusion_node are placeholder — replace with actual calibration
- Map server is wired (amcl + map_server in `bringup.launch.py`), but it needs a map built first via `mapping.launch.py` (autonomous SLAM exploration, see above) — nothing is checked into `config/maps/` yet
- E-stop MCU firmware isn't part of this repo at all yet
