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
- Driver and control code lives in per-concern packages such as `bot_motor`, `bot_odometry`, `bot_safety`, `bot_imu`, and `bot_perception`.
- Hardware logic stays in those node implementations; launch files only compose the graph and do not contain robot behavior.
- The safety MCU and Linux/ROS stack remain intentionally separate, with the MCU handling the hard real-time e-stop cutoff and ROS nodes handling higher-level monitoring and planning.

This keeps the application code independent from the middleware transport and makes it easier to swap or update the ROS stack without rewriting the robot logic.

## Build

```bash
cd bot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means edits to Python files take effect without
rebuilding — only re-run `colcon build` when you add a new file or change
`setup.py`/`package.xml`.

## Raspberry Pi 5 setup (race robot)

One-time, on a fresh Ubuntu 24.04 Pi. Installs ROS 2 Jazzy plus every
driver/Nav2/SLAM package the robot needs (no Gazebo/RViz -- the Pi is
headless), fetches and builds the lidar driver, sets up device
permissions and stable device names, and builds the workspace:

```bash
cd ~/bot_ws
sudo ./scripts/setup_pi.sh
```

Then **log out and back in** (the script adds you to the `dialout` group,
which serial/I2C/GPIO access needs). If the script says it added the I2C3
overlay, **reboot** instead -- the IMU's bus doesn't exist until then.

Before every run -- and on race morning -- run the preflight check:

```bash
./scripts/check_hardware.sh            # devices present + accessible, workspace built, map present
./scripts/check_hardware.sh --topics   # also checks data flow; run while bringup is running
```

Expected devices:

| Device | Connection | Shows up as |
| --- | --- | --- |
| RPLIDAR S2 | USB (CP210x) | `/dev/rplidar` |
| Teensy (encoders + ultrasonics) | USB | `/dev/teensy` |
| OAK-D S2 | USB 3 | `lsusb` ID `03e7:` |
| BNO055 IMU | I2C bus 3: SDA pin 15 (GPIO22), SCL pin 16 (GPIO23), Vin 5V, GND | address `0x28` (`i2cdetect -y 3`) |
| Pololu G2 motor driver | GPIO 12/13 (PWM), 5/16 (DIR), 6/19 (SLEEP) | `gpiochip4` |

Troubleshooting:

- **Teensy missing from `lsusb`** -- usually a charge-only USB cable. A
  running Teensy shows as `16c0:0483`; `16c0:0478` means it's in the
  bootloader (needs flashing from `teensy_ws`).
- **"GPIO busy"** from motor_node/watchdog_node, or **"lidar scan stale"**
  with the lidar plugged in -- an earlier launch crashed and left orphaned
  nodes holding the pins / lidar port. Run `./scripts/clean_robot.sh`: it
  stops this workspace's nodes and anything holding a sensor (lidar/Teensy
  serial, IMU I2C bus, motor GPIO, OAK-D USB), drives the motor pins low,
  then runs `clean_sim.sh` for the general ROS cleanup. Run it before every
  launch on the robot (`clean_sim.sh` alone is for the laptop/sim).
- **IMU not on bus 3** -- check `/dev/i2c-3` exists (needs
  `dtoverlay=i2c3-pi5,pins_22_23` in `/boot/firmware/config.txt` + reboot;
  `setup_pi.sh` adds it). Don't use I2C1 (header pins 3/5): on this Pi it
  times out ("controller timed out" in `/var/log/kern.log`) even with
  nothing connected. The launch's `imu_i2c_bus` argument selects the bus.
- **Lidar "operation time out"** -- wrong baud or wrong driver; see
  [RPLidar](#rplidar) below.

## Run

### Hardware (Raspberry Pi 5)

Wheels off the ground and wireless e-stop in hand for the first run.
`rviz:=false` because the Pi has no display:

```bash
ros2 launch bot_bringup bringup.launch.py rviz:=false
```

To build a course map on the real robot first (see `mapping.launch.py`'s
docstring for teleop mode and saving):

```bash
ros2 launch bot_bringup mapping.launch.py rviz:=false
```

Both launches pull in `bot_bringup/launch/hardware.launch.py` (robot model
TF, lidar, OAK-D, motor, watchdog, wheel odometry, IMU) whenever
`use_sim` is false.

### Gazebo Simulation (Laptop/Development)

```bash
ros2 launch bot_bringup bringup.launch.py use_sim:=true
```

For mapping Run

```bash
ros2 launch bot_bringup mapping.launch.py use_sim:=true
```

This launches the full stack with Gazebo simulation:
- Simulated robot in the CFR course world
- Lidar scans, depth, IMU, camera images and `/odom` from Gazebo
- Start trigger, lap navigator, EKF and Nav2
- **Not** the Pi-only hardware nodes (motor, watchdog, wheel odometry,
  IMU driver) -- Gazebo's DiffDrive plugin stands in for them

See [src/bot_gazebo/README.md](src/bot_gazebo/README.md) for detailed simulation setup, topics, and troubleshooting.

On hardware, the lidar and OAK-D drivers are skipped (with a log line) if not installed. `robot_localization` and the full Nav2 stack (amcl + map_server + navigation) are already wired into `bringup.launch.py` when those packages are installed — but Nav2 needs a map to localize against first. Build one with `ros2 launch bot_bringup mapping.launch.py use_sim:=true` (autonomous frontier exploration, no teleop needed — see that launch file's docstring), then `colcon build --packages-select bot_bringup` before running the race launch below.

**To enable real sensors** on the Pi, run `scripts/setup_pi.sh` (see
[Raspberry Pi 5 setup](#raspberry-pi-5-setup-race-robot)) -- the lidar
driver isn't an apt package, so a plain `apt install` isn't enough.

## Sensor Integration

### RPLidar

- **Model**: RPLIDAR S2 -- 1,000,000 baud, DenseBoost mode (32 kHz, 10 Hz, 30 m)
- **Driver package**: Slamtec's `sllidar_ros2`, built from source into
  `src/sllidar_ros2` (gitignored; `setup_pi.sh` clones it at a pinned
  commit). **Not** the apt `ros-jazzy-rplidar-ros` -- its old SDK
  segfaults on this lidar.
- **Topic**: `/scan` (sensor_msgs/LaserScan), frame `lidar_link`
- **Usage**: Watchdog node monitors scan freshness (motors are held stopped
  until scans arrive); Nav2 uses it for mapping and obstacle detection
- **Config**: [src/bot_bringup/config/rplidar_params.yaml](src/bot_bringup/config/rplidar_params.yaml)
- **Serial port**: `/dev/rplidar` (udev symlink); override with
  `lidar_port:=/dev/ttyUSB0` if the udev rule isn't installed

### OAK-D S2 (Luxonis DepthAI)

- **Driver package**: `depthai_ros_driver`
- **Topics**:
  - `/oak/rgb/image_raw` (sensor_msgs/Image) — RGB frames, used only by
    start_trigger_node
  - `/oak/stereo/image_raw` (sensor_msgs/Image) — depth image
  - `/oak/points` (sensor_msgs/PointCloud2) — XYZ point cloud made from the
    depth image by `depth_image_proc` (no RGB involved)
- **Usage**: start_trigger_node watches the RGB feed for the start signal
  flipping red->green (see "Vision-based start signal" below); `/oak/points`
  feeds the Nav2 costmaps' voxel layer for obstacles the planar lidar can't
  see
- **Config**: [src/bot_bringup/config/oak_params.yaml](src/bot_bringup/config/oak_params.yaml)
- **TF**: the driver's `oak-*` frames hang off the URDF's `camera_link`

## Dev machine vs. Pi 5

`motor_node` and `watchdog_node` use `lgpio` (pigpio and RPi.GPIO don't
work on the Pi 5) inside a try/except, and fall back to a dry-run mode
(logs a warning, no hardware output) when it isn't installed -- so the
node code runs fine on a laptop. Both also take a `dry_run:=true`
parameter to force that mode on the Pi. Ultrasonics are read by the
Teensy and published by `bot_odometry`'s `wheel_odom_node`; there is no
separate ultrasonic node.
See CLAUDE.md's "Development workflow" section for the full laptop → Pi 5
pipeline (including using Gazebo for Nav2 tuning before touching hardware).

## Navigation (Nav2)

Nav2 is integrated for autonomous path planning and obstacle-aware navigation:

- **Planner**: `NavFn` -- this robot is skid-steer diff-drive (can rotate
  in place), so it doesn't need SmacPlannerHybrid's non-holonomic
  Reeds-Shepp motion model
- **Local controller**: DWB, capped at 0.5 m/s and 1.0 rad/s
- **Costmap layers** (local and global):
  - Obstacle layer from the lidar's `/scan`
  - Voxel layer from the OAK-D's `/oak/points`
  - Static layer from the pre-built map (global costmap only)
  - Inflation layer, 0.35 m radius (`robot_radius` 0.26 m)
  - The ultrasonics publish `sensor_msgs/Range` but are **not** a costmap
    layer yet
- **Localization**: amcl against the pre-built map (`map` -> `odom`); the
  `robot_localization` EKF fuses wheel odometry (`/odom`, velocity) and
  the IMU (`/imu`, yaw/yaw-rate) into `odom` -> `base_link`
- **Recovery behaviors**: spin, back up, drive on heading, wait

Configuration files:

- [src/bot_bringup/config/nav2_params.yaml](src/bot_bringup/config/nav2_params.yaml) — planner, controller, costmap params
- [src/bot_bringup/config/ekf_params.yaml](src/bot_bringup/config/ekf_params.yaml) — odometry fusion settings
- [src/bot_bringup/config/maps/map.yaml](src/bot_bringup/config/maps/map.yaml) — the map the race launch localizes against
- Behavior trees: nav2_bringup's defaults

On the Pi, `scripts/setup_pi.sh` installs Nav2 and robot_localization. The
launch skips each one if it isn't installed (useful for laptop
development).

### Comparing planner/controller alternatives (development tool)

An in-progress A/B comparison against the current NavFn/DWB config is
underway — see `CLAUDE.md`'s "Nav2 planner/controller comparison" section
for what's being compared and why. This is a development tool for tuning,
not part of the race-day or mapping workflow itself:

```bash
scripts/run_nav2_variant_test.sh race --all      # race/bringup variants
scripts/run_nav2_variant_test.sh mapping --all   # mapping/frontier-exploration variants
```

Or launch a single variant manually with
`ros2 launch bot_bringup bringup.launch.py use_sim:=true nav2_params_file:=<variant>.yaml`
(swap `bringup.launch.py` for `mapping.launch.py` and the `nav2_params_`
prefix for `nav2_mapping_params_` for the mapping-side variants).

## Vision-based start signal and race route

The competition requires an autonomous, vision-triggered race start (a
manual trigger costs a 5s time penalty). Two nodes split the job:

1. `bot_perception`'s `start_trigger_node` watches the RGB feed for the
   course's start signal flipping from red to green (debounced over a few
   consecutive frames to reject a flicker/false read), then publishes a
   latched `start_signal` (std_msgs/Bool). It does not drive anything.
   - **Real hardware**: subscribes to `/oak/rgb/image_raw` (the node's
     default `image_topic`).
   - **Sim**: subscribes to `/camera/image_raw` (`bot_gazebo`'s simulated
     camera) — `bringup.launch.py` picks the right topic from `use_sim`.
2. `bot_navigation`'s `lap_navigator_node` waits for `start_signal`, then
   sends Nav2 one `NavigateThroughPoses` goal covering the whole race: the
   chosen course's checkpoints, repeated for the lap count.

Pick the course (and optionally the lap count) at launch:

```bash
ros2 launch bot_bringup bringup.launch.py rviz:=false course:=speed_course_cfr     # 3 laps by default
ros2 launch bot_bringup bringup.launch.py rviz:=false course:=obstacle_course_cfr  # 2 laps by default
ros2 launch bot_bringup bringup.launch.py rviz:=false course:=speed_course_cfr num_laps:=1
```

**Not race-ready yet:** the checkpoints in `lap_navigator_node.py` come
from the sim world files, not the real course, and are validated only
against the sim-built map. The speed-course "lap" is currently an
**out-and-back shuttle in one lane, not a closed lap**: that map never
covered the far side of the oval, and its two top lanes don't connect for
the planner. Before racing, map the real course with
`mapping.launch.py` and re-derive the checkpoints against that map (see the
node's docstring).

### Simulating the start signal in Gazebo

`speed_course_cfr`/`obstacle_course_cfr` include a physical start-signal
model (`start_signal_arms`) — a red/green paddle on a revolute joint,
commanded over `/start_signal/arm`. `scripts/start_signal.py` drives it,
simulating a race official flipping the signal, so the full
detect → trigger → drive path can be tested end to end against an
already-launched `bringup.launch.py use_sim:=true`:

```bash
python3 scripts/start_signal.py                     # red->green after a fixed 5s delay
python3 scripts/start_signal.py --delay 8            # fixed delay, custom
python3 scripts/start_signal.py --randomize          # random 3-8s delay (unpredictable, like a real official)
python3 scripts/start_signal.py --randomize --min-delay 2 --max-delay 10
```

## Status (2026-09-25)

Verified on the race Pi: ROS 2 Jazzy installed; the race launch comes up;
lidar `/scan` (10 Hz) and OAK-D `/oak/points` publish; motor_node owns
its GPIO; watchdog holds the motors stopped until the lidar is live.

Still open before race day:

- **Teensy not detected over USB** and **BNO055 not answering on I2C** —
  without them there is no `/odom` or `/imu`, so the EKF can't publish
  `odom` -> `base_link` and Nav2 can't drive. See the troubleshooting list
  under [Raspberry Pi 5 setup](#raspberry-pi-5-setup-race-robot).
- **Nothing has driven on real hardware yet.** First motion test: wheels
  off the ground, wireless e-stop in hand.
- **Race map and checkpoints are sim-derived** (see "Vision-based start
  signal and race route") — the speed course is an out-and-back shuttle,
  not a lap. Map the real course and re-derive the checkpoints.
- **Bridge/helix ramps** will show up as walls to the real lidar and depth
  camera — the fix so far is sim-only (see CLAUDE.md).
- Nav2 controller gains and EKF noise params are tuned only in sim, if at
  all.
- E-stop (ESP32), Teensy, and kill-switch transmitter firmware
  (`arduino_ws`, `teensy_ws`, `feather_ws`) aren't in this repo or on the
  Pi, and are untested on real hardware.
