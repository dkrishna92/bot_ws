# Project Context

This file is read automatically by Claude Code when opened in this repo.
It captures the architecture decisions made while planning this project, so
work continues from where it left off instead of re-litigating settled
questions.

## Competition

Culture Club Robotics Competition (CAT Robotics), October 1-2, 2026, two
courses: "speed" (3 laps) and "obstacle" (2 laps + obstacle handling).
Safety review due September 25, 2026. Path-following demo milestone is
already complete — current effort targets the full October competition.

Hard constraints from the rules doc:
- Robot: max 16"W x 24"L x 16"H, 25 lb, 50V max onboard
- Vision-based autonomous start trigger (manual trigger = 5s time penalty)
- Wireless e-stop: motor cutoff within 1s of signal loss — hard real-time,
  must live on a dedicated MCU (Teensy/STM32), NOT on the main compute stack
- Course-provided wired e-stop interface via RJ45 (pins 1-4 vs 5-8 continuity)
- Obstacle course: skip penalty = nominalObstacleTime*(numSkips+(MAX(numSkips-1,0)*numSkips)/2)

## Hardware

- Compute: Raspberry Pi 5, Ubuntu Server 24.04 LTS (arm64), native install —
  NOT Raspberry Pi OS, NOT Docker. Confirmed officially supported starting
  with 24.04.
- Lidar: RPLIDAR (exact model TBD — "RP32" reference unconfirmed against a
  real Slamtec model name; confirm before locking serial baudrate config)
- Camera: OAK-D S2 (Luxonis / DepthAI) — inference and stereo depth run
  on-device; host only receives detections + depth, never raw frames
- Motor driver: Pololu Dual G2 High-Power Motor Driver 18v18 — PWM + DIR +
  SLEEP digital I/O, NO serial/I2C interface (this was a corrected mistake
  early on — don't reintroduce a serial-protocol assumption for this board)
- Ultrasonic: HC-SR04-class sensors, GPIO trigger/echo (bit-banged; consider
  moving to the safety MCU if timing noise becomes a problem)

## Package layout

Split (2026-08-12) into one ROS 2 package per node, rather than bundling
everything into `bot_bringup`:

- `bot_bringup` — launch files and config only, no node implementations.
  Composes the packages below plus rplidar_ros, depthai_ros_driver,
  nav2_bringup, and robot_localization (each optional/skip-if-not-installed).
  Two entry points: `mapping.launch.py` (build a course map, see below) and
  `bringup.launch.py` (the race launch, localizes against that map).
- `bot_motor` — motor_node (Pololu G2 driver)
- `bot_ultrasonic` — ultrasonic_node (HC-SR04 array)
- `bot_safety` — watchdog_node (secondary, defense-in-depth watchdog — see
  Safety architecture below; NOT the primary MCU e-stop)
- `bot_perception` — start_trigger_node (vision-based start signal)
- `bot_gazebo` — the `bot` URDF/xacro, gz-sim launch/bridge config, and the
  CFR speed/obstacle course world files (see GAZEBO_WORLDS.md)
- `bot_explore` — frontier_explore_node: drives autonomous SLAM mapping for
  `bot_bringup/launch/mapping.launch.py` (no teleop needed)

Reasoning: each node has different hardware/library dependencies (pigpio,
RPi.GPIO, pyserial, cv_bridge/OpenCV) that don't belong on a single
package's manifest. Keep new nodes in their own package unless there's a
specific reason to co-locate.

## Software stack decision

**ROS 2 Jazzy on Ubuntu 24.04, native apt install, no Docker.** This was a
reversal from an earlier bare-Python/ZeroMQ custom middleware design (built
and shipped for the path-following demo). For the October competition,
ROS 2 + Nav2 wins because the obstacle course needs real local planning
(costmaps, obstacle-aware planning, retry/skip state logic) that Nav2
already provides — not worth hand-rolling for a wheeled robot at ~10-50Hz
control rates (no kHz-loop justification for a custom middleware here).

The key architectural separation is this: middleware concerns are isolated to
ROS 2 transport, topics, services, and launch composition, while robot logic
remains in the per-node packages (`bot_motor`, `bot_ultrasonic`, `bot_safety`,
`bot_perception`). The custom middleware layer is intentionally not part of the
runtime behavior or the robot application logic.

- Middleware: default RMW (Fast DDS or Cyclone DDS), NOT Iceoryx.
  `rmw_iceoryx` has an open, unresolved Jazzy-support issue upstream and
  isn't apt-installable — not worth the risk given nothing in this stack
  ships large payloads over the bus (camera does on-device inference).
- Navigation: Nav2 (costmap layers, DWB or MPPI local planner, small
  behavior tree for obstacle attempt/skip/retry logic per rule 3.7)
- Localization: `robot_localization` EKF node (not hand-rolled)
- Drivers: `depthai-ros` for the OAK-D, an existing RPLIDAR ROS 2 driver
  package; ultrasonic and motor nodes are custom (no off-the-shelf package
  fits the Pololu G2's PWM/DIR interface or a generic HC-SR04 array)

## Safety architecture (unchanged regardless of software stack above)

Two independent layers:
1. **Primary, hard real-time:** dedicated MCU (Teensy/STM32), electrically
   isolated from the Pi/ROS 2 stack, wired directly to the motor driver's
   SLEEP/enable line. This is the actual <1s guarantee — Linux/ROS 2 cannot
   provide it.
2. **Secondary, defense-in-depth:** a ROS 2 watchdog node monitoring
   heartbeats from all nodes, publishing a fault topic that the motor node
   and Nav2 both respect. Never treat this as the primary mechanism.

## Development workflow

Laptop-first, Pi 5 for final integration:
1. Laptop (Ubuntu 24.04 + ROS 2 Jazzy, dual-boot or VM): package structure,
   node logic, unit tests
2. Laptop + real sensors over USB: RPLIDAR and OAK-D S2 both work fine over
   USB on x86_64 — validate driver/perception nodes against real sensor data
   without needing the Pi
3. Laptop + Gazebo: tune Nav2 (costmap, planner, obstacle behavior) in
   simulation before touching hardware. Two-phase launch: run
   `mapping.launch.py` once per course/world (autonomous frontier
   exploration via `bot_explore` + slam_toolbox, saves a map — no teleop),
   then `bringup.launch.py` (amcl + map_server localize against that map,
   full Nav2 stack drives). See `bot_bringup/launch/mapping.launch.py`'s
   docstring for exact commands.
4. Pi 5: deploy full stack, swap in real GPIO-based motor/ultrasonic nodes
   (RPi.GPIO/pigpio only work on actual Pi hardware — mock or skip these
   nodes on the laptop)
5. Course/field testing on Pi 5 only

## Open items / things not yet resolved

- Exact RPLIDAR model and baudrate
- Course layout / obstacle geometry: official SharePoint resources still
  inaccessible, but a teammate (D Turner) imported CFR speed/obstacle course
  geometry into Gazebo world files (`bot_gazebo/worlds/*_cfr.world`) from
  CAD, close enough to drive Nav2 tuning in sim in the meantime
- Perception sensor spec confirmation (camera-only CV vs added depth/LiDAR
  for obstacle detection) — pending official course documentation
- E-stop MCU firmware not yet written
- Nav2 params: `robot_radius` now matches `bot.urdf.xacro`'s real footprint
  and the map-then-race launch wiring (mapping.launch.py / bringup.launch.py)
  is in place, but costmap inflation and controller gains are still
  untuned against real (non-CAD) course geometry
