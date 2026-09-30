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

Course surface (confirmed 2026-10-01): asphalt road throughout; the
obstacle course also has potholes and sand sections; loose hay may lie on
the road. Potholes are negative obstacles -- invisible to the 2D lidar and
to the OAK voxel layer (it only marks points above the road) -- so race
routes must be recorded to steer around them. The OAK voxel layers'
min_obstacle_height is 0.06 m (was 0.02) so loose hay/depth noise on the
road doesn't mark phantom obstacles. Pivot breakaway must be measured on
asphalt (it was 40% duty on tile).

Hard constraints from the rules doc:
- Robot: max 16"W x 24"L x 16"H, 25 lb, 50V max onboard
- Vision-based autonomous start trigger (manual trigger = 5s time penalty)
- Wireless e-stop: motor cutoff within 1s of signal loss — hard real-time,
  must live on a dedicated MCU, NOT on the main compute stack (implemented
  as a separate Arduino Nano driving a relay that breaks the motor
  driver's power supply directly — see Hardware and Safety architecture
  below)
- Course-provided wired e-stop interface via RJ45 (pins 1-4 vs 5-8 continuity)
- Obstacle course: skip penalty = nominalObstacleTime*(numSkips+(MAX(numSkips-1,0)*numSkips)/2)

## Hardware

- Compute: Raspberry Pi 5, Ubuntu Server 24.04 LTS (arm64), native install —
  NOT Raspberry Pi OS, NOT Docker. Confirmed officially supported starting
  with 24.04. The race Pi had a GNOME desktop installed; it now boots
  headless (`multi-user.target`, set 2026-09-28, also in `setup_pi.sh`) to
  leave CPU/RAM for the Nav2 stack.
- Lidar: RPLIDAR S2 (identified 2026-09-25 on the Pi: answers only at
  1 Mbaud, DenseBoost mode = 32 kHz sample rate / 10 Hz / 30 m — the old
  "RP32" reference was the 32K sample rate). Driver is Slamtec's
  `sllidar_ros2` built from source into `src/` (gitignored, fetched at a
  pinned commit by `scripts/setup_pi.sh`) — the apt `rplidar_ros` 2.1.0
  (SDK 1.12) segfaults on scan start with this unit in every scan mode.
- Camera: OAK-D S2 (Luxonis / DepthAI) — inference and stereo depth run
  on-device; host only receives detections + depth, never raw frames.
  Mounted below the lidar: housing bottom measured 9 cm above ground
  (2026-09-25), so `camera_link` (housing centre) is at z = 0.104 m in
  `bot.urdf.xacro`; its x offset is still the original estimate. The
  URDF's lidar height (0.1143 m, a "planned" figure) is likely too low —
  the real lidar's beam clears the camera top (~11.8 cm) with no
  self-returns under 0.32 m — measure and update it.
- Motor driver: Pololu Dual G2 High-Power Motor Driver 18v18 — PWM + DIR +
  SLEEP digital I/O, NO serial/I2C interface (this was a corrected mistake
  early on — don't reintroduce a serial-protocol assumption for this board).
  It's the **Raspberry Pi HAT version, so its pins are fixed by the board**
  (BCM GPIO): PWM 12/13, DIR 24/25, SLP 22/23, FLT 5/6 (driver's
  open-drain fault output, read with a Pi pull-up; low = fault); motor 1 =
  left, motor 2 = right. Confirmed 2026-09-27. Before that, `motor_node`
  used a guessed map (DIR 5/16, SLEEP 6/19): it drove the FLT pins as
  outputs, never drove the real DIR/SLP pins, and the IMU's I2C bus sat on
  the SLP pins -- so direction never changed and the driver never slept.
  `scripts/motor_test.py` checks each motor/direction against the
  encoders. The motors are mounted mirror-image, so the right channel was
  originally inverted in software (`motor_node`'s `channel_b_inverted`,
  same in the test script) so a positive command drives both wheels
  forward. **New motors installed 2026-09-28, both sides needed
  re-calibrating, and the process caught a real gap in
  `scripts/motor_test.py`'s methodology**:
  - Left: `scripts/motor_test.py` caught it running backwards relative to
    command (right unaffected at the time) -- its leads landed reversed
    compared to the old motor. `channel_a_inverted` is now `true`.
  - Right: `scripts/motor_test.py` reported "OK", but a direct visual
    check of the wheel caught it spinning backwards relative to command
    anyway. That script only checks *self-consistency* between commanded
    direction and encoder count -- it can't distinguish "genuinely
    correct" from "motor direction wrong AND encoder sign wrong,
    cancelling out," which is exactly what had happened. Fixed with two
    independent corrections: `channel_b_inverted` flipped back to `false`
    (true motor direction), and the right encoder's sign inverted in
    `teensy_ws` firmware itself (`RIGHT_ENCODER_SIGN`, not a Python-side
    patch, so `scripts/motor_test.py`'s own direct reading of the raw `E`
    line stays correct too). **Lesson: always visually confirm wheel
    direction after a motor or encoder change, not just this script's
    summary** -- re-run it (and watch the wheel) after any future
    motor/encoder rewiring rather than assuming these polarities hold.
- Ultrasonic: HC-SR04-class sensors, trigger/echo timing. Moved from the
  Pi's bit-banged RPi.GPIO (2026-09-20) onto the same Teensy that reads
  wheel encoders, reporting raw echo pulse widths over the same USB serial
  link as encoder ticks — see `bot_odometry`'s wheel_odom_node below,
  which now owns both. The old `bot_ultrasonic` package (Pi-side GPIO
  node) is retired. **Design changed 2026-09-27: two sensors (left/right),
  not three** — the earlier front-left/front-right/rear layout dropped the
  rear sensor. Trigger/echo pins confirmed on the Teensy: left Trig=2/
  Echo=3, right Trig=4/Echo=5 (`teensy_ws`'s `LEFT_TRIG_PIN`/etc).
- Wheel encoders: quadrature, one per side (matches the DiffDrive plugin's
  left_joint/right_joint grouping in `bot_gazebo`), read by a Teensy via the
  `Encoder` library and reported to the Pi over USB serial as tick counts —
  see `bot_odometry`'s wheel_odom_node below. This Teensy now handles
  encoder AND ultrasonic reporting (see Ultrasonic above) over its one USB
  serial link; it is NOT the e-stop MCU (that's a separate Arduino Nano,
  see Safety architecture). Encoder part number/CPR confirmed 2026-09-24:
  Pololu #4843 (20.4:1 25D 12V HP gearmotor, 48 CPR motor-shaft encoder =
  979.62 CPR at the gearbox output shaft). Teensy pins confirmed
  2026-09-27: mounted on the front-left/front-right wheels (labeled LF/RF)
  but electrically the "left"/"right" side encoders wheel_odom_node
  expects — left A=20/B=21, right A=22/B=23, meter-checked (`teensy_ws`'s
  `LEFT_ENC_A_PIN`/etc; an earlier note here had the sides and A/B
  swapped). Read via the analog workaround until the dividers are fixed —
  see Open items.
- Wheelbase (front-to-rear axle spacing) measured 0.216 m (2026-09-30; the
  URDF had a 0.33 estimate). Track 0.33 m (measured the same day; was 0.32). A short wheelbase relative to
  the track makes skid-steer pivots easier (less wheel scrub). Nav2's
  footprint uses the chassis length, not the wheelbase, so it's unaffected.
- Drive motors: Pololu #4843 (20.4:1 25D 12V HP gearmotor), one per wheel,
  paired 2-per-side onto the Pololu G2 driver's two channels (matches the
  DiffDrive plugin's per-side joint grouping above). Real spec (12V):
  500 RPM/300 mA no-load, 7.4 kg-cm (~0.73 N-m) stall torque @ 5.0 A
  extrapolated, 4 kg-cm (~0.39 N-m) recommended continuous limit.
  `bot.urdf.xacro`'s `max_wheel_torque` now matches this stall figure
  (previously a 20 N-m placeholder, ~27x too high) — in-place skid-steer
  rotation needs torque in roughly this same range just to overcome scrub
  friction, so real hardware may find pivot turns noticeably harder than
  the old, unrealistically generous sim behavior suggested.
- IMU: Bosch BNO055 — 9-DOF (accel + gyro + magnetometer) with onboard
  sensor fusion; outputs an absolute, magnetically-referenced orientation
  directly from the chip, not just raw gyro. Interface decided (2026-09-20):
  I2C, on **I2C3 routed to GPIO14/15 (header pins 8 SDA / 10 SCL)** as of
  2026-09-27. History: I2C1 (GPIO2/3, pins 3/5) logs "controller timed
  out" on the race Pi even with nothing attached, so the IMU moved to
  I2C3 on GPIO22/23 (2026-09-25) -- but GPIO22/23 turned out to be the
  Pololu G2 motor HAT's SLP pins, so it moved again to GPIO14/15 (free:
  the console is on tty1, no serial getty). Needs
  `dtoverlay=i2c3-pi5,pins_14_15` in `/boot/firmware/config.txt`
  (`setup_pi.sh` adds it and removes the old pins_22_23 line);
  `hardware.launch.py`'s `imu_i2c_bus` arg (default 3) selects the bus.
  Verified on I2C3 2026-09-25: chip ID 0xA0 at 0x28, `/imu` at 50 Hz,
  ~9.4 m/s² on +z at rest (re-verify after the rewire). Real driver node now exists: `bot_imu`'s bno055_node (NDOF
  fusion mode, smbus2). Mounted 2026-09-28: 11.5 cm back from the chassis
  front panel (not counting the OAK), on the centreline, 94 mm above
  ground, with its X axis facing the BACK of the robot (Y right, Z up).
  `bot.urdf.xacro`'s `imu_joint` encodes this as yaw = pi; the driver does
  no axis remapping, so that transform is the only correction. **Runs in
  IMUPLUS mode (gyro + accel, no magnetometer) since 2026-09-29**, not
  NDOF: the chip reported CALIB_STAT sys=0 mag=0 on the robot, its
  magnetometer-referenced heading re-snapped as the motors disturbed it,
  and a mapping run came out as two copies of the room rotated ~25-30 deg
  apart. Heading is now relative to power-on (EKF `imu0_relative: true`);
  no calibration step is needed (the gyro self-calibrates at rest;
  `bno055_node`'s `fusion_mode` param can switch back to `ndof`). Mount
  verified the same day: +9.59 m/s^2 on chip Z at rest (Z up), so the
  yaw-rate sign is unaffected by the yaw = pi mount. Fused into
  `robot_localization`'s EKF for yaw/yaw-rate only (see
  `bot_bringup/config/ekf_params.yaml`) — wheel odometry keeps
  position/linear velocity. Simulated in Gazebo via `bot.urdf.xacro`'s
  `imu_link` + the `gz-sim-imu-system` world plugin, with noise values
  approximated from the BNO055 datasheet (verify against the actual
  datasheet before hardware integration — see that file's comment;
  `bot_imu`'s real driver reuses the same approximated stddevs for its
  angular_velocity/linear_acceleration covariance, and a rough unverified
  placeholder for orientation covariance, which has no equivalent
  datasheet noise-density spec since it's the chip's fused output).
  Caution: BNO055 magnetometer-based heading (NDOF fusion mode) is
  commonly reported to degrade near motor current/magnetic fields — this
  robot's Pololu G2 driver and DC motors are exactly that kind of source,
  so mounting location matters and on-robot calibration should be
  verified before trusting its absolute yaw output.

## Package layout

Split (2026-08-12) into one ROS 2 package per node, rather than bundling
everything into `bot_bringup`:

- `bot_bringup` — launch files and config only, no node implementations.
  Composes the packages below plus rplidar_ros, depthai_ros_driver,
  nav2_bringup, and robot_localization (each optional/skip-if-not-installed).
  Two entry points: `mapping.launch.py` (build a course map, see below) and
  `bringup.launch.py` (the race launch, localizes against that map).
- `bot_motor` — motor_node (Pololu G2 driver)
- `bot_safety` — watchdog_node (secondary, defense-in-depth watchdog — see
  Safety architecture below; NOT the primary MCU e-stop). Its MCU-serial
  status mirror is currently commented out — it assumed e-stop status was
  readable over serial from a Teensy, which isn't the real architecture
  (see Safety architecture below); the rest of the watchdog (heartbeat/
  fault-topic logic) is unaffected.
- `bot_odometry` — wheel_odom_node: converts Teensy-reported quadrature
  encoder tick counts into wheel odometry (`nav_msgs/Odometry` on `odom`)
  for `robot_localization`'s EKF, AND (2026-09-20) Teensy-reported
  ultrasonic echo pulses into `sensor_msgs/Range` — this node is now the
  sole owner of the Teensy's one USB serial link, since a second ROS node
  can't safely read the same serial port independently (this retired the
  old `bot_ultrasonic` package's Pi-side bit-banged RPi.GPIO node — see
  "Ultrasonic moved to Teensy" below). Real hardware only — Gazebo sim gets
  equivalent `/odom` for free from `bot_gazebo`'s DiffDrive plugin, so this
  node is skipped in sim exactly like `bot_motor`.
- `bot_imu` — bno055_node: BNO055 driver over I2C (NDOF fusion mode),
  publishes `sensor_msgs/Imu` on `imu` for `robot_localization`'s EKF. Real
  hardware only, same skip-in-sim pattern as `bot_motor`/`bot_odometry`.
- `bot_perception` — start_trigger_node (vision-based start signal)
- `bot_gazebo` — the `bot` URDF/xacro, gz-sim launch/bridge config, and the
  CFR speed/obstacle course world files (see GAZEBO_WORLDS.md)
- `bot_explore` — frontier_explore_node: drives autonomous SLAM mapping for
  `bot_bringup/launch/mapping.launch.py` (no teleop needed)
- `bot_web_control` — web_control_node: browser dashboard (stdlib HTTP
  server, port 8080, no auth) that starts/stops bringup/mapping as
  `ros2 launch` subprocesses, teleops via `/cmd_vel` (0.5 s staleness
  zeroing), saves/resumes slam_toolbox checkpoints, and renders `/map`.
  Dev/ops convenience only — its Stop is not an e-stop. Stop runs
  `clean_robot.sh` (or `clean_sim.sh` in sim) when `workspace_root` is
  set, passing `--keep-dashboard` so the scripts don't kill the dashboard
  itself. Run by hand without that flag, both scripts stop the dashboard
  too (2026-09-27).
  See `src/bot_web_control/README.md`.

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
1. **Primary, hard real-time:** a dedicated Arduino Nano, electrically
   isolated from the Pi/ROS 2 stack, driving a relay that breaks the motor
   driver's power supply directly (not a logic-level enable/disable line).
   This is the actual <1s guarantee — Linux/ROS 2 cannot provide it. The
   Nano's serial RX is fed by a UART LoRa transceiver module (not the Pi);
   it treats a heartbeat message from that module as the deadman signal —
   an explicit e-stop message or the heartbeat simply stopping (wireless
   signal loss) both de-energize the relay and cut power, and it stays cut
   until a clean heartbeat resumes. The transmitting end is `feather_ws` —
   a Feather board reading a physical kill-switch input pin and reporting
   status continuously over its own paired LoRa module. See
   `arduino_ws/src/main.cpp` and `feather_ws/src/main.cpp` for the shared
   serial protocol and each side's logic. (Clarified 2026-09-27: the
   e-stop MCU is an Arduino Nano ESP32 -- "Nano" elsewhere in this doc
   means this board. The ESP32 has onboard Bluetooth/WiFi; any radio link
   to the Pi must stay status-only and never able to command the relay, or
   it breaks the isolation above. BLE was removed from `arduino_ws`
   entirely during bring-up -- see "E-stop wireless link: real-hardware
   bring-up and sequencing" below.) (Notes: an earlier version of this doc
   assumed this role would be a Teensy — corrected 2026-09-20. The Teensy
   is now dedicated to wheel encoder reporting instead, see Hardware above
   and `bot_odometry`. An earlier version of this doc and of
   `arduino_ws/src/main.cpp`'s comments also assumed the relay switched a
   logic-level motor-driver enable/disable line rather than the driver's
   power supply directly — corrected 2026-09-20.)
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
4. Pi 5: deploy full stack, swap in real GPIO-based motor node and the
   Teensy-serial-backed odometry/ultrasonic node (RPi.GPIO/pigpio and the
   Teensy's USB serial link only work on actual Pi hardware — mock or skip
   these nodes on the laptop)
5. Course/field testing on Pi 5 only

## Open items / things not yet resolved

- Course layout / obstacle geometry: official SharePoint resources still
  inaccessible, but a teammate (D Turner) imported CFR speed/obstacle course
  geometry into Gazebo world files (`bot_gazebo/worlds/*_cfr.world`) from
  CAD, close enough to drive Nav2 tuning in sim in the meantime
- Perception sensor spec confirmation (camera-only CV vs added depth/LiDAR
  for obstacle detection) — pending official course documentation
- Pi 5 CPU optimization (2026-09-29): the per-node Python hotspots have
  been vectorized with NumPy, verified byte/numerically identical to the
  loops they replaced — `bot_perception`'s sensor_fusion_node
  (`_lidar_to_3d`/`_depth_to_3d`, previously a ~76k-iteration nested
  per-pixel Python loop, plus the per-point `Point32` build),
  `bot_imu`'s bno055_node (one 32-byte I2C block read per tick over the
  contiguous accel/gyro/quaternion registers instead of ten 2-byte reads,
  ~10x fewer I2C transactions at 50 Hz), and `bot_web_control`'s
  web_control_node map render (`np.where`/`np.flipud` instead of two
  per-cell loops on every `/map` GET). `start_trigger_node` now destroys
  its image subscription on latch (the TRANSIENT_LOCAL publish still
  reaches late subscribers) so it stops decoding camera frames for the
  whole race. **Suggested next levers, both config not code, and both
  needing on-Pi measurement (`gz stats`/`top`/RTF) to pick values — NOT
  yet done:** (a) if racing on MPPI, benchmark and lower
  `MPPIController`'s `batch_size` (default 2000, single-threaded on the Pi
  — see the Nav2 planner/controller comparison section) until the
  controller holds its 20 Hz rate, or keep DWB on the Pi and reserve MPPI
  for laptop A/B tests; (b) decimate the OAK depth cloud feeding
  depth_image_proc / the `voxel_layer`, and/or trim `obstacle_max_range`
  (currently 3.0 in the nav2 params), to cut costmap update cost. Note:
  sensor_fusion_node is still not started by any launch file — if it's
  ever enabled, it's now Pi-ready, but confirm it's actually wanted first.
- E-stop wireless link (`arduino_ws` + `feather_ws`) is now tested and
  confirmed working on real hardware, 2026-09-26 — see "E-stop wireless
  link: real-hardware bring-up and sequencing" below for the full pin
  assignments, protocol sequencing, and bring-up findings. Not yet fully
  reliable: occasional dropped heartbeats point to a still-marginal
  physical connection on the feather's module that needs re-seating or
  soldering before this is race-ready.
- **E-stop Nano ESP32 hangs on a full battery power-up (found
  2026-09-29):** it comes up in ESP32 ROM download mode (USB 303a:1001,
  "waiting for download") instead of running its firmware, so no heartbeat
  is ever processed and the relay stays de-energized (fails safe, but the
  motors stay dead). A USB replug or its reset button boots it fine. Read
  over the ROM loader: `GPIO_STRAP_REG` = 0x00000000 (all strapping pins
  latched LOW, incl. GPIO0 which has a pull-up) and `FORCE_DOWNLOAD_BOOT`
  clear -- i.e. not a host-triggered reset but the chip leaving reset
  before its supply had ramped up on the slow battery power-up. The BLE
  removal (below) was likely a red herring for the earlier bring-up hang.
  Workaround: `scripts/estop_recover.py` clears it in software via the
  ROM loader + an RTC-watchdog reset; `check_hardware.sh` flags the state.
  Decided 2026-09-29 to leave the Nano on the Pi's USB permanently so this
  runs automatically: `setup_pi.sh` installs a udev rule that starts
  `estop-recover.service` whenever 303a:1001 appears (it waits 3 s, since
  every normal boot shows 303a:1001 for ~0.6 s), and keeps ModemManager
  off the Nano. Trade-off vs the isolation principle above: the Pi can now
  reset or reflash the e-stop MCU over USB. A reset only ever fails safe
  (relay de-energized until the firmware sees a clean heartbeat), but
  nothing on the Pi should open that port except the recovery script.
  Hardware fix still worth doing: delay the Nano's reset release
  (capacitor from its RST pin to GND, e.g. 1-10 uF) or feed it a
  faster-ramping supply.
- Wheel encoder Teensy pin assignment confirmed 2026-09-27 against the
  actual wiring with a meter: **left A/B = pins 20/21, right A/B = 22/23**
  (an earlier assignment had the sides reversed). Part number/CPR
  confirmed 2026-09-24 as Pololu #4843 (979.62 CPR at the gearbox output
  shaft); `bot_odometry`'s wheel_odom_node's `ticks_per_rev` matches. The
  `teensy41` build in `teensy_ws/platformio.ini` flashes and runs on the
  robot's Teensy (flashed from the Pi with PlatformIO in `~/.platformio`;
  needs PJRC's udev rule, which `setup_pi.sh` installs).
- **Encoder divider workaround (2026-09-27, temporary):** the A/B voltage
  dividers deliver only ~1.2 V (left) / ~1.37 V (right) at the Teensy
  pins, below the Teensy 4.1's ~2.3 V digital-high threshold, so no
  digital edge ever registers. `teensy_ws/src/main.cpp` sets
  `ENCODER_ANALOG_WORKAROUND 1`: a 40 kHz IntervalTimer reads the pins with
  the ADC (8-bit, 0.4/0.8 V hysteresis) and decodes quadrature in software
  (ISR ~13 us of each 25 us; `S,<isr_us>,<left_invalid>,<right_invalid>`
  status line once a second, ignored by wheel_odom_node). Verified by hand:
  each side counts positive going forward, zero invalid transitions, and
  `/odom` publishes. Only checked at hand speed; watch the invalid counts at
  full motor speed. Once the dividers are re-sized for ~3.0–3.3 V at the
  pin, set it back to 0 (Encoder library).
- **Teensy firmware mix-up (found and fixed 2026-09-29):** from 2026-09-28
  until 2026-09-29 the Teensy ran a build from the stale standalone
  `~/claudebot/teensy_ws` checkout on the laptop (old pin map, digital
  `Encoder` library on ~1.2 V signals, no `S` status lines) instead of
  this repo's `teensy_ws` (analog workaround). Odometry in that period was
  garbage (e.g. left +854k vs right -3.6k ticks over the same driving).
  **Only build/flash `bot_ws/teensy_ws`, from the Pi.** Reflashed with
  `LEFT_ENCODER_SIGN` +1 / `RIGHT_ENCODER_SIGN` -1 (mirror-mounted motors,
  checked with motor_test.py on the new firmware). Its `S` line now also
  carries raw ADC min-max per encoder pin. **Still open:** under load (both
  motors driving the robot at >=35-40% duty) one side over-counted ~3-4x
  and both G2 channels faulted together ~1.5-2 s in -- suspected motor
  noise on the analog encoder lines and a supply dip (e-stop relay dropout
  or wiring sag). `drive_straight.py`/`rotate.py` now print encoder
  diagnostics and whether FLT clears on its own after a fault; re-run them
  to narrow it down. Pivot breakaway measured at 40% duty on tile
  (`min_turn_duty_cycle` 0.45); carpet needs far more.
- Nav2 params: `robot_radius` now matches `bot.urdf.xacro`'s real footprint
  and the map-then-race launch wiring (mapping.launch.py / bringup.launch.py)
  is in place; costmap inflation and controller gains are being addressed
  via an in-progress planner/controller A/B comparison rather than further
  hand-tuning DWB in isolation — see "Nav2 planner/controller comparison"
  below
- BNO055: mounted 2026-09-28 (see Hardware above), 94 mm off the ground
  and ~9 cm behind the front axle, i.e. close to the front motors -- the
  NDOF magnetometer heading WAS disturbed there, hence IMUPLUS mode (see
  Hardware above). Still to verify: gyro drift over a full mapping run,
  and orientation_covariance (a rough placeholder) against real data
- Ultrasonic trigger/echo pin assignment on the Teensy confirmed 2026-09-27
  (see Hardware above) — was TBD, and the design also dropped from three
  sensors to two (left/right only) in the same change. `max_range_m`/field
  of view in `bot_odometry`'s wheel_odom_node are still unverified
  placeholders against the actual sensor datasheet.
- Frontier exploration still drives slowly/unreliably in sim even after the
  goal-placement bug below was fixed — see "Frontier exploration
  reliability investigation" below; current lead suspect is dev-machine
  compute contention, not a further code bug, but that isn't confirmed yet
- AMCL's initial pose is a hardcoded constant, real-hardware-ready only in
  appearance (2026-09-27): `nav2_params.yaml`'s `set_initial_pose: true` /
  `initial_pose: {x: 19.0, y: 4.75, yaw: pi}` exists because the
  vision-only autonomous start means no human is ever present to publish
  `/initialpose` — without a seed, AMCL never starts broadcasting `map`,
  and everything downstream hangs waiting on that transform forever. In
  sim this is exact, since `gazebo_sim.launch.py` deterministically spawns
  the robot at that same coordinate. On real hardware it's unconditional
  (not gated by `use_sim`/`use_sim_time`) — AMCL would seed at that exact
  fixed point regardless of where the robot is actually placed, an
  implicit and currently undocumented assumption that the robot will be
  placed at a physical spot surveyed to match it precisely. Not yet
  decided how to actually handle this for the real course: candidates are
  (a) physically calibrate + document a marked start position against the
  real map's origin — lowest effort, fragile on placement precision every
  run; (b) switch AMCL to global localization mode, trading a slightly
  slower first-scan convergence for removing the placement-precision
  requirement entirely; (c) have `start_trigger_node` (already
  vision-based) estimate and publish `/initialpose` itself from a known
  visual marker, replacing the hardcoded seed — most robust, most work.
- A restored/stale map can silently mismatch the hardcoded AMCL initial
  pose above (found 2026-09-27): `config/maps/map.yaml`'s saved origin
  varies noticeably between separate mapping runs against the same course
  and spawn point (one run's map covered `x=19.0` comfortably, another's
  fell 0.16m short of it) — consistent with the frontier exploration
  reliability issues above producing incomplete coverage near the map's
  edges, not a config drift. A map that's just barely too small places
  AMCL's seeded pose outside the map entirely, surfacing as "robot out of
  bounds" in Nav2/RViz despite Gazebo's ground-truth spawn being correct.
  No automated check currently catches this before it's committed or
  raced on — worth a validation step (confirm a freshly-saved map's
  bounds actually contain `nav2_params.yaml`'s `initial_pose` with margin)
  before trusting any given `map.yaml` for a race run. Temporary
  workaround applied 2026-09-27: `initial_pose.x` nudged 19.0 → 18.5 to
  fall back inside the current map's bounds, at the cost of a small known
  inaccuracy versus the real spawn point. Revert to 19.0 once the map is
  regenerated to actually cover it.

## E-stop wireless link: real-hardware bring-up and sequencing (2026-09-26)

Both `arduino_ws` and `feather_ws` are now tested and confirmed working on
real hardware — the 2026-09-20 entry below covers when they were first
written, but that was compiled-only, never wired up. Real hardware
surfaced several placeholder-vs-actual-pin mismatches, all now corrected:

- **Board identification:** the kill-switch transmitter is a "YD-RP2040"
  Feather-form-factor clone board — it identifies over USB with Adafruit's
  own Feather RP2040 vendor/product ID (239A:80F1/80F2), a common clone
  pattern (reusing the factory bootloader identity rather than registering
  a new one). `feather_ws/platformio.ini` targets PlatformIO's
  `adafruit_feather` board definition (earlephilhower RP2040 core, via
  maxgerhardt's community platform fork, since the official PlatformIO
  `raspberrypi` platform has no Feather RP2040 board at all).
- **feather_ws confirmed pins:** kill switch on GPIO20; the Ebyte E32
  module's M0/M1 mode-select on GPIO10/GPIO11 (driven LOW/LOW for Normal
  transparent mode); module UART on `Serial2` (RP2040 UART1), GP8 TX /
  GP9 RX, crossed to the module's RXD/TXD. This board variant's own
  `pins_arduino.h` defines `Serial2`'s pins as the dummy value 31 ("not
  pinned out"), unlike the generic "pico" board where Serial2 defaults to
  GP8/GP9 automatically — an explicit `setTX(8)/setRX(9)` remap in
  `setup()` before `.begin()` is required, and was the actual root cause
  of an extended period where the module never responded to anything,
  even once the wiring itself was corrected.
- **arduino_ws confirmed pins:** module UART RX/TX on the Nano ESP32's
  "RX0"/"TX1" silkscreen pins, `D0`/`D1` in the board variant's own macros
  (GPIO44/GPIO43) — an earlier placeholder had guessed raw GPIO4/GPIO5, a
  real mismatch. M0/M1 on `D3`/`D4`. Relay on `D5` (GPIO8) — an earlier
  raw-number placeholder (`7`) had actually resolved to `D4`'s GPIO (7), a
  different physical pin than the confirmed D5 wiring. All of these now
  use the board variant's own symbolic `Dx` macros rather than raw GPIO
  numbers, specifically to avoid this class of mismatch recurring.
- **BLE removed from arduino_ws:** it was only ever used for debug/
  identification (finding the board over a Bluetooth scan), never part of
  the e-stop signal path, and was the prime suspect for an intermittent
  boot hang observed during bring-up (board would enumerate under its ROM
  USB-Serial-JTAG fallback identity with a completely silent `Serial`,
  recoverable only by a manual double-reset of the physical button).
  Removing it eliminated the hang.
- **LoRa UART baud standardized:** both sides' MCU-to-module UART baud is
  9600 (Ebyte E32's factory default) — `arduino_ws` had been left at an
  earlier placeholder of 115200, a mismatch that would have silently
  broken the link (garbled bytes, permanent fail-safe cutoff) had it gone
  untested.
- **Confirmed working over real RF, not just bench loopback:** with the
  above fixes, the Nano reliably receives the feather's heartbeat over the
  actual wireless link (verified via continuous `H` reception matching the
  ~100ms transmit interval). The reverse direction (Nano→feather) was not
  exercised the same way and is unconfirmed — not a concern for the safety
  path itself, since only the feather→Nano direction carries the actual
  kill-switch signal.
- **Link is not yet rock-solid:** occasional dropped heartbeats still
  occur (visible as the Nano's status LED briefly blinking even under
  normal operation) — likely a still-marginal physical connection
  (breadboard jumpers) to the feather's module specifically, not a
  protocol or pin-configuration problem (both are independently confirmed
  correct via a UART loopback test and the module's own real
  command-response during bring-up). Re-seating or soldering that
  module's connections is the next step toward full reliability;
  `HEARTBEAT_TIMEOUT_MS` was raised from 200ms to 400ms to tolerate more
  of this while the wiring is finalized, then to 700ms on 2026-09-29
  after the relay kept clicking at course range (still under the rules'
  1s cutoff). That masks a weak link: fix antenna placement/height, module
  power and air rate. `scripts/estop_link_log.py` logs relay dropouts from
  the Pi (via the G2's FLT pins) for walk-out range tests.

**Sequencing / protocol, for reference:**

1. `feather_ws` reads the kill switch every loop iteration (asymmetric
   debounce: instant trip, 50ms debounced release) and sends one line —
   `H\n` (OK) or `X\n` (asserted) — over its LoRa module every
   `REPORT_INTERVAL_MS` (100ms / 10Hz), regardless of whether the state
   changed since the last report.
2. `arduino_ws` reads whatever bytes have arrived from its own paired
   module every loop iteration. The most recently parsed `H` or `X` sets
   the relay state immediately.
3. Independent of message content, if no valid `H`/`X` has arrived within
   `HEARTBEAT_TIMEOUT_MS` (700ms) of the last one — or none has ever
   arrived, e.g. right after power-on — the relay is forced de-energized
   (fail-safe), overriding whatever the last received state was.
4. The relay only re-energizes on a clean `H` received while not
   link-lost. There is no separate "resume" message — the transmitter
   simply goes back to sending `H` once the switch clears.
5. Status LEDs (for diagnosis without a USB serial monitor): `feather_ws`'s
   LED (GPIO25 — this clone's `LED_BUILTIN`/GPIO13 is unpopulated; it
   separately has an unused NeoPixel) blinks while sending `H` and goes
   solid while sending `X`. `arduino_ws`'s LED is off while healthy (last
   message was `H`), solid while an `X` is asserted, and blinks
   specifically when the link is lost/stale — so a fail-safe triggered by
   a timeout is visually distinguishable from a deliberate kill-switch
   press.

## Frontier exploration reliability investigation (2026-09-20, in progress)

`mapping.launch.py`'s autonomous exploration (`bot_explore`'s
`frontier_explore_node`) was measured as barely moving in sim on both
courses — `scripts/measure_exploration_progress.py` (straight-line
start→end displacement + path length over a fixed sim-time window) gives
a repeatable way to check this rather than eyeballing it.

**Confirmed and fixed:** `_find_frontiers()` was picking each frontier's
goal as the raw arithmetic mean of a cluster's cells. A cluster that wraps
around an obstacle corner — which is exactly what creates a frontier there
in the first place, since the sensor shadow behind the obstacle is the
unknown space — can have a mean that lands outside every real free cell,
sometimes exactly on the obstacle itself, even though each individual
member cell is genuinely free. Checked against both course world files:
pre-fix goals landed 0.00–0.44m from a real bale. Fixed by using each
cluster's medoid (an actual member cell) instead of the mean, plus a new
`min_obstacle_clearance_m` param that drops clusters without room around
them. Before/after (60s sim-time window, displacement / path length):
obstacle course 0.008m / 0.075m → 0.325m / 0.325m; speed course
0.000m / 0.000m → 0.199m / 0.864m.

**Added, but unproven for the remaining slowness:** `min_nav_distance_m`
skips sending Nav2 a goal already within the robot's own approach
tolerance, meant to avoid triggering DWB's `RotateToGoalCritic`'s
rotation-only phase over a trip too short to need it. The specific case
that motivated this turned out to be a coordinate-frame mistake in the
analysis (a `map`-frame goal compared directly against a `world`-frame
spawn point) — the goal in question was actually ~0.88m away, not ~0.18m,
so this fix doesn't explain the stall it was built to address. Kept as a
defensive measure for genuinely-close frontiers, not as a proven fix.

**Still unresolved, current lead suspect is the dev machine, not the
code:** even with the above fix, a 240s run spent 60% idle, 38.1% purely
rotating in place (hitting the configured 1.0 rad/s max), and only 1.9%
actually driving (capped at 0.21 m/s, well under the configured 0.5 max/s)
— while the local costmap was completely empty (checked live, zero
nonzero cells) for the entire stall. Logs showed the global planner hand
DWB a new path roughly once per second (`Passing new path to controller`),
`Control loop missed its desired rate of 20Hz` (actual 6.7–13.9Hz), and a
plain 90° `spin` recovery timing out in open space. A host-level check
during one run found two unrelated `ruby` processes consuming ~3 CPU
cores and `/clock` actively dropping messages ("message was lost", 282 in
one sample) — consistent with CPU contention starving the control loop
rather than a Nav2/DWB config problem. **Not yet confirmed** via Gazebo's
own real-time-factor stat (`gz stats` — watch for RTF sustained well below
1.0 during a run); do that before spending more effort retuning DWB
critics or the progress checker, since a starved control loop can't be
fixed by retuning it.

## Nav2 planner/controller comparison (in progress, not a decided architecture)

The stock config (`nav2_params.yaml` / `nav2_mapping_params.yaml`) runs
`NavfnPlanner` + `DWBLocalPlanner`, with DWB's critics hand-tuned reactively
against specific corridor failure modes. Rather than continuing to hand-tune
DWB in isolation, three alternatives are being empirically A/B tested
against that baseline — **no decision has been made yet**, this is
exploratory scaffolding, not a settled choice the way the rest of this file
describes:

- **`ThetaStarPlanner`** — any-angle A* variant, no turning-radius
  constraint (architecturally the best fit for a robot with no minimum
  turning radius), but has no `tolerance` parameter at all — a real risk
  for `frontier_explore_node`'s frontier-boundary-adjacent goals during
  mapping specifically.
- **`SmacPlannerLattice`** — motion-primitive-constrained search using the
  `diff` motion model (includes in-place-rotation primitives, unlike the
  already-rejected `SmacPlannerHybrid`/Reeds-Shepp above). Uses the shipped
  0.5m-turning-radius lattice file; no custom-radius YAML override exists
  for this plugin (radius comes from the lattice file's own metadata), and
  generating a custom-radius file would require the upstream
  lattice-generator tool, not installed on this system.
- **`MPPIController`** — replaces DWB. This Jazzy install's own
  `nav2_bringup` reference config defaults to MPPI, not DWB — a strong
  upstream signal. Main open risk: default `batch_size: 2000` compute cost
  on the eventual Raspberry Pi 5 (confirmed single-threaded, no
  OpenMP/TBB) is unbenchmarked on real hardware.

Config file naming: `nav2_params_<variant>.yaml` (race/bringup) and
`nav2_mapping_params_<variant>.yaml` (mapping), each a full copy of its
baseline with only one plugin block changed (`planner_server.GridBased` for
the two planners, `controller_server.FollowPath` for MPPI) — see those
files' own header comments for the full rationale per variant.

Run the comparison with `scripts/run_nav2_variant_test.sh` (see that
script's own usage comment), or manually via
`ros2 launch bot_bringup bringup.launch.py use_sim:=true nav2_params_file:=<variant>.yaml`
/ the equivalent for `mapping.launch.py`.

## Wheel odometry + ground-truth pose comparison (2026-09-20)

Added real wheel encoder support and a way to directly test the leading
suspect in the frontier exploration investigation above (that `/odom` might
be lying about displacement during a stall, not just under-driven by a
starved control loop):

- **`bot_odometry`** (new package): wheel_odom_node converts Teensy-reported
  quadrature encoder ticks (one encoder per side, over USB serial) into
  `nav_msgs/Odometry` on `odom` — same topic/frame/message shape as
  `bot_gazebo`'s DiffDrive plugin already publishes in sim, so this is a
  drop-in for real hardware with no EKF/Nav2 config changes. Real hardware
  only; sim keeps using DiffDrive's own `/odom`.
- **`teensy_ws`** (previously empty): new PlatformIO firmware reading the
  two encoders and reporting ticks over serial at ~50Hz (later extended to
  also report ultrasonic ranges — see "Ultrasonic moved to Teensy" below).
  E-stop turned out to be a separate Arduino Nano doing a direct hardware
  motor-driver disconnect, not a Teensy link as this doc previously assumed
  (corrected above in Safety architecture). Accordingly,
  `bot_safety/watchdog_node.py`'s MCU-serial e-stop mirror (which assumed
  that wrong architecture) is now commented out.
- **Ground-truth pose bridge** (`bot_gazebo/urdf/bot.urdf.xacro`'s new
  `PosePublisher` plugin, bridged in `gazebo_sim.launch.py` as
  `/model/bot/pose`) plus **`scripts/compare_odom_to_ground_truth.py`**:
  sim-only diagnostic that compares `/odom`'s displacement against Gazebo's
  actual model pose. DiffDrive's `/odom` is kinematic (integrated from
  wheel joint angle, zero-slip assumed) — if the chassis is physically
  wedged/colliding while the wheels still spin, `/odom` will over-report
  displacement and feed slam_toolbox a wrong believed pose, which matches
  the "robot thinks it moved, scan gets written at the wrong place, ends up
  stuck against a wall" symptom reported during the stall.

**Not yet run/confirmed:** this comparison script hasn't actually been run
against a stalled exploration run yet — that's the next concrete step
before spending more effort on DWB/CPU-contention tuning for the frontier
exploration stall. If it shows large divergence during a stall and
near-zero divergence during normal driving, that confirms wheel slip/
collision as the root cause rather than the control-loop-starvation lead.

## Bridge/helix ramps falsely marked as obstacles (2026-09-20, fixed)

The obstacle course's climbable elements (`bridge`, `helix` in
`obstacle_course_cfr.world`) were being treated as impassable obstacles by
Nav2's costmaps — a real limitation, not a sim quirk: the 2D RPLIDAR's
returns are all at one fixed mounted height, so as the robot approaches a
ramp, the beam hits the rising deck surface at the point where it crosses
that height and reports a wall there; the OAK-D depth `voxel_layer` sees
the same rising deck/rails within its obstacle height band and marks them
too. This will recur on the real course.

Fixed sim-side only (chosen over a full Nav2 Keepout Filter for now, given
timeline — see GAZEBO_WORLDS.md's Notes section for the exact mechanism):
the ramp/deck-surface visuals in `obstacle_course_cfr.world` are tagged
`visibility_flags=1`, and `bot.urdf.xacro`'s `lidar_sensor`/`oak_depth_sensor`
clear that same bit in their `visibility_mask`, so those sensors render
straight through the decks while physics collision (and thus climbing) is
untouched. **Open item:** this doesn't help the real robot on the actual
course — a real fix there (Keepout Filter with a mask over the known
official ramp geometry, or another slope-aware approach) is still needed
before race day.

## E-stop Nano firmware, remote kill-switch, BNO055 driver, and ultrasonic-on-Teensy (2026-09-20)

Four related hardware-integration pieces landed together:

- **`arduino_ws`** (previously empty): the primary e-stop Nano's firmware
  is now written — a fail-safe deadman implementation, not a simple
  on/off relay. Its serial RX is fed by a UART LoRa transceiver module
  (not the Pi, preserving electrical isolation from the Pi/ROS 2 stack per
  the Safety architecture above). The relay is de-energized (motors cut)
  by default at power-on, on an explicit `X` e-stop message, or if no `H`
  heartbeat arrives within 200ms (covers wireless signal loss), and only
  re-energizes on a clean heartbeat. The relay breaks the motor driver's
  power supply directly, not a logic-level enable/disable line (corrected
  above in Safety architecture — an earlier version of this doc and of
  the firmware's own comments assumed the latter). Untested on real
  hardware; `RELAY_PIN`, relay polarity, and the LoRa module's UART baud
  are still placeholders (see Open items).
- **`feather_ws`** (new, previously empty): the remote kill-switch
  transmitter on the other end of that LoRa link. A Feather reads a
  physical kill-switch input pin (normally-closed to ground,
  `INPUT_PULLUP`, so idle reads LOW and a press/cut wire/dead switch all
  read HIGH — wiring faults fail toward e-stop) and reports status
  continuously over its own paired LoRa module using the same `H`/`X`
  protocol the Nano expects, every 100ms (well inside the Nano's 200ms
  timeout). Debounce is asymmetric on purpose: any single HIGH reading
  trips e-stop immediately, but clearing back to OK requires 50ms of
  continuous LOW first — standard safety-relay practice (instant trip,
  debounced release). Assumes a transparent-serial LoRa module needing no
  AT-command setup; untested on real hardware (see Open items).
- **`bot_imu`** (new package): bno055_node reads the BNO055 over I2C
  (Pi 5's hardware I2C1 bus, GPIO2/GPIO3) in NDOF fusion mode, publishing
  `sensor_msgs/Imu` on `imu` for `robot_localization`'s EKF — resolving
  the previously-undecided I2C-vs-UART interface question. Untested on
  real hardware; mounting location is still undecided, and
  orientation_covariance is an unverified placeholder (see Hardware
  above).
- **Ultrasonic moved to Teensy:** the old `bot_ultrasonic` package
  (Pi-side, bit-banged RPi.GPIO trigger/echo timing) is retired. The
  three HC-SR04-class sensors are now wired to the same Teensy that
  reports wheel encoder ticks, using `pulseIn()` to time echoes and
  reporting raw pulse widths (not pre-converted distances — same
  "MCU reports raw samples, Pi applies the math" split already used for
  encoder ticks) over the same one USB serial link, at a slower ~20Hz
  cadence than the ~50Hz encoder reports. This forced a serial-ownership
  decision: since the Teensy exposes a single virtual COM port (not a
  dual-serial USB config), two independent ROS nodes can't safely read
  it at once, so `bot_odometry`'s wheel_odom_node was extended to also
  parse the new `U,...` lines and publish `sensor_msgs/Range`, rather
  than keeping ultrasonic as a separate node/package. Chosen deliberately
  over giving the Teensy a second virtual serial port, for simplicity
  (one USB cable, one node). Trigger/echo pin assignment is still a
  placeholder (see Open items).

## Prep-day changes (2026-10-01, sim-tested only -- verify on the robot)

See PREP_DAY.md for the field plan, the mapping fallback ladder (A', B =
race on the live slam map, C = robotics-challenge-2026 gap follower) and
the "if bringup fails" guide.

- **Routes and start poses belong to a map:** `config/maps/<map>.route.yaml`
  (checkpoints, recorded with the dashboard's "Add checkpoint here") and
  `<map>.start.yaml` ("Set race start pose here"). bringup pairs them with
  `map:=` automatically (`route:=` / `initial_pose:=` override).
  lap_navigator's built-in `_COURSE_CHECKPOINTS` are from an older sim map
  frame and are wrong on the real course (and on the current sim map).
- **lap_navigator resumes after a Nav2 abort** from the first unreached
  checkpoint (max_retries 5, 3 s apart) instead of stopping for the race.
- **Nav2 footprint is the real rectangle** `[[+-0.225, +-0.195]]` + 0.02
  padding in all 9 params files, not `robot_radius: 0.26`: the circle was
  7 cm too wide per side and the sim planner refused a lane the robot had
  just driven through by teleop.
- **Speed caps** `max_speed:=`/`max_turn:=` (bringup and mapping) write a
  capped copy of the Nav2 params (controller, velocity smoother, spin).
- **Plan B** `mapping.launch.py race:=true route:=<name>`: live slam
  map + Nav2 + start trigger + lap_navigator (dashboard button).
- **bringup no longer forces `ROS_DOMAIN_ID=0`** -- it cut the dashboard off
  on any other domain. Race day: unusual `ROS_DOMAIN_ID` +
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (RACE_DAY.md).
- **`scripts/check_localization.py`** scores lidar-vs-map at AMCL's pose
  (82% localized vs 2% with a deliberately wrong pose in the sim) -- run
  before every start. Map-frame poses can't be compared with Gazebo's
  world frame (the sim map's origin is near the spawn).
- Dashboard: SLAM settings (`config/slam_toolbox_overrides.yaml`, loaded via
  `slam_params_overrides:=`), live cmd_vel/pose, topic reader (blocks
  `/oak/` and `/scan`), Nav2 variant pickers, resume pose saved with each
  checkpoint.
- `scripts/check_heading.py` compares IMU orientation / IMU gyro / wheels /
  EKF / slam heading during a hand turn.
- Laptop-only gotcha: building with the repo's `.venv` active puts
  `#!.../.venv/bin/python` in the installed node scripts (works, but the Pi
  must build with system python -- setup_pi.sh does).

## Pi 5 race deployment (2026-09-25)

- **GPIO library is `lgpio`, not pigpio/RPi.GPIO.** pigpio doesn't support
  the Pi 5's RP1 I/O chip and has no Ubuntu 24.04 arm64 package; RPi.GPIO
  doesn't work on the Pi 5 either. `motor_node` and `watchdog_node` both
  use lgpio against `gpiochip4` (RP1 header GPIO on Ubuntu's 6.8 raspi
  kernel; `gpio_chip` param if that ever changes). lgpio PWM is
  software-timed, capped at 10 kHz — RP1 hardware PWM on GPIO12/13 via
  sysfs is the upgrade path if motor jitter shows up under Nav2 CPU load.
  Both nodes take a `dry_run` param (used by their unit tests).
- **`bot_bringup/launch/hardware.launch.py`** is the real-hardware layer
  (robot_state_publisher from `bot.urdf.xacro`, RPLIDAR, OAK-D + depth
  point cloud, motor/watchdog/wheel_odom/bno055 nodes), included by both
  `bringup.launch.py` and `mapping.launch.py` unless `use_sim:=true`.
  Before this, every Pi-only node was commented out of the race launch
  and nothing published the URDF's static TF on hardware.
- **`scripts/setup_pi.sh`** (run with sudo) installs the ROS 2 Jazzy
  runtime + Nav2/SLAM/driver packages (no Gazebo/RViz — Pi is headless),
  adds udev rules (`/dev/rplidar`, `/dev/teensy` symlinks, OAK-D USB
  permissions), adds the user to `dialout`, and builds the workspace.
  **`scripts/check_hardware.sh`** is the race-morning preflight
  (`--topics` also checks data flow while bringup is running).
- **Launch-config leak:** depthai's `camera.launch.py` sets launch configs
  (`use_composition='true'`, `name`, `namespace`) that leak into later
  includes; nav2_bringup then evals `not true` and the whole launch dies
  with "name 'true' is not defined". `hardware.launch.py` wraps the OAK-D
  include in a scoped `GroupAction` — keep it that way. A launch that
  crashes this way also leaves orphaned node processes (still holding
  GPIO/serial) — `scripts/clean_robot.sh` (run before every launch on the
  Pi) stops this workspace's nodes, the lidar driver, and any process
  holding a sensor device (via `fuser`), SIGINT first so motor_node's
  shutdown runs, drives the motor pins low, then runs `clean_sim.sh` (the
  general, device-agnostic ROS/Gazebo cleanup used on the laptop too). Launches with an RViz node skip it when `rviz2` isn't
  installed rather than crashing midway, for the same reason.
