# Race-Day Commands (manual fallback)

## Packing list

- [ ] Power strip
- [ ] Wi-Fi router (the Pi must already know its network -- see PREP_DAY.md "Before the field")
- [ ] 4 charged robot batteries + battery charger
- [ ] Laptop + laptop charger
- [ ] Hex keys, screwdriver
- [ ] 9 V batteries
- [ ] Water, power bars

Also worth packing (from what this robot has needed so far):
- [ ] Kill-switch transmitter (Feather) + its power, and the spare LoRa antennas
- [ ] USB cables for the Teensy, Arduino, OAK-D and lidar (+ a spare)
- [ ] Masking tape (to mark the race start spot) and a marker
- [ ] Zip ties, electrical tape
- [ ] Multimeter
- [ ] Phone (the dashboard works in its browser)
- [ ] The wired e-stop's RJ45 cable, if the course interface needs ours

Raw terminal commands to run the robot on race day **without** the web
control dashboard (`bot_web_control`), in case the UI is unavailable. Every
command here is what that dashboard runs for you under the hood.

Run everything on the **Pi**, from the workspace root, as your normal user.
This assumes `scripts/setup_pi.sh` has already been run once and a course
map exists in `src/bot_bringup/config/maps/`. The real emergency stop is
always the **physical kill switch / wired RJ45 e-stop** (a dedicated MCU
cutting motor power) — nothing below is a substitute for it.

```bash
cd ~/bot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Keep other teams' ROS 2 robots on the same Wi-Fi out of ours (and ours
# out of theirs). Set these in EVERY shell, including the one that starts
# the dashboard. Laptop RViz can't see the robot's topics while they're set.
export ROS_DOMAIN_ID=73
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

## 0. Reach the Pi (from the laptop)

The Pi's IP changes between networks. From the laptop's `bot_ws`:

```bash
scripts/find_pi.sh            # finds it (mDNS, then a subnet sweep), prints its IP
scripts/find_pi.sh --update   # ...and points the `robot-pi` SSH alias at it
ssh robot-pi
```

Web dashboard (same controls as below, from a browser): `http://<pi-ip>:8080`.
If it isn't running:

```bash
ros2 launch bot_web_control web_control.launch.py use_sim:=false workspace_root:=$HOME/bot_ws
```

---

## 1. Preflight — is the hardware present?

```bash
scripts/check_hardware.sh
```

Every line should read `OK` (RPLIDAR, Teensy, OAK-D, BNO055, e-stop Nano,
gpiochip4, python deps, ROS, build, race map). Fix any `FAIL` before
launching. The e-stop Nano can power up stuck in ROM download mode after a
full battery power-on (`FAIL ... 303a:1001`) — the Pi normally recovers it
automatically; if not:

```bash
python3 scripts/estop_recover.py    # or press the Nano's reset button
```

## 2. Clean up before launching

Frees the sensors/motor GPIO from any orphaned process and drives the motor
pins low. Run this before **every** launch — a previously crashed launch can
hold the lidar port / motor GPIO and make the next one fail.

```bash
scripts/clean_robot.sh
```

## 3. Launch the race

Pick the course and the saved map you're racing on (RViz is skipped
automatically on the headless Pi):

```bash
# Speed course (3 laps, default), default map config/maps/map.yaml
ros2 launch bot_bringup bringup.launch.py course:=speed_course_cfr

# Obstacle course (2 laps) on a named saved map
ros2 launch bot_bringup bringup.launch.py course:=obstacle_course_cfr map:=obstacle_v2

# Any map by path (e.g. one saved since the last build)
ros2 launch bot_bringup bringup.launch.py map:=$HOME/bot_ws/src/bot_bringup/config/maps/Test.yaml
```

Leave this terminal running — it is the whole stack (drivers, Nav2,
localization, motor, safety, vision start trigger, lap navigator).

Useful extra args (all optional):

| Arg | Default | Purpose |
|-----|---------|---------|
| `course:=` | `speed_course_cfr` | `speed_course_cfr` or `obstacle_course_cfr` |
| `map:=` | `map` | Saved map name in `config/maps` (e.g. `Test`) or a full path to its `.yaml` |
| `initial_pose:=` | *(map's `<map>.start.yaml`)* | AMCL start pose `x,y,yaw`; default is the pose recorded with the map, else `nav2_params.yaml` |
| `route:=` | *(map's `<map>.route.yaml`)* | Race route recorded on that map (dashboard **Add checkpoint here**) |
| `max_speed:=` / `max_turn:=` | `0` (no cap) | Cap Nav2 speed (m/s) / turn rate (rad/s) for the first slow laps |
| `num_laps:=` | `0` | `0` = course default (3 speed / 2 obstacle); override to force a lap count |
| `rviz:=` | `true` | Auto-skipped when there's no display or RViz isn't installed |
| `nav2_params_file:=` | `nav2_params.yaml` | Swap Nav2 planner/controller variant (see CLAUDE.md) |
| `map_bounds_margin_m:=` | `0.5` | Startup check that the map covers the AMCL start pose; `0.0` disables |

## 4. Verify data is flowing (optional, second terminal)

```bash
cd ~/bot_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
scripts/check_hardware.sh --topics
```

Checks `/scan`, `/odom`, `/imu`, `/oak/points`, `/odometry/filtered` are
publishing while the launch above runs.

**Then confirm the robot knows where it is (before every start):**

```bash
scripts/check_localization.py     # must end "OK: localized"
```

NOT LOCALIZED means AMCL's start pose doesn't match where the robot is:
put it on the marked start spot, and check the launch log line `AMCL starts
at ... from ...`. Every node can report healthy while it's wrong. Also check
the launch log says `Race route: ...<map>.route.yaml`, not "No route file".

## 5. Starting the run

**Automatic (no penalty):** the robot starts itself when
`start_trigger_node` sees the start signal flip red→green on the OAK-D
camera. Nothing to type — just launch (step 3) while the signal shows
**red**: the node ignores the first 2 s (camera exposure settling), arms
only after it has seen red, then fires on green. Launched while it's
already green, it waits for red first.

Check the camera sees the signal before the race (never publishes
`start_signal`, so it can't start the robot; snapshots in
`~/start_signal_test/`):

```bash
scripts/test_start_signal.py --start-camera    # without --start-camera if bringup is running
# try a different region / thresholds:
scripts/test_start_signal.py --start-camera --ros-args -p roi:="[0.2, 0.0, 0.35, 0.4]"
```

**Manual start (incurs the 5 s time penalty — use only if vision fails):**
the dashboard's **Manual start** button (Race section; only while Plan A or
Plan B is running), or in a second sourced terminal publish the latched
start signal the lap navigator is waiting on. Both plans otherwise wait for
the green light -- for practice laps, use the button:

```bash
ros2 topic pub -1 --qos-durability transient_local \
    /start_signal std_msgs/msg/Bool "{data: true}"
```

## 6. Stopping

- **Emergency:** hit the physical kill switch / wired e-stop. This cuts
  motor power at the hardware level, independent of anything below.
- **Normal end of run:** `Ctrl+C` in the launch terminal (step 3). This
  SIGINTs the stack so `motor_node` drives the pins low on the way out.
- **Then always sweep before the next run:**

  ```bash
  scripts/clean_robot.sh
  ```

## Between runs (re-arm for another attempt)

1. `Ctrl+C` the launch terminal.
2. `scripts/clean_robot.sh`
3. Physically place the robot back at the surveyed start position (AMCL
   seeds a fixed initial pose — see CLAUDE.md).
4. Re-run step 3.

---

## If the map is missing or wrong (build it first)

`bringup.launch.py` does **not** build a map. If step 1 reports no race map,
or the course changed, build one once with the mapping launch, then race:

The easy way is the dashboard: **Start New Mapping Run** (teleop) or
**Start Autonomous Mapping Run**, then **Save Final Map** with a name, and
pick that name in **on map** next to Start (bringup). From a terminal:

```bash
# Autonomous frontier exploration (drives itself, saves the map when done)
ros2 launch bot_bringup mapping.launch.py course:=speed_course_cfr

# Or drive it yourself:
ros2 launch bot_bringup mapping.launch.py course:=speed_course_cfr teleop:=true
# ...in a separate terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
# ...then save once the course is covered (then race with map:=my_course):
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: 'src/bot_bringup/config/maps/my_course'}}"
```

Before mapping, check `/imu` is publishing (`check_hardware.sh --topics`):
the EKF takes its heading only from the IMU, and a map built without it
comes out as a smeared starburst. See `src/bot_bringup/launch/mapping.launch.py`'s
docstring for resume / checkpoint options.

## Recording a run (training / debugging)

The dashboard records every launch it starts (**Record rosbag**, on by
default; tick **+ camera** for the OAK image, ~3 MB/s more). Stop closes the
bag and says where it went: `~/bot_ws/runs/<map>_<kind>_<date-time>/` on
the Pi (kind = mapping / resume / auto / race / bringup). It holds lidar,
odometry, EKF, IMU, TF, drive commands, slam pose and the map. The status
line shows "recording runs/..." while it runs. By hand (second terminal):

```bash
scripts/record_run.sh carwash               # Ctrl-C to stop
scripts/record_run.sh carwash --no-camera   # smaller
scripts/record_run.sh --stop                # stop one started in the background
```

Study a run later, on the laptop:

```bash
ssh robot-pi "tar cf - -C bot_ws/runs speed_mapping_20261001_093000" | tar xf - -C runs/
ros2 bag info runs/speed_mapping_20261001_093000
scripts/replay_mapping.sh runs/speed_mapping_20261001_093000            # re-map from scratch
scripts/replay_mapping.sh runs/... --overrides my_slam.yaml --rviz      # try other SLAM settings
scripts/replay_mapping.sh runs/... --as-recorded --rviz                 # watch what happened
```

The re-map writes `<bag>_replay.pgm/.yaml/.posegraph` next to the bag, so a
better map from new settings can be copied into `config/maps/` and raced.

## Bench / field test scripts (no ROS -- stop the launch first)

These drive the motors or read the sensors directly and refuse to run while
the ROS stack holds them. **Wheels off the ground / clear space, e-stop in
hand.**

```bash
scripts/motor_test.py                        # each side fwd/rev vs encoders (wheels OFF the ground; watch the wheels too)
scripts/drive_straight.py --distance 2       # IMU heading hold; reports drift and breakaway duty
scripts/rotate.py                            # 360 deg pivot by IMU; reports breakaway duty -> min_turn_duty_cycle
scripts/estop_link_log.py                    # logs e-stop relay dropouts; walk the kill switch out to course range
```

Measure the pivot breakaway with `rotate.py` **on the actual course
surface** (tile ~40% duty; carpet far more) and set `motor_node`'s
`min_turn_duty_cycle` a few percent above it.

## Quick troubleshooting

| Symptom | Command / fix |
|---------|---------------|
| Lidar "scan stale", motor "GPIO busy" | `scripts/clean_robot.sh`, then relaunch |
| A device `FAIL` in preflight | Re-seat USB / check `/dev/rplidar`, `/dev/teensy`, I2C-3; see `check_hardware.sh` output |
| Robot won't start on the signal | Confirm `/start_signal` — `ros2 topic echo /start_signal`; use manual start (step 5) as last resort |
| "robot out of bounds" in logs | Map doesn't cover the AMCL start pose — rebuild the map, or launch with `map_bounds_margin_m:=0.0` if you've confirmed it's fine |
| Nothing publishing | `scripts/check_hardware.sh --topics` while the launch runs |
| Launch dies right after start | Check the log for `FATAL` (map yaml's `image:` must match the `.pgm` next to it) |
| Map smeared / rotated copies | `/imu` wasn't publishing — check `check_hardware.sh --topics`, rebuild the map |
| Bringup "works" but the robot heads into bales | `scripts/check_localization.py` -- wrong start pose; full guide in PREP_DAY.md "If bringup fails" |
| Lanes mapped in the wrong direction | While mapping, run `scripts/check_heading.py` and turn the robot ~90° left: it names the source that disagrees (IMU orientation vs gyro, EKF, or slam_toolbox's scan matcher). Then map slower (especially turns) and tune in the dashboard's **SLAM settings** |
| E-stop relay clicking / motors cut randomly | Radio link dropping heartbeats — `scripts/estop_link_log.py`; raise/move the antennas |
| `check_hardware.sh`: e-stop Nano `303a:1001` | Stuck in ROM download mode — `python3 scripts/estop_recover.py` or its reset button |
| Robot won't turn in place | Below breakaway power on this surface — `scripts/rotate.py`, raise `min_turn_duty_cycle` |
