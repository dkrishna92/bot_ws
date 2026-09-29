# Race-Day Commands (manual fallback)

Raw terminal commands to run the robot on race day **without** the web
control dashboard (`bot_web_control`), in case the UI is unavailable. Every
command here is what that dashboard runs for you under the hood.

Run everything on the **Pi**, from the workspace root, as your normal user.
This assumes `scripts/setup_pi.sh` has already been run once and the course
map already exists (`src/bot_bringup/config/maps/map.yaml`). The real
emergency stop is always the **physical kill switch / wired RJ45 e-stop**
(a dedicated MCU cutting motor power) — nothing below is a substitute for it.

```bash
cd /mnt/data/claude_bot/bot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## 1. Preflight — is the hardware present?

```bash
scripts/check_hardware.sh
```

Every line should read `OK` (RPLIDAR, Teensy, OAK-D, BNO055, gpiochip4,
python deps, ROS, build, race map). Fix any `FAIL` before launching.

## 2. Clean up before launching

Frees the sensors/motor GPIO from any orphaned process and drives the motor
pins low. Run this before **every** launch — a previously crashed launch can
hold the lidar port / motor GPIO and make the next one fail.

```bash
scripts/clean_robot.sh
```

## 3. Launch the race

Headless Pi (no RViz). Pick the course you're racing:

```bash
# Speed course (3 laps, default)
ros2 launch bot_bringup bringup.launch.py rviz:=false course:=speed_course_cfr

# Obstacle course (2 laps)
ros2 launch bot_bringup bringup.launch.py rviz:=false course:=obstacle_course_cfr
```

Leave this terminal running — it is the whole stack (drivers, Nav2,
localization, motor, safety, vision start trigger, lap navigator).

Useful extra args (all optional):

| Arg | Default | Purpose |
|-----|---------|---------|
| `course:=` | `speed_course_cfr` | `speed_course_cfr` or `obstacle_course_cfr` |
| `num_laps:=` | `0` | `0` = course default (3 speed / 2 obstacle); override to force a lap count |
| `rviz:=` | `true` | `false` on the headless Pi (also auto-skips if RViz isn't installed) |
| `nav2_params_file:=` | `nav2_params.yaml` | Swap Nav2 planner/controller variant (see CLAUDE.md) |
| `map_bounds_margin_m:=` | `0.5` | Startup check that the map covers the AMCL start pose; `0.0` disables |

## 4. Verify data is flowing (optional, second terminal)

```bash
cd /mnt/data/claude_bot/bot_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
scripts/check_hardware.sh --topics
```

Checks `/scan`, `/odom`, `/imu`, `/oak/points`, `/odometry/filtered` are
publishing while the launch above runs.

## 5. Starting the run

**Automatic (no penalty):** the robot starts itself when
`start_trigger_node` sees the start signal flip red→green on the OAK-D
camera. Nothing to type — just launch (step 3) before the signal flips.

**Manual start (incurs the 5 s time penalty — use only if vision fails):**
in a second sourced terminal, publish the latched start signal the lap
navigator is waiting on:

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

```bash
# Autonomous frontier exploration (drives itself, saves the map when done)
ros2 launch bot_bringup mapping.launch.py course:=speed_course_cfr

# Or drive it yourself:
ros2 launch bot_bringup mapping.launch.py course:=speed_course_cfr teleop:=true
# ...in a separate terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
# ...then save once the course is covered:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: 'src/bot_bringup/config/maps/map'}}"
```

See `src/bot_bringup/launch/mapping.launch.py`'s docstring for resume /
checkpoint options.

## Quick troubleshooting

| Symptom | Command / fix |
|---------|---------------|
| Lidar "scan stale", motor "GPIO busy" | `scripts/clean_robot.sh`, then relaunch |
| A device `FAIL` in preflight | Re-seat USB / check `/dev/rplidar`, `/dev/teensy`, I2C-3; see `check_hardware.sh` output |
| Robot won't start on the signal | Confirm `/start_signal` — `ros2 topic echo /start_signal`; use manual start (step 5) as last resort |
| "robot out of bounds" in logs | Map doesn't cover the AMCL start pose — rebuild the map, or launch with `map_bounds_margin_m:=0.0` if you've confirmed it's fine |
| Nothing publishing | `scripts/check_hardware.sh --topics` while the launch runs |
