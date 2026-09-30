# Prep Day Plan (day before the race)

About 8 hours of on-and-off field access. The goal is **one safe,
complete autonomous run of each course**, then speed. A slow lap that
finishes beats a fast one that doesn't.

Commands assume the Pi, in `~/bot_ws`, with the workspace sourced. The
race-day procedure itself is [RACE_DAY.md](RACE_DAY.md).

## The course surface

Asphalt road everywhere; the obstacle course also has **potholes** and
**sand**; there may be **loose hay** on the road.
- **Asphalt:** grippy, so pivoting in place takes much more power than on
  tile (tile broke away at 40%). Measure first (Block 1) and prefer Q/E arcs.
- **Loose hay:** the depth camera now ignores anything under 6 cm (was 2 cm,
  which would have marked hay and depth noise as obstacles). If the costmap
  still shows phantom obstacles on open road, raise
  `min_obstacle_height` in the `nav2*params*.yaml` voxel layers.
- **Potholes are invisible to the robot:** the lidar sees only things at its
  height, and the camera marks only things *above* the road. Nav2 will plan
  straight through them, so **record route checkpoints that steer around
  them** (an extra checkpoint on each side of a pothole).
- **Sand:** the wheels slip (odometry overstates distance, and AMCL corrects
  it with the lidar) and load rises (driver-fault risk). Drive through slowly,
  and don't stop in it.

## Priorities, in order

1. **Safety.** The e-stop cuts the motors within 1 s at full course range. No pass, no race.
2. **A correct map of each course.** Localization and the route depend on it.
3. **A route and start pose recorded on that map.** The built-in route is
   from the sim and is wrong on the real course.
4. **One complete autonomous lap, slowly.**
5. **Vision start.** The manual start costs 5 s, so it can't sink you.
6. **Speed**, then the obstacle course and the car wash.

**Skip:** AprilTags, planner/controller A/B tests, the Python-only backup
stack, and the encoder divider rewiring.

## Before the field (bench, wheels off the ground, about 1 h)

1. **Put the Pi on the race-day router's Wi-Fi.** On the Pi, with the router
   powered up (use the current network or a monitor + keyboard to get in):
   ```bash
   sudo nmcli dev wifi connect "<router SSID>" password "<router password>"
   nmcli connection show            # the router's network should be listed
   ```
   NetworkManager remembers it and reconnects automatically. The Pi keeps its
   old networks too. On the router, give the Pi a **fixed address** (DHCP
   reservation for its Wi-Fi MAC, shown by `ip link show wlan0`) so you
   don't have to hunt for it on race day. Connect the laptop to the router
   too, then run `scripts/find_pi.sh --update`.
2. Pull or sync the latest `race_day_prep` onto the Pi.
3. `sudo ~/bot_ws/scripts/setup_pi.sh`, reboot, then `scripts/check_hardware.sh`.
   Every line must say OK.
4. `scripts/motor_test.py`: four OKs. **Watch** both wheels spin forward on
   "forward".
5. E-stop:
   - Power the robot off and on from the battery 3 times. The Arduino should
     recover by itself within about 5 s each time (`check_hardware.sh`).
   - Run `scripts/estop_link_log.py` and press the kill switch. It should log
     a dropout, and the motors should stop.
6. IMU sign: start mapping from the dashboard, run `scripts/check_heading.py`,
   and turn the robot 90° left by hand. Every source should read **positive**.
   If it reports "imu orient: OPPOSITE SIGN", fix that before mapping.
7. Solder or tape the LoRa module wiring. Mount both antennas high and
   vertical, clear of the frame.

## On the field

**Mark the race start spot with tape first.** Every mapping run, every
recorded start pose, and every race starts on exactly that spot.

### Block 1: safety and driving on the real surface (about 1.5 h)
- **E-stop range:** run `scripts/estop_link_log.py` and walk the kill switch
  to the farthest point of the course. You need **zero dropouts**. If not,
  move the antennas and walk again.
- **Stopping time:** while the robot drives slowly, press the kill switch and
  time the stop. It must stop in under 1 s.
- **Turning:** run `scripts/rotate.py` **on the asphalt**. Set `motor_node`'s
  `min_turn_duty_cycle` a few points above the breakaway duty it reports.
- **Driving straight:** run `scripts/drive_straight.py --distance 3`. Check the
  encoder diagnostics (peak tick rate, missed states) and whether any driver
  faults happen.

### Block 2: map both courses (about 2 h)
For each course, in the dashboard:
1. Robot on the marked start spot, then **Start New Mapping Run** (teleop).
   In another terminal, run `scripts/record_run.sh <course>_map`.
2. Enter the course name (e.g. `speed`) in **Map name**, then press
   **Set race start pose here** while still on the start spot.
3. Drive the course **slowly, with gentle turns**: use **Q/E** (veer
   left/right along a 1 m arc) instead of A/D (pivot in place), since pivots
   jerk and smear the map. At each corner press **Add
   checkpoint here**, in driving order. Finish one lap back near the start.
   After each good section, press **Save Checkpoint** so a bad turn only costs
   that section.
4. Watch the map build. If lanes come out rotated, see "If mapping fails"
   below.
5. Press **Save Final Map** under the same name (`speed`). The map,
   `speed.route.yaml` and `speed.start.yaml` now belong together.

### Block 3: first autonomous laps (about 2 h)
1. Robot on the start spot. In **on map** pick `speed`, set the Nav2 speed cap
   to **0.4 m/s and 1.0 rad/s**, then press **Start (bringup)**.
2. **Before starting:** run `scripts/check_localization.py`. It must say
   **OK: localized**. If it says NOT LOCALIZED, the start pose is wrong: see
   the failure guide below.
3. Start with the manual start (see RACE_DAY.md step 5) or the real signal.
   Watch the Live panel (pose, cmd_vel), with `scripts/record_run.sh` running.
4. Goal: one clean lap, then 3 laps, then raise the cap step by step (0.6,
   0.8, ... up to none).
5. **Start signal:** at the start spot run `scripts/test_start_signal.py
   --start-camera`. Check that `~/start_signal_test/latest.jpg` shows both discs,
   and set the detection region from it. Then do real flips: exactly one
   trigger per flip.

### Block 4: obstacle course and full rehearsal (about 1.5 h)
- Obstacle course laps, going slowly through the car wash.
- **Two full rehearsals** exactly as in RACE_DAY.md: preflight, launch,
  localization check, signal start, laps, stop, reset.

**If a block overruns, protect Blocks 1 and 3.**

### Tonight after the field
- Go through the recordings in `runs/`, and apply only small, tested parameter fixes.
- Commit. **Race morning: parameters only, no code changes.**

---

## If mapping fails: the fallback ladder

**Diagnose first (about 10 min):**

| Symptom | Likely cause | Quick fix |
|---|---|---|
| Lanes rotated / wrong direction | Heading | `scripts/check_heading.py` while turning. IMU flagged → fix the IMU. slam flagged → dashboard **SLAM settings**: `minimum_time_interval` 0.2, `angle_variance_penalty` 2.0, `do_loop_closing` false (test); drive slower through turns |
| Smeared or doubled everywhere | IMU not publishing, or odometry wrong | Check `/imu` in the Live panel / topic reader; drive slower (the encoders over-count under load) |
| Good lanes, then one bad jump | A false loop closure, or one fast turn | **Resume** from the last good checkpoint and remap that section |
| Pi slows down or crashes | CPU | `record_run.sh --no-camera`; raise `minimum_time_interval` again |

**Plan A′, map in pieces (up to about 1 h):** one section at a time, with
**Save Checkpoint** after each good one and **Resume** if the next goes wrong.

**Plan B, race on a live map (decide by the end of Block 2):** no saved map
needed. Robot on the start spot, **Map name** = the course whose route you
recorded, then **Race on live map (Plan B)**. slam_toolbox runs during the
race; the live map's origin is the start spot, so the route recorded from the
same spot still lines up. Same speed cap field. If lanes rotate live, the same
SLAM settings fixes apply.

**Plan C, reactive gap-following, no map (only if Plan B fails by mid-Block 3):**
`~/robotics-challenge-2026` (`python3 -m src.main --course speed`, with the ROS
launch stopped). Its hardware layer runs on this robot, but it has **never
driven** and needs about 1 h of tuning. Completes laps only if the course is
one continuous corridor.

**Decision points:** a usable saved map by the end of Block 2 means Plan A,
otherwise Plan B in Block 3. Plan B not lapping by mid-Block 3 means Plan C on
the speed course.

---

## If bringup fails (even with a valid map)

Look in the launch output (or the dashboard's terminal) for `FATAL`,
`process has died` and `launch.user` lines first. Check these in order:

| # | Symptom | Cause | Fix |
|---|---|---|---|
| 1 | Launch shuts down right after starting; `FATAL: couldn't parse map` | `map.yaml`'s `image:` doesn't match the `.pgm` next to it (seen on the Pi: `half_speed_map.pgm` after the files were renamed) | Fix the `image:` line; save maps with the dashboard's name field instead of renaming files |
| 2 | `FATAL: amcl's initial_pose ... falls outside` | The start pose isn't inside this map | Record the start pose on this map (**Set race start pose here**), or pass `initial_pose:=x,y,yaw` |
| 3 | **Everything "active", but the robot drives into bales or Nav2 fails constantly** | Wrong start pose for this map (AMCL localized somewhere else) | Run `scripts/check_localization.py`. NOT LOCALIZED means the robot isn't on the marked spot, or `<map>.start.yaml` is from another map. The log line `AMCL starts at ... from ...` shows which pose was used; "nav2 params file (hardcoded)" is a warning sign |
| 4 | Log: `No route file for this map` | No `<map>.route.yaml` | Record the route on that map. Without it, `lap_navigator` uses built-in **sim** checkpoints, which are meaningless on the real course |
| 5 | `planner_server: Failed to create plan` in a lane the robot fits | Footprint or clearance. **Fixed tonight:** Nav2 now uses the real 0.45 × 0.38 m rectangle, not a 0.26 m circle that was 7 cm too wide per side. Untested on the real robot | If it persists: check that checkpoints were recorded mid-lane; mapped bales can look thicker than they are; check `footprint_padding` (0.02) |
| 6 | Robot stops mid-route | Nav2 aborted (blocked or planning failure) | `lap_navigator` now resends the remaining checkpoints (5 retries, 3 s apart). Look for "resending from checkpoint N" in the log |
| 7 | Nav2 sends commands (Live panel cmd_vel ≠ 0) but the wheels don't move | E-stop relay open: radio link lost, or the Arduino stuck in download mode | Arduino LED (blinking = link lost); `check_hardware.sh`; `estop_link_log.py` |
| 8 | Motors stop by themselves under load | Driver fault from a supply dip (relay dropout / wiring sag) | `drive_straight.py` / `rotate.py` report whether the fault clears by itself; check battery and wiring |
| 9 | Pose drifts or rotates during laps | IMU not publishing (node crashed on an I2C read before tonight's fix) or heading issue | `/imu` in the Live panel; `check_heading.py` |
| 10 | Start signal never fires | Never saw red first (region wrong, or the signal already green at launch) | `test_start_signal.py`; launch while the signal shows **red**; manual start as the last resort |
| 11 | `GPIO busy`, `scan stale`, `could not open port` | A previous launch still holds the hardware | `scripts/clean_robot.sh`, then relaunch |
| 12 | Dashboard shows nothing (no pose/cmd_vel) while bringup runs | Dashboard and launch on different ROS domains. **Fixed tonight:** bringup used to force domain 0 | Start everything from the same shell environment |
| 13 | Robot reacts to things it shouldn't / strange `/cmd_vel` | Another team's ROS 2 robot on the same Wi-Fi and domain | On the Pi, before starting the dashboard: `export ROS_DOMAIN_ID=<unusual number>` and `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (the browser dashboard still works; only laptop RViz stops seeing topics) |
| 14 | `Control loop missed its desired rate` / sluggish | Pi CPU | Speed cap, `record_run.sh --no-camera`, no topic reader on heavy topics |
| 15 | "Timed out waiting for transform" for more than about 10 s | EKF or odometry not running (Teensy port, or an IMU problem) | `check_hardware.sh --topics` |

Harmless: RViz `process has died ... exit code -6` (no display; it's
skipped on the headless Pi now, and it doesn't stop the launch anyway).
