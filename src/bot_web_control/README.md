# bot_web_control

A browser dashboard for running the robot without a terminal: start and
stop the race or mapping launch, drive it with the keyboard or on-screen
buttons, save and resume mapping checkpoints, and watch the map build live.

It's a convenience tool, **not a safety device**. The Stop button is not an
e-stop. The real e-stop is the separate wireless cutoff described in
`CLAUDE.md`'s Safety architecture section. Keep that in hand whenever the
robot can move.

## Running it

Run from the workspace root: the default checkpoint path is relative to it.

**On the robot (Pi 5):**

```bash
cd ~/bot_ws
./scripts/clean_robot.sh
ros2 launch bot_web_control web_control.launch.py use_sim:=false workspace_root:=$HOME/bot_ws
```

**In simulation (laptop):**

```bash
cd ~/bot_ws
ros2 launch bot_web_control web_control.launch.py workspace_root:=$HOME/bot_ws
```

Then open `http://localhost:8080` on the same machine, or
`http://<pi-ip-address>:8080` from a laptop, phone or tablet on the same
network (`hostname -I` on the Pi shows its address).

The dashboard only starts itself. Its buttons start and stop the
bringup/mapping launches as child processes, so you can leave it running
across many runs.

> **No password.** The server listens on every network interface
> (`0.0.0.0`) with no login, so anyone who can reach port 8080 can drive the
> robot. Only use it on a network you control. To restrict it to the Pi
> itself, set the `http_host` parameter to `127.0.0.1`.

## Controls

| Control | What it does |
| --- | --- |
| **Start (bringup)** | Runs `ros2 launch bot_bringup bringup.launch.py` (the race launch) with this dashboard's `use_sim` and `world`, racing on the map picked in **on map**. |
| **on map** | The saved maps in `src/bot_bringup/config/maps`: every `<name>.yaml` whose image file exists (`map` first). Passed to bringup as `map:=<full path>`, so a map saved a minute ago works without a rebuild. The list refreshes when you open it and after **Save Final Map**. |
| **Start New Mapping Run** | Runs `mapping.launch.py teleop:=true`: SLAM builds a map from scratch while you drive with the teleop controls. |
| **Resume Mapping Run** | Same, but continues from the checkpoint named in **Map name** (default `checkpoint`), starting at the pose in x / y / yaw. |
| **Nav2** (next to Start) / **Nav2 for autonomous mapping** | Which Nav2 params variant to launch with (`nav2_params_file:=`): the `nav2_params*.yaml` / `nav2_mapping_params*.yaml` files installed with `bot_bringup`, default first (e.g. `_mppi` for the MPPI controller). Teleop mapping runs no Nav2, so it ignores the mapping choice. A new variant file needs a `colcon build` before it shows up. |
| **Start Autonomous Mapping Run** | Runs `mapping.launch.py` without `teleop:=true`: Nav2 and `bot_explore`'s frontier explorer drive the robot into unexplored space on their own while SLAM builds the map (always a fresh map). Asks for confirmation first. Heavier than teleop mapping, since it runs the full Nav2 stack. |
| **Stop** | Sends Ctrl-C (SIGINT) to the running launch, force-kills it after 5 s, publishes a zero `/cmd_vel`, and, if `workspace_root` is set, runs `clean_robot.sh` (robot) or `clean_sim.sh` (sim) to catch leftover processes. It passes `--keep-dashboard`, so the dashboard itself keeps running. |
| **Map name** | Optional bare name for saves and for which checkpoint Resume uses; blank = the defaults (`checkpoint` / `map`). |
| **Save Checkpoint** | Calls slam_toolbox's `/slam_toolbox/serialize_map` (the pose graph, resumable) without ending the run, and records the robot's current map pose (slam_toolbox's `/pose`) next to it as `<name>.pose.json`. |
| **Save Final Map** | Calls `/slam_toolbox/save_map`: the `.pgm`/`.yaml` pair bringup races on. Not resumable. |
| **x / y / yaw (rad)** | The robot's pose in the checkpoint's map, used when resuming. Filled in automatically from the pose recorded at Save Checkpoint (for the checkpoint in **Map name**); only edit it if the robot isn't back where it was when you saved. |
| **Teleop** | Hold **W / S** to drive forward / back and **A / D** to turn left / right (combine for arcs), or hold the arrow buttons. The page sends commands every 100 ms while a key or button is held. |
| **linear / angular speed** | Teleop speed in m/s and rad/s (defaults 0.3 and 1.0). |
| **Map** | The latest `/map`, reloaded every 2 s (blank until a map has been published). |

## How teleop stops

Several layers each stop the motors on their own:

1. **The dashboard** publishes zero velocity if the browser goes quiet for
   0.5 s (tab closed, Wi-Fi drop, laptop asleep).
2. **`motor_node`** stops the motors if `/cmd_vel` is older than 0.3 s.
3. **`watchdog_node`** holds the motors stopped whenever the lidar isn't
   publishing.
4. **The wireless e-stop** cuts motor power in hardware, independent of all
   of the above.

On the real robot, Start and Mapping Run bring up the motor node, so teleop
drives the real wheels. Test with the wheels off the ground first.

## Mapping a course with checkpoints

1. **Start New Mapping Run** and drive the course slowly.
2. Before a break, stop where you'll restart from and press **Save
   Checkpoint** (optionally with a **Map name**). The pose graph goes to
   `src/bot_bringup/config/maps/<name>.{posegraph,data}` and the robot's
   pose to `<name>.pose.json`.
3. Later, with the robot in the same spot: same **Map name**, check x / y /
   yaw filled in, then **Resume Mapping Run**.
4. When the course is fully covered, **Save Final Map** with a name and pick
   it in **on map** for bringup (no rebuild needed).

## Parameters

Set on the command line as `ros2 launch bot_web_control web_control.launch.py name:=value`.
The launch file exposes the first four; the rest are node parameters with
the defaults shown.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `use_sim` | `true` | Passed to bringup/mapping. **Set `false` on the robot**, which also makes Stop use `clean_robot.sh`. |
| `workspace_root` | *(empty)* | Path to `bot_ws`. Needed for Stop's cleanup sweep; empty skips it. |
| `http_port` | `8080` | Dashboard port. |
| `world` | `speed_course_cfr` | Gazebo world for mapping runs (see Limitations for bringup). |
| `http_host` | `0.0.0.0` | Interface to listen on; `127.0.0.1` for the Pi only. |
| `default_linear_speed` | `0.3` | Initial teleop linear speed (m/s). |
| `default_angular_speed` | `1.0` | Initial teleop angular speed (rad/s). |
| `checkpoint_path` | `src/bot_bringup/config/maps/checkpoint` | Default checkpoint (blank Map name), relative to the workspace root. |
| `slam_pose_topic` | `/pose` | slam_toolbox's robot pose in the map frame, recorded at Save Checkpoint. |

## HTTP API

The page is a thin client over these endpoints; `curl` works too, e.g.
`curl -X POST localhost:8080/api/stop`.

| Method | Path | Body / response |
| --- | --- | --- |
| GET | `/` | The dashboard page. |
| GET | `/api/status` | JSON: `running` (`"bringup"`, `"mapping"` or `null`), `map_available`, `linear_speed`, `angular_speed`. |
| GET | `/api/map.png` | Latest `/map` as a PNG; 503 if no map has arrived yet. |
| POST | `/api/cmd_vel` | `{"linear": m/s, "angular": rad/s}`, published as `/cmd_vel` (resets the 0.5 s staleness timer). |
| GET | `/api/maps` | JSON: `maps`, the saved race map names. |
| GET | `/api/nav2_params` | JSON: `bringup` and `mapping`, the installed Nav2 params variants. |
| POST | `/api/launch/bringup` | Start the race launch; optional body `{"map": "<name>", "nav2_params": "<file>"}` (from `/api/maps` / `/api/nav2_params`). |
| POST | `/api/launch/mapping` | Start a mapping run; body `{"resume": true, "name": "<checkpoint>", "pose": "x,y,yaw"}` to resume (pose optional: defaults to the recorded one), or `{"autonomous": true, "nav2_params": "<file>"}` for frontier exploration instead of teleop. |
| GET | `/api/checkpoint_pose?name=<checkpoint>` | JSON: `pose` (`x`, `y`, `yaw`) recorded at Save Checkpoint, or `null`. |
| POST | `/api/save_checkpoint` | Save a resumable checkpoint + its pose; optional body `{"name": "<bare name>"}`. |
| POST | `/api/save_final_map` | Save the race map; optional body `{"name": "<bare name>"}`. |
| POST | `/api/stop` | Stop the running launch (see Stop above). |

## Limitations

- **Start doesn't choose the race course.** The dashboard passes `world`,
  which `bringup.launch.py` doesn't use; the race route comes from its
  `course` argument, which the dashboard doesn't set, so it's always
  `speed_course_cfr`. For the obstacle course, run
  `ros2 launch bot_bringup bringup.launch.py course:=obstacle_course_cfr`
  from a terminal instead.
- **Start and Mapping Run don't pass `rviz:=false`.** On a Pi with RViz
  installed and a monitor attached, RViz opens on the Pi's screen; without
  RViz installed, the launches skip it.
- **One launch at a time.** Stop the current run before starting another.
