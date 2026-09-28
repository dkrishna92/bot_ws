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
| **Start (bringup)** | Runs `ros2 launch bot_bringup bringup.launch.py` (the race launch) with this dashboard's `use_sim` and `world`. |
| **Mapping Run (teleop)** | Runs `mapping.launch.py teleop:=true`: SLAM builds a map while you drive with the teleop controls. With **Resume from checkpoint** ticked, it continues from the saved checkpoint map at the pose in the x / y / yaw fields instead of starting empty. |
| **Stop** | Sends Ctrl-C (SIGINT) to the running launch, force-kills it after 5 s, publishes a zero `/cmd_vel`, and, if `workspace_root` is set, runs `clean_robot.sh` (robot) or `clean_sim.sh` (sim) to catch leftover processes. It passes `--keep-dashboard`, so the dashboard itself keeps running. |
| **Save Map (checkpoint)** | Calls slam_toolbox's `/slam_toolbox/save_map` to save the in-progress map as the checkpoint without ending the mapping run. |
| **x / y / yaw (rad)** | The robot's pose in the checkpoint map, used when resuming. It must be where the robot actually was when the checkpoint was saved: read it with `ros2 run tf2_ros tf2_echo map base_link` before saving. On the real robot, place the robot back at that pose before resuming. |
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

1. **Mapping Run** (resume unticked) and drive the course slowly.
2. Before a break, read the pose (`ros2 run tf2_ros tf2_echo map base_link`),
   then press **Save Map**. The map is saved to
   `src/bot_bringup/config/maps/checkpoint.{yaml,pgm}`.
3. Later: enter that pose in x / y / yaw, tick **Resume from checkpoint**,
   and press **Mapping Run** again.
4. When the course is fully covered, save the final map as
   `config/maps/map` (see `mapping.launch.py`'s docstring), then rebuild:
   `colcon build --packages-select bot_bringup`. The race launch
   localizes against that file.

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
| `checkpoint_path` | `src/bot_bringup/config/maps/checkpoint` | Where Save Map writes and Resume reads, relative to the workspace root. |

## HTTP API

The page is a thin client over these endpoints; `curl` works too, e.g.
`curl -X POST localhost:8080/api/stop`.

| Method | Path | Body / response |
| --- | --- | --- |
| GET | `/` | The dashboard page. |
| GET | `/api/status` | JSON: `running` (`"bringup"`, `"mapping"` or `null`), `map_available`, `linear_speed`, `angular_speed`. |
| GET | `/api/map.png` | Latest `/map` as a PNG; 503 if no map has arrived yet. |
| POST | `/api/cmd_vel` | `{"linear": m/s, "angular": rad/s}`, published as `/cmd_vel` (resets the 0.5 s staleness timer). |
| POST | `/api/launch/bringup` | Start the race launch. |
| POST | `/api/launch/mapping` | Start a mapping run; body `{"resume": true, "pose": "x,y,yaw"}` to resume. |
| POST | `/api/save_map` | Save the checkpoint; optional body `{"name": "<path>"}`. |
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
- **Three unit tests fail** (`test_start_launch_mapping_*`): the test
  fixture builds the node without its logger. This predates the robot
  bring-up and doesn't affect the dashboard itself.
