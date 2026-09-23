# Docker (Gazebo sim development)

This image provides a ROS 2 Jazzy + Gazebo + Nav2 environment for running
this workspace's simulation, since the host laptop doesn't have these
installed natively. See root `CLAUDE.md` for why Docker isn't used for the
Pi 5 deployment itself -- this is a laptop dev convenience only.

## Build

From the repo root (`bot_ws/`):

```bash
docker build -t bot_ws_jazzy -f docker/Dockerfile .
```

## Run

On the host, allow the container to use the X server (needed for Gazebo/RViz):

```bash
xhost +local:docker
```

Run, mounting this repo and forwarding the X11 display:

```bash
docker run -it \
  -v /mnt/data/claude_bot/bot_ws:/root/bot_ws \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  --group-add video \
  bot_ws_jazzy
```

If `/dev/dri` isn't available on the host (no GPU passthrough), drop the
`--device`/`--group-add` flags above and add `-e LIBGL_ALWAYS_SOFTWARE=1`
instead -- Gazebo/RViz will render in software (slower, but works).

## Build and launch the workspace (inside the container)

```bash
cd /root/bot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch bot_bringup mapping.launch.py use_sim:=true
```

## Run Python tests (inside the container)

After building/sourcing as above:

```bash
python3 -m pytest src/*/test/ -v
```

Or a single package's tests:

```bash
python3 -m pytest src/bot_explore/test/test_frontier_explore_node.py -v
```

`colcon test` also works, but has shown a spurious "no tests ran" (exit
code 5) result tied to stale build metadata after adding a new test file --
rebuild the affected package first (`colcon build --packages-select
<pkg>`) if you see that, or just use plain `pytest` above for quicker
iteration.
