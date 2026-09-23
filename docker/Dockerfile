FROM osrf/ros:jazzy-desktop
RUN apt update && apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    ros-jazzy-cv-bridge \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/bot_ws

# Build (from the repo root):
#   docker build -t bot_ws_jazzy -f docker/Dockerfile .
#
# On the host, allow the container to use the X server (needed for Gazebo's GUI):
#   xhost +local:docker
#
# Run, mounting this repo and forwarding the X11 display:
#   docker run -it \
#     -v /mnt/data/claude_bot/bot_ws:/root/bot_ws \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     --device=/dev/dri:/dev/dri \
#     --group-add video \
#     bot_ws_jazzy
#
# If /dev/dri isn't available on the host (no GPU passthrough), drop the
# --device/--group-add flags above and add -e LIBGL_ALWAYS_SOFTWARE=1
# instead -- Gazebo/RViz will render in software (slower, but works).