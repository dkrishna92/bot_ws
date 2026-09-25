#!/usr/bin/env bash
# One-shot race-bot Pi 5 setup: ROS 2 Jazzy runtime + everything bot_ws
# needs on real hardware, device permissions, then a workspace build.
# Safe to re-run (idempotent). Gazebo/RViz are deliberately NOT installed --
# the Pi runs headless; sim stays on the laptop (see CLAUDE.md workflow).
#
#   sudo ./scripts/setup_pi.sh
#
# Log out/in (or reboot) afterwards so the new group membership applies.
set -euo pipefail

if [[ $EUID -ne 0 || -z "${SUDO_USER:-}" ]]; then
    echo "run as: sudo $0   (from your normal user, not a root shell)" >&2
    exit 1
fi
WS="$(cd "$(dirname "$0")/.." && pwd)"
USER_HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"

echo "== apt packages"
# ros2.sources (packages.ros.org) is already configured on this image.
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-mppi-controller \
    ros-jazzy-nav2-smac-planner \
    ros-jazzy-nav2-theta-star-planner \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    ros-jazzy-rplidar-ros \
    ros-jazzy-depthai-ros-driver \
    ros-jazzy-depth-image-proc \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pytest \
    python3-lgpio \
    python3-smbus2 \
    python3-serial \
    python3-numpy \
    python3-opencv \
    gpiod \
    i2c-tools

echo "== device permissions"
# On this Ubuntu raspi image, /dev/gpiochip*, /dev/i2c-* and /dev/ttyUSB*/
# ttyACM* are all group dialout.
usermod -aG dialout,plugdev "$SUDO_USER"

cat > /etc/udev/rules.d/80-racebot.rules <<'EOF'
# OAK-D S2 (Movidius MyriadX; re-enumerates with a different PID after boot)
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
# Stable names so the lidar and Teensy can't swap ttyUSB/ttyACM numbers
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="teensy", MODE="0666"
EOF
udevadm control --reload-rules
udevadm trigger

echo "== rosdep"
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || rosdep init
sudo -u "$SUDO_USER" -H rosdep update --rosdistro jazzy >/dev/null

echo "== shell setup"
BASHRC="$USER_HOME/.bashrc"
grep -q "/opt/ros/jazzy/setup.bash" "$BASHRC" || echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC"
grep -q "$WS/install/setup.bash" "$BASHRC" || \
    echo "[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash" >> "$BASHRC"

echo "== build workspace"
sudo -u "$SUDO_USER" -H bash -c "
    source /opt/ros/jazzy/setup.bash
    cd '$WS'
    colcon build --symlink-install
"

echo
echo "Done. Log out and back in (dialout group), then check hardware with:"
echo "  ./scripts/check_hardware.sh"
