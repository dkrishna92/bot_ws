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

echo "== I2C3 for the IMU"
# BNO055 lives on I2C3 (GPIO22/23, header pins 15/16): the default I2C1 on
# GPIO2/3 times out on this Pi even with nothing attached. Takes effect
# after a reboot.
CONFIG=/boot/firmware/config.txt
if ! grep -q "^dtoverlay=i2c3-pi5,pins_22_23" "$CONFIG"; then
    cp "$CONFIG" "$CONFIG.bak.$(date +%Y%m%d%H%M%S)"
    # Append under an explicit [all] so it isn't caught by a [pi4]/[cm4] section
    printf '\n[all]\ndtoverlay=i2c3-pi5,pins_22_23\n' >> "$CONFIG"
    echo "  added I2C3 overlay -- REBOOT required"
fi

echo "== rosdep"
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || rosdep init
sudo -u "$SUDO_USER" -H rosdep update --rosdistro jazzy >/dev/null

echo "== shell setup"
BASHRC="$USER_HOME/.bashrc"
grep -q "/opt/ros/jazzy/setup.bash" "$BASHRC" || echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC"
grep -q "$WS/install/setup.bash" "$BASHRC" || \
    echo "[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash" >> "$BASHRC"

echo "== lidar driver (RPLIDAR S2 needs Slamtec's sllidar_ros2 -- not in apt)"
SLLIDAR_COMMIT=34300099fadfc772965962dec837bf436706188f
if [[ ! -d "$WS/src/sllidar_ros2/.git" ]]; then
    sudo -u "$SUDO_USER" git clone -q https://github.com/Slamtec/sllidar_ros2.git "$WS/src/sllidar_ros2"
fi
sudo -u "$SUDO_USER" git -C "$WS/src/sllidar_ros2" fetch -q origin
sudo -u "$SUDO_USER" git -C "$WS/src/sllidar_ros2" checkout -q "$SLLIDAR_COMMIT"

echo "== build workspace"
sudo -u "$SUDO_USER" -H bash -c "
    source /opt/ros/jazzy/setup.bash
    cd '$WS'
    colcon build --symlink-install
"

echo
echo "Done. Reboot if the I2C3 overlay was just added (otherwise log out and"
echo "back in for the dialout group), then check hardware with:"
echo "  ./scripts/check_hardware.sh"
