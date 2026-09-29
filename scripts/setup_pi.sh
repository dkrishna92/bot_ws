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
# On this Ubuntu raspi image, /dev/gpiochip4 and /dev/ttyUSB*/ttyACM* are
# group dialout. /dev/i2c-* start out dialout too, but i2c-tools (installed
# above) ships a udev rule that moves them to group i2c -- without that
# group bno055_node gets "Permission denied" on /dev/i2c-3.
usermod -aG dialout,plugdev,i2c "$SUDO_USER"

cat > /etc/udev/rules.d/80-racebot.rules <<'EOF'
# OAK-D S2 (Movidius MyriadX; re-enumerates with a different PID after boot)
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
# Stable names so the lidar and Teensy can't swap ttyUSB/ttyACM numbers
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="teensy", MODE="0666"
# E-stop Nano ESP32, left on the Pi's USB: running firmware (2341:0070)
# and its ROM loader (303a:1001). ModemManager must never probe it -- a
# DTR/RTS toggle like esptool's can reboot it into download mode.
ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0070", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0070", SYMLINK+="estop"
# It can power up stuck in ROM download mode (see scripts/estop_recover.py);
# whenever the loader appears, estop-recover.service waits 3 s and kicks
# it into its firmware if it's still there.
ACTION=="add", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="303a", ATTR{idProduct}=="1001", TAG+="systemd", ENV{SYSTEMD_WANTS}+="estop-recover.service"
EOF

cat > /etc/systemd/system/estop-recover.service <<EOF
[Unit]
Description=Boot the e-stop Nano ESP32 out of ROM download mode if it powered up stuck there

[Service]
Type=oneshot
ExecStart=/usr/bin/python3 $WS/scripts/estop_recover.py --wait 3
EOF
systemctl daemon-reload

# PJRC's Teensy rules (https://www.pjrc.com/teensy/00-teensy.rules): lets a
# normal user flash the Teensy (teensy_loader_cli talks to the bootloader
# and the running sketch over raw USB/hidraw, root-only by default), and
# keeps ModemManager from probing the Teensy's /dev/ttyACM port.
cat > /etc/udev/rules.d/00-teensy.rules <<'EOF'
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04[789B]?", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04[789A]?", ENV{MTP_NO_PROBE}="1"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04[789ABCD]?", MODE:="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04[789B]?", MODE:="0666"
EOF
udevadm control --reload-rules
udevadm trigger

echo "== I2C3 for the IMU"
# BNO055 lives on I2C3 routed to GPIO14/15 (header pins 8 SDA / 10 SCL):
# the default I2C1 on GPIO2/3 times out on this Pi even with nothing
# attached, and GPIO22/23 belong to the Pololu G2 motor HAT (motor SLP).
# Takes effect after a reboot.
CONFIG=/boot/firmware/config.txt
if ! grep -q "^dtoverlay=i2c3-pi5,pins_14_15" "$CONFIG"; then
    cp "$CONFIG" "$CONFIG.bak.$(date +%Y%m%d%H%M%S)"
    # Drop the old routing onto the motor HAT's SLP pins, if present
    sed -i '/^dtoverlay=i2c3-pi5,pins_22_23/d' "$CONFIG"
    # Append under an explicit [all] so it isn't caught by a [pi4]/[cm4] section
    printf '\n[all]\ndtoverlay=i2c3-pi5,pins_14_15\n' >> "$CONFIG"
    echo "  I2C3 overlay set to GPIO14/15 -- REBOOT required"
fi

echo "== headless boot"
# No desktop on the race Pi: GNOME costs ~300 MB and background CPU the
# Nav2 stack needs. SSH and the web dashboard don't depend on it. Undo
# with 'systemctl set-default graphical.target'.
if [ "$(systemctl get-default)" != "multi-user.target" ]; then
    systemctl set-default multi-user.target
    echo "  default boot target set to multi-user (console only)"
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
