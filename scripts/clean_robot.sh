#!/usr/bin/env bash
# Robot-side cleanup: frees the sensors and motor driver, then runs
# clean_sim.sh for the general ROS 2 cleanup (nav2/EKF/RViz leftovers, the
# ros2 daemon, Fast DDS shared memory, core dumps, final verification).
#
# Stops this workspace's nodes, the lidar driver, any `ros2 launch`, and --
# whatever it's called -- any process still holding a sensor device: the
# lidar/Teensy serial port, the IMU's I2C bus, the motor GPIO chip, or the
# OAK-D's USB device. Then drives the motor driver pins low (PWM off,
# driver asleep).
#
# Why: a launch that crashes midway (e.g. a node whose package isn't
# installed) exits without stopping the nodes it had already started. Those
# orphans keep the lidar port and motor GPIO open, so the *next* launch's
# lidar driver dies on startup (watchdog then spams "lidar scan stale") and
# motor_node logs "GPIO busy" and never drives.
#
# Usage (on the Pi, before every launch):
#   scripts/clean_robot.sh
#
# Exit code 0: sensors free and environment clean, safe to launch.
# Exit code 1: something survived SIGKILL or still holds a sensor (e.g.
#              another user's process such as ModemManager grabbing the
#              Teensy -- needs sudo); do not launch, investigate manually.

set -uo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# "$WORKSPACE_ROOT/install/" catches every node built from this workspace
# (motor_node, watchdog_node, wheel_odom_node, bno055_node, sllidar_node, ...).
PATTERN="ros2 launch|sllidar_node|rplidar_composition|component_container|$WORKSPACE_ROOT/install/"

# Motor driver pins (motor_node defaults: PWM 12/13, DIR 5/16, SLEEP 6/19)
# and the RP1 header GPIO chip -- keep in sync with bot_motor/motor_node.py.
GPIO_CHIP=gpiochip4
MOTOR_PINS="12 13 5 16 6 19"

list_pids() {
    # grep -v grep: the pipeline's own grep would otherwise match itself.
    ps aux | grep -E "$PATTERN" | grep -v grep | awk '{print $2}'
}

# Sensor device nodes that only one process can own at a time. Resolved to
# real paths so a process holding /dev/ttyUSB0 matches /dev/rplidar.
sensor_devices() {
    local d bus dev
    for d in /dev/rplidar /dev/teensy /dev/ttyUSB* /dev/ttyACM* /dev/i2c-3 "/dev/$GPIO_CHIP"; do
        [ -e "$d" ] && readlink -f "$d"
    done
    # OAK-D (Movidius, vendor 03e7) -- depthai opens it through libusb
    lsusb -d 03e7: 2>/dev/null | while read -r _ bus _ dev _; do
        echo "/dev/bus/usb/$bus/${dev%:}"
    done
}

# PIDs (other than this script) with any sensor device open. fuser prints
# the PIDs on stdout (device names go to stderr). One call for all devices --
# walking /proc/*/fd per file in bash takes minutes on a desktop session.
list_device_holders() {
    local devs
    devs="$(sensor_devices | sort -u)"
    [ -z "$devs" ] && return
    # shellcheck disable=SC2086
    fuser $devs 2>/dev/null | tr -s ' ' '\n' | grep -E '^[0-9]+$' | grep -vx "$$"
}

all_pids() {
    { list_pids; list_device_holders; } | sort -un
}

# Signals in escalating order. SIGINT first: it's what Ctrl-C sends, and it
# lets motor_node run its shutdown (put the driver to sleep) and watchdog
# assert its fault line, which SIGTERM/SIGKILL skip.
stop_all() {
    local sig pids
    for sig in INT TERM KILL; do
        pids="$(all_pids)"
        [ -z "$pids" ] && return
        echo "Sending SIG$sig to $(echo "$pids" | wc -l) process(es)..."
        echo "$pids" | xargs -r kill -"$sig" 2>/dev/null
        sleep 3
    done
}

echo "Checking for robot nodes and processes holding sensors..."
pids="$(all_pids)"
if [ -z "$pids" ]; then
    echo "None found."
else
    # shellcheck disable=SC2086
    ps -o pid,user,lstart,cmd -p $(echo "$pids" | paste -sd,) | cut -c1-160
    holders="$(list_device_holders)"
    if [ -n "$holders" ]; then
        echo "Of these, holding a sensor device: $(echo "$holders" | paste -sd' ')"
    fi
    stop_all
fi

# A force-killed motor_node leaves its pins as outputs at whatever level they
# had -- possibly PWM high with SLEEP high. Drive them all low (the G2 sleeps
# with SLEEP low). Skipped if something still owns the chip or gpioset is
# missing.
if command -v gpioset >/dev/null && [ -e "/dev/$GPIO_CHIP" ]; then
    args=""
    for p in $MOTOR_PINS; do args="$args $p=0"; done
    # shellcheck disable=SC2086
    if gpioset "$GPIO_CHIP" $args 2>/dev/null; then
        echo "Motor driver pins set low (PWM off, driver asleep)."
    else
        echo "WARNING: could not set motor pins low -- is something still holding $GPIO_CHIP?" >&2
    fi
fi

echo
echo "== General ROS 2 cleanup (clean_sim.sh) =="
"$WORKSPACE_ROOT/scripts/clean_sim.sh"
sim_status=$?

holders="$(list_device_holders)"
if [ -n "$holders" ]; then
    echo "FAILED: still holding a sensor device -- investigate manually (may need sudo):" >&2
    # shellcheck disable=SC2086
    ps -o pid,user,cmd -p $(echo "$holders" | paste -sd,) >&2
    exit 1
fi
exit "$sim_status"
