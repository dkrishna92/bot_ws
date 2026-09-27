#!/usr/bin/env bash
# Race-day preflight: is every device the hardware launch expects actually
# present and accessible? Run as your normal user after scripts/setup_pi.sh.
# Pass --topics while bringup.launch.py is running to also check data flow.
WS="$(cd "$(dirname "$0")/.." && pwd)"
fail=0
ok()   { printf '  \e[32mOK\e[0m   %s\n' "$1"; }
bad()  { printf '  \e[31mFAIL\e[0m %s\n' "$1"; fail=1; }

echo "Devices"
[[ -e /dev/rplidar ]] && ok "RPLIDAR at /dev/rplidar -> $(readlink -f /dev/rplidar)" || bad "RPLIDAR (/dev/rplidar) missing -- USB plugged in? udev rule installed?"
[[ -e /dev/teensy ]]  && ok "Teensy at /dev/teensy -> $(readlink -f /dev/teensy)"   || bad "Teensy (/dev/teensy) missing -- no odometry/ultrasonics without it"
lsusb -d 03e7: >/dev/null && ok "OAK-D on USB ($(lsusb -d 03e7: | cut -d' ' -f6-))" || bad "OAK-D not on USB"
# BNO055 is on I2C3 (GPIO22/23) -- see hardware.launch.py's imu_i2c_bus
if [[ -r /dev/i2c-3 ]] && command -v i2cget >/dev/null; then
    # BNO055 CHIP_ID register 0x00 reads 0xA0
    id=$(i2cget -y 3 0x28 0x00 2>/dev/null || i2cget -y 3 0x29 0x00 2>/dev/null)
    [[ "$id" == "0xa0" ]] && ok "BNO055 on I2C-3 (chip id 0xA0)" || bad "BNO055 not answering on I2C-3 at 0x28/0x29 (SDA pin 15, SCL pin 16)"
elif [[ ! -e /dev/i2c-3 ]]; then
    bad "/dev/i2c-3 missing -- add dtoverlay=i2c3-pi5,pins_22_23 to /boot/firmware/config.txt and reboot"
else
    bad "/dev/i2c-3 not readable (dialout group? log out/in after setup)"
fi
[[ -r /dev/gpiochip4 && -w /dev/gpiochip4 ]] && ok "gpiochip4 (RP1 header GPIO) accessible" || bad "gpiochip4 not accessible (dialout group?)"
python3 -c "import lgpio, smbus2, serial" 2>/dev/null && ok "python lgpio/smbus2/pyserial importable" || bad "python deps missing -- rerun setup_pi.sh"

echo "Workspace"
[[ -f /opt/ros/jazzy/setup.bash ]] && ok "ROS 2 Jazzy installed" || bad "ROS 2 Jazzy not installed"
[[ -f "$WS/install/setup.bash" ]] && ok "bot_ws built" || bad "bot_ws not built (colcon build --symlink-install)"
[[ -s "$WS/src/bot_bringup/config/maps/map.yaml" ]] && ok "race map present (config/maps/map.yaml)" || bad "no race map -- run mapping.launch.py on the course"

if [[ "${1:-}" == "--topics" ]]; then
    echo "Topics (bringup.launch.py must be running)"
    source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
    for t in /scan /odom /imu /oak/points /odometry/filtered; do
        if timeout 6 ros2 topic hz "$t" --window 20 2>/dev/null | grep -m1 -q "average rate"; then ok "$t publishing"; else bad "$t not publishing"; fi
    done
fi

echo
[[ $fail -eq 0 ]] && echo "All checks passed." || { echo "Some checks FAILED."; exit 1; }
