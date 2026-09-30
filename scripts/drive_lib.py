"""Shared hardware access for the bench/floor drive scripts
(drive_straight.py, rotate.py): motors, wheel encoders and the IMU, used
directly -- no ROS. Same pins and conventions as bot_motor's motor_node,
bot_odometry's wheel_odom_node and bot_imu's bno055_node.

These scripts need exclusive use of the motor GPIO and the Teensy's serial
port, so they refuse to start while the ROS hardware nodes are running.
"""
import math
import subprocess
import sys
import threading
import time

import lgpio
import serial
import smbus2

GPIO_CHIP = 4  # RP1 header GPIO on this Pi, same as motor_node's gpio_chip
PWM_HZ = 10000  # lgpio's software-PWM ceiling, same as motor_node
# Pololu Dual G2 for Raspberry Pi pin map (BCM GPIO): motor 1 = left =
# channel A, motor 2 = right = channel B. FLT low = driver fault.
CHANNELS = {
    "left": {"pwm": 12, "dir": 24, "sleep": 22},
    "right": {"pwm": 13, "dir": 25, "sleep": 23},
}
FAULT_PINS = {"left": 5, "right": 6}
# Same as motor_node's channel_*_inverted (see motor_test.py for history).
INVERTED = {"left": True, "right": False}
MAX_DUTY = 75.0  # same as motor_node's max_duty_cycle: ~12 V (motor rating) from a full 4S pack

# Same as wheel_odom_node's defaults.
TICKS_PER_REV = 979.62
WHEEL_RADIUS_M = 0.062  # tape-calibrated 2026-09-30 (nominal 0.06)
TRACK_WIDTH_M = 0.33  # measured 2026-09-30
M_PER_TICK = 2 * math.pi * WHEEL_RADIUS_M / TICKS_PER_REV

# BNO055 on I2C3 (GPIO14/15), same as hardware.launch.py's imu_i2c_bus
IMU_BUS = 3
IMU_ADDR = 0x28

ROS_HARDWARE_NODES = ("motor_node", "wheel_odom_node", "bno055_node")


def require_hardware_free():
    """Exit with a clear message if the ROS stack holds the motors/Teensy/IMU."""
    running = []
    for name in ROS_HARDWARE_NODES:
        if subprocess.run(["pgrep", "-f", f"lib/bot_[a-z]+/{name}"],
                          stdout=subprocess.DEVNULL).returncode == 0:
            running.append(name)
    if running:
        sys.exit(f"{', '.join(running)} running -- stop the launch first (dashboard Stop, "
                 "or scripts/clean_robot.sh --keep-dashboard). This script drives the "
                 "motors and reads the Teensy directly and can't share them.")


class Encoders(threading.Thread):
    """Latest E,<left>,<right>,<micros> counts from the Teensy. A read error
    is kept in .error (not swallowed) so the control loop can stop."""

    STALE_S = 0.3

    def __init__(self, port="/dev/teensy"):
        super().__init__(daemon=True)
        try:
            self._serial = serial.Serial(port, 115200, timeout=0.5)
        except serial.SerialException as e:
            sys.exit(f"can't open {port}: {e}")
        self._lock = threading.Lock()
        self._left = self._right = None
        self._stamp = 0.0
        self._us = 0  # Teensy micros of the latest counts
        self.error = None
        # Diagnostics: fastest tick rate seen per side (from the Teensy's own
        # micros timestamps), and its once-a-second S status lines
        # (missed quadrature states, raw ADC min/max per pin).
        self._prev = None  # (left, right, teensy_us)
        self.max_rate = [0.0, 0.0]
        self.status_lines = []

    def run(self):
        try:
            while True:
                line = self._serial.readline().decode(errors="ignore").strip()
                parts = line.split(",")
                if len(parts) == 4 and parts[0] == "E":
                    try:
                        left, right, us = int(parts[1]), int(parts[2]), int(parts[3])
                    except ValueError:
                        continue
                    with self._lock:
                        if self._prev and us > self._prev[2]:
                            dt = (us - self._prev[2]) / 1e6
                            for i, (now, before) in enumerate(((left, self._prev[0]), (right, self._prev[1]))):
                                self.max_rate[i] = max(self.max_rate[i], abs(now - before) / dt)
                        self._prev = (left, right, us)
                        self._left, self._right, self._stamp, self._us = left, right, time.monotonic(), us
                elif parts[0] == "S":
                    with self._lock:
                        self.status_lines.append((time.monotonic(), line))
        except Exception as e:  # noqa: BLE001 -- report anything to the main loop
            self.error = f"encoder read failed: {e}"

    def wait_ready(self, timeout=1.5):
        self.start()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.error:
                sys.exit(self.error)
            if self._left is not None:
                return
            time.sleep(0.05)
        sys.exit("no encoder data from the Teensy -- is its firmware running?")

    def diagnostics(self, since):
        """Summary of encoder health since monotonic time `since`, for the end-of-run report."""
        with self._lock:
            lines = [l for t, l in self.status_lines if t >= since]
            rate_l, rate_r = self.max_rate
        out = [f"peak tick rate  left {rate_l:.0f}/s  right {rate_r:.0f}/s "
               f"(~{rate_l * M_PER_TICK:.1f} / {rate_r * M_PER_TICK:.1f} m/s)"]
        fields = [l.split(",") for l in lines]
        fields = [f for f in fields if len(f) >= 4]
        if fields:
            inv_l = int(fields[-1][2]) - int(fields[0][2])
            inv_r = int(fields[-1][3]) - int(fields[0][3])
            out.append(f"missed quadrature states during run  left {inv_l}  right {inv_r}")
            out.append("Teensy status lines (isr_us,left_inv,right_inv,ADC min-max LA,LB,RA,RB):")
            out += [f"    {l}" for l in lines]
        else:
            out.append("no Teensy status (S) lines during the run -- analog-encoder firmware not running?")
        return out

    def read(self):
        """(left, right) ticks; raises RuntimeError if the data is stale or failed."""
        return self.read_stamped()[:2]

    def read_stamped(self):
        """(left, right, teensy_us): the Teensy's own sample time, for speeds
        (USB delivers the lines in bursts, so host read times are jittery)."""
        with self._lock:
            left, right, stamp, us = self._left, self._right, self._stamp, self._us
        if self.error:
            raise RuntimeError(self.error)
        if time.monotonic() - stamp > self.STALE_S:
            raise RuntimeError("encoder data stopped arriving")
        return left, right, us


class Imu:
    """BNO055 in IMUPLUS mode (gyro + accel fusion, no magnetometer -- the
    motors would disturb it). yaw_deg() is counter-clockwise-positive
    relative to where it was at start(), unwrapped past +-180."""

    def __init__(self):
        try:
            self._bus = smbus2.SMBus(IMU_BUS)
            if self._bus.read_byte_data(IMU_ADDR, 0x00) != 0xA0:
                raise OSError("wrong chip id")
            self._bus.write_byte_data(IMU_ADDR, 0x3D, 0x00)  # CONFIG mode
            time.sleep(0.03)
            self._bus.write_byte_data(IMU_ADDR, 0x3E, 0x00)  # normal power
            self._bus.write_byte_data(IMU_ADDR, 0x3B, 0x00)  # units: deg, deg/s
            self._bus.write_byte_data(IMU_ADDR, 0x3D, 0x08)  # IMUPLUS fusion
            time.sleep(0.1)
        except OSError as e:
            sys.exit(f"BNO055 not answering on I2C-{IMU_BUS} at 0x{IMU_ADDR:02x} ({e}) -- "
                     "i2c group? wiring? (scripts/check_hardware.sh)")
        self._last_heading = None
        self._yaw = 0.0

    def _i16(self, reg):
        lo, hi = self._bus.read_i2c_block_data(IMU_ADDR, reg, 2)
        v = lo | hi << 8
        return v - 0x10000 if v & 0x8000 else v

    def gyro_z_dps(self):
        """Yaw rate, counter-clockwise positive (chip Z is up)."""
        return self._i16(0x18) / 16.0

    def start(self):
        self._last_heading = self._i16(0x1A) / 16.0
        self._yaw = 0.0

    def yaw_deg(self):
        # BNO055 heading is compass-style (clockwise positive, 0..360)
        heading = self._i16(0x1A) / 16.0
        delta = (heading - self._last_heading + 180.0) % 360.0 - 180.0
        self._last_heading = heading
        self._yaw -= delta
        return self._yaw


class Motors:
    """Context manager: claims the G2's pins, always stops and sleeps the
    driver on exit (normal, Ctrl-C or error)."""

    def __enter__(self):
        self._h = lgpio.gpiochip_open(GPIO_CHIP)
        try:
            for ch in CHANNELS.values():
                for pin in ch.values():
                    lgpio.gpio_claim_output(self._h, pin, 0)
            for pin in FAULT_PINS.values():
                lgpio.gpio_claim_input(self._h, pin, lgpio.SET_PULL_UP)
        except lgpio.error as e:
            lgpio.gpiochip_close(self._h)
            sys.exit(f"can't claim motor GPIO ({e}) -- is motor_node still running?")
        for ch in CHANNELS.values():
            lgpio.gpio_write(self._h, ch["sleep"], 1)
        # The G2 pulls FLT low for ~1 ms while waking from sleep (measured
        # 2026-09-29) -- let that pass before anything reads faults.
        time.sleep(0.01)
        self._fault_count = {side: 0 for side in FAULT_PINS}
        return self

    def set(self, side, duty):
        """duty in % (-MAX_DUTY..MAX_DUTY), positive = that wheel forward."""
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        ch = CHANNELS[side]
        lgpio.gpio_write(self._h, ch["dir"], 1 if (duty >= 0) != INVERTED[side] else 0)
        lgpio.tx_pwm(self._h, ch["pwm"], PWM_HZ, abs(duty))

    FAULT_READS = 3  # consecutive low reads (~60 ms at 50 Hz) before a fault counts

    def faults(self):
        """Sides whose FLT has been low for FAULT_READS calls in a row."""
        for side, pin in FAULT_PINS.items():
            low = lgpio.gpio_read(self._h, pin) == 0
            self._fault_count[side] = self._fault_count[side] + 1 if low else 0
        return [s for s, n in self._fault_count.items() if n >= self.FAULT_READS]

    def fault_follow_up(self, window_s=1.0):
        """Call right after stop() on a fault abort: watches FLT with the
        driver still awake. Clearing on its own (within ms) points at a
        supply dip -- under-voltage lockout, e.g. the e-stop relay dropping
        out or wiring sag -- while staying low points at over-current or
        over-temperature."""
        start = time.monotonic()
        while time.monotonic() - start < window_s:
            low = [s for s, pin in FAULT_PINS.items() if lgpio.gpio_read(self._h, pin) == 0]
            if not low:
                return (f"FLT cleared {1000 * (time.monotonic() - start):.0f} ms after stopping -> "
                        "likely a supply dip (under-voltage: e-stop relay dropout or wiring sag under load)")
            time.sleep(0.002)
        return (f"FLT still low on {', '.join(low)} {window_s:.0f} s after stopping -> "
                "latched fault: over-current (short) or over-temperature")

    def stop(self):
        for ch in CHANNELS.values():
            lgpio.tx_pwm(self._h, ch["pwm"], PWM_HZ, 0)
            lgpio.gpio_write(self._h, ch["pwm"], 0)

    def __exit__(self, *exc):
        self.stop()
        for ch in CHANNELS.values():
            lgpio.gpio_write(self._h, ch["sleep"], 0)
        lgpio.gpiochip_close(self._h)
        return False


def confirm(prompt, assume_yes):
    if assume_yes:
        return
    if input(f"{prompt} Path clear, e-stop in hand? [y/N] ").strip().lower() != "y":
        sys.exit("aborted")
