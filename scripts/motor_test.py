#!/usr/bin/env python3
"""Bench test: do the drive motors run, and do the encoders agree?

Runs each side briefly at low power -- left forward, left reverse, right
forward, right reverse -- and reads the Teensy's encoder counts before and
after each step. Reports whether the wheel turned, whether the matching
encoder saw it (left/right not crossed), and whether "forward" on the motor
driver counts positive, which wheel_odom_node assumes.

Drives the Pololu G2 directly over GPIO with the same pins and conventions
as bot_motor's motor_node (channel A = left, B = right, DIR high =
forward), so it bypasses ROS and the software watchdog entirely. The
wireless e-stop is the only cutoff: WHEELS OFF THE GROUND, e-stop in hand.
Motors are stopped and the driver put to sleep on exit, Ctrl-C or error.

Nothing else may hold the motor GPIO or the Teensy's serial port -- stop
any launch first (scripts/clean_robot.sh).

Usage:
    scripts/motor_test.py              # 25% power, 1 s per step
    scripts/motor_test.py --duty 40 --seconds 2
"""
import argparse
import sys
import threading
import time

import lgpio
import serial

GPIO_CHIP = 4  # RP1 header GPIO on this Pi, same as motor_node's gpio_chip
PWM_HZ = 10000  # lgpio's software-PWM ceiling, same as motor_node
# Pololu Dual G2 for Raspberry Pi pin map (BCM GPIO), same as motor_node:
# motor 1 = left = channel A, motor 2 = right = channel B. FLT is the
# driver's open-drain fault output, read with a pull-up (low = fault).
CHANNELS = {
    "left": {"pwm": 12, "dir": 24, "sleep": 22},
    "right": {"pwm": 13, "dir": 25, "sleep": 23},
}
FAULT_PINS = {"left": 5, "right": 6}
# Same as motor_node's channel_*_inverted: the motors are mounted mirror-image,
# so the right one needs the opposite DIR level to drive the robot forward.
INVERTED = {"left": False, "right": True}
# A moving wheel easily gives hundreds of counts per second (979.62 per
# wheel turn); fewer than this during a step means it didn't really turn.
MIN_COUNTS_TO_COUNT_AS_MOVING = 20


class EncoderReader(threading.Thread):
    """Keeps the latest E,<left>,<right>,<micros> counts from the Teensy."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self._serial = serial.Serial(port, 115200, timeout=0.5)
        self.left = self.right = None
        self.lines = 0

    def run(self):
        while True:
            line = self._serial.readline().decode(errors="ignore").strip()
            parts = line.split(",")
            if len(parts) == 4 and parts[0] == "E":
                try:
                    self.left, self.right = int(parts[1]), int(parts[2])
                    self.lines += 1
                except ValueError:
                    pass

    def counts(self):
        return self.left, self.right


def stop_all(h):
    for ch in CHANNELS.values():
        lgpio.tx_pwm(h, ch["pwm"], PWM_HZ, 0)
        lgpio.gpio_write(h, ch["pwm"], 0)
        lgpio.gpio_write(h, ch["sleep"], 0)  # G2 sleeps with SLEEP low


def run_step(h, enc, side, forward, duty, seconds):
    ch = CHANNELS[side]
    before = enc.counts()
    lgpio.gpio_write(h, ch["dir"], 1 if forward != INVERTED[side] else 0)
    lgpio.gpio_write(h, ch["sleep"], 1)
    lgpio.tx_pwm(h, ch["pwm"], PWM_HZ, duty)
    time.sleep(seconds)
    lgpio.tx_pwm(h, ch["pwm"], PWM_HZ, 0)
    lgpio.gpio_write(h, ch["sleep"], 0)
    time.sleep(0.5)  # let it coast to a stop and the counts settle
    after = enc.counts()
    return after[0] - before[0], after[1] - before[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--duty", type=float, default=25.0, help="PWM duty %% (default 25)")
    ap.add_argument("--seconds", type=float, default=1.0, help="run time per step (default 1)")
    ap.add_argument("--port", default="/dev/teensy", help="Teensy serial port")
    ap.add_argument("--yes", action="store_true", help="skip the wheels-off-the-ground prompt")
    args = ap.parse_args()
    if not 0 < args.duty <= 60:
        sys.exit("--duty must be between 0 and 60 for a bench test")

    enc = EncoderReader(args.port)
    enc.start()
    time.sleep(1.0)
    if enc.lines == 0:
        sys.exit(f"no encoder data from {args.port} -- is the Teensy firmware running?")

    if not args.yes:
        answer = input("Wheels OFF the ground and e-stop in hand? [y/N] ")
        if answer.strip().lower() != "y":
            sys.exit("aborted")

    h = lgpio.gpiochip_open(GPIO_CHIP)
    try:
        for ch in CHANNELS.values():
            for pin in ch.values():
                lgpio.gpio_claim_output(h, pin, 0)
        for pin in FAULT_PINS.values():
            lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_UP)
    except lgpio.error as e:
        lgpio.gpiochip_close(h)
        sys.exit(f"can't claim motor GPIO ({e}) -- is motor_node running (try scripts/clean_robot.sh)? "
                 "Or is GPIO22/23 still the IMU's I2C bus (move the IMU, see README)?")

    results = []
    try:
        for side in ("left", "right"):
            for forward in (True, False):
                label = f"{side} {'forward' if forward else 'reverse'}"
                print(f"running {label} at {args.duty:.0f}% for {args.seconds:.1f} s ...", flush=True)
                d_left, d_right = run_step(h, enc, side, forward, args.duty, args.seconds)
                results.append((side, forward, d_left, d_right))
                faults = [s for s, pin in FAULT_PINS.items() if lgpio.gpio_read(h, pin) == 0]
                print(f"    encoder change: left {d_left:+d}, right {d_right:+d}"
                      + (f"   DRIVER FAULT on {', '.join(faults)}" if faults else ""))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        stop_all(h)
        lgpio.gpiochip_close(h)
        print("motors stopped, driver asleep")

    print("\nSummary")
    problems = 0
    for side, forward, d_left, d_right in results:
        own, other = (d_left, d_right) if side == "left" else (d_right, d_left)
        label = f"{side} {'forward' if forward else 'reverse'}"
        if abs(own) < MIN_COUNTS_TO_COUNT_AS_MOVING:
            if abs(other) >= MIN_COUNTS_TO_COUNT_AS_MOVING:
                msg = "the OTHER side's encoder moved -- left/right crossed (motor or encoder wiring)"
            else:
                msg = ("no encoder movement -- motor not running (e-stop active? driver unpowered?) "
                       "or encoder not counting")
            problems += 1
        elif (own > 0) != forward:
            msg = (f"runs, but counts {'down' if forward else 'up'} -- motor or encoder direction "
                   "reversed relative to the other (swap motor leads, or A/B in teensy_ws)")
            problems += 1
        else:
            msg = f"OK ({abs(own)} counts, ~{abs(own) / 979.62:.2f} wheel turns)"
        print(f"  {label:14s} {msg}")
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
