#!/usr/bin/env python3
"""Pivot the robot in place by a set angle (default 360 deg), measured by the IMU.

Two phases:
  1. Breakaway: ramps PWM up (both sides, opposite directions) from
     --start-duty until the IMU sees the robot actually turning, and
     reports that duty -- the real minimum power this robot needs to pivot
     on this floor. Use it to set motor_node's min_turn_duty_cycle.
  2. Turn: holds max(breakaway duty, --duty) until close to the target,
     eases back to the breakaway duty for the last --slow-zone degrees,
     then stops.

The angle comes from the BNO055 (gyro + accel fusion, no magnetometer),
not the wheels -- skid-steer wheels slip sideways when pivoting, so wheel
encoders overstate the turn. The encoder estimate is printed alongside;
the ratio is the slip.

Drives the motors and reads the Teensy/IMU directly (no ROS) -- stop any
launch first. THE ROBOT WILL MOVE: clear space around it, e-stop in hand.
Aborts on driver fault, encoder loss, timeout or Ctrl-C; always stops the
motors on exit.

Usage:
    scripts/rotate.py                    # 360 deg counter-clockwise (left)
    scripts/rotate.py --right            # clockwise
    scripts/rotate.py --angle 90 --duty 60
"""
import argparse
import math
import sys
import time

import drive_lib as dl

LOOP_HZ = 50
MOVING_DPS = 15.0  # the IMU must see at least this yaw rate...
MOVING_HOLD_S = 0.2  # ...for this long to count as "turning"
RAMP_RATE = 15.0  # % duty per second while looking for breakaway
MAX_BREAKAWAY_WAIT_S = 1.0  # at MAX_DUTY with no rotation for this long = give up
SETTLE_S = 0.7


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--angle", type=float, default=360.0, help="degrees to turn (default 360)")
    ap.add_argument("--right", action="store_true", help="turn clockwise (default counter-clockwise)")
    ap.add_argument("--duty", type=float, default=60.0,
                    help="cruise PWM %% once turning, if above the breakaway duty (default 60)")
    ap.add_argument("--start-duty", type=float, default=30.0, help="breakaway ramp starts here (default 30)")
    ap.add_argument("--slow-zone", type=float, default=30.0,
                    help="degrees before the target to drop to the breakaway duty (default 30)")
    ap.add_argument("--timeout", type=float, default=20.0, help="give up after this many seconds (default 20)")
    ap.add_argument("--yes", action="store_true", help="skip the safety prompt")
    args = ap.parse_args()
    if not 0 < args.angle <= 1080:
        sys.exit("--angle must be between 0 and 1080")

    dl.require_hardware_free()
    enc = dl.Encoders()
    enc.wait_ready()
    imu = dl.Imu()
    dl.confirm(f"Robot will pivot {args.angle:.0f} deg {'clockwise' if args.right else 'counter-clockwise'}.",
               args.yes)

    turn = -1 if args.right else 1  # +1 = CCW: left wheel back, right wheel forward
    l0, r0 = enc.read()
    imu.start()
    breakaway = None
    moving_since = None
    at_max_since = None
    duty = args.start_duty
    reason = "reached target angle"
    start = time.monotonic()
    rates = []

    with dl.Motors() as motors:
        try:
            while True:
                loop_start = time.monotonic()
                t = loop_start - start
                enc.read()  # raises if the encoders stopped
                yaw = turn * imu.yaw_deg()  # progress in the commanded direction
                rate = turn * imu.gyro_z_dps()

                if yaw >= args.angle - 2.0:  # coasting covers the last couple of degrees
                    break
                if t > args.timeout:
                    reason = f"ABORTED: timeout after {args.timeout:.0f} s at {yaw:.0f} deg"
                    break
                faults = motors.faults()
                if faults:
                    reason = f"ABORTED: motor driver fault on {', '.join(faults)} (over-current?)"
                    break

                if breakaway is None:
                    # Phase 1: ramp until the IMU sees a real, sustained turn
                    if rate >= MOVING_DPS:
                        moving_since = moving_since or loop_start
                        if loop_start - moving_since >= MOVING_HOLD_S:
                            breakaway = duty
                            print(f"turning at {duty:.0f}% duty ({rate:.0f} deg/s) -- breakaway duty")
                    else:
                        moving_since = None
                    if breakaway is None:
                        duty = min(dl.MAX_DUTY, duty + RAMP_RATE / LOOP_HZ)
                        if duty >= dl.MAX_DUTY:
                            at_max_since = at_max_since or loop_start
                            if loop_start - at_max_since > MAX_BREAKAWAY_WAIT_S:
                                reason = (f"ABORTED: no rotation even at {dl.MAX_DUTY:.0f}% duty -- not a "
                                          "software limit; check battery voltage under load, the e-stop "
                                          "relay, and wheel traction")
                                break
                else:
                    # Phase 2: cruise, then ease off near the target
                    cruise = max(breakaway, args.duty)
                    remaining = args.angle - yaw
                    duty = breakaway if remaining < args.slow_zone else cruise
                    rates.append(rate)

                motors.set("left", -turn * duty)
                motors.set("right", turn * duty)
                time.sleep(max(0.0, 1 / LOOP_HZ - (time.monotonic() - loop_start)))
        except KeyboardInterrupt:
            reason = "ABORTED: Ctrl-C"
        except RuntimeError as e:
            reason = f"ABORTED: {e}"
        fault_note = None
        if reason.startswith("ABORTED: motor driver fault"):
            motors.stop()
            fault_note = motors.fault_follow_up()
    elapsed = time.monotonic() - start

    time.sleep(SETTLE_S)
    yaw = turn * imu.yaw_deg()
    try:
        left, right = enc.read()
        enc_deg = turn * math.degrees(((right - r0) - (left - l0)) * dl.M_PER_TICK / dl.TRACK_WIDTH_M)
    except RuntimeError:
        enc_deg = None

    print(f"\n{reason}  ({elapsed:.1f} s, motors stopped, driver asleep)")
    print(f"  turned (IMU)       {yaw:7.1f} deg   (target {args.angle:.0f})")
    if enc_deg is not None:
        slip = f"   -> wheels slip ~{(1 - yaw / enc_deg) * 100:.0f}%" if enc_deg > 10 else ""
        print(f"  turned (encoders)  {enc_deg:7.1f} deg{slip}")
    if breakaway is not None:
        print(f"  breakaway duty     {breakaway:7.0f} %     (set motor_node min_turn_duty_cycle "
              f"to ~{min(breakaway + 5, dl.MAX_DUTY) / 100:.2f})")
    if rates:
        print(f"  average rate       {sum(rates) / len(rates):7.0f} deg/s")
    if fault_note:
        print(f"  driver fault: {fault_note}")
    print("  encoder diagnostics:")
    for line in enc.diagnostics(start):
        print(f"    {line}")
    sys.exit(0 if reason == "reached target angle" else 1)


if __name__ == "__main__":
    main()
