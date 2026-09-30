#!/usr/bin/env python3
"""Drive the robot in a straight line for a set distance.

Heading is held with the IMU (BNO055 gyro + accel fusion): if the robot
yaws left, left duty rises and right duty falls, and vice versa. Distance
comes from the wheel encoders (average of both sides).

Starts by ramping PWM up from --start-duty until the encoders show the
robot actually rolling (and reports that breakaway duty), then runs at
max(breakaway, --duty), easing back over the last 15 cm.

At the end it reports each side's distance, the heading change (IMU),
the sideways drift (dead-reckoned from encoder distance + IMU heading),
and the average duty each side needed -- a big left/right gap means the
motors or wheels are unevenly matched. --open-loop drives both sides at
the same duty with no correction, to see the natural drift.

Drives the motors and reads the Teensy/IMU directly (no ROS) -- stop any
launch first. THE ROBOT WILL MOVE: clear path, e-stop in hand. Aborts on
driver fault, encoder loss, stalled wheel, timeout or Ctrl-C; always stops
the motors on exit.

Usage:
    scripts/drive_straight.py                      # 1.0 m forward
    scripts/drive_straight.py --distance 2 --duty 60
    scripts/drive_straight.py --reverse --distance 0.5
    scripts/drive_straight.py --open-loop
"""
import argparse
import math
import sys
import time

import drive_lib as dl

LOOP_HZ = 50
MOVING_TICKS_PER_S = 150  # both wheels at least this fast = rolling (~0.06 m/s)
RAMP_RATE = 15.0  # % duty per second while looking for breakaway
MAX_BREAKAWAY_WAIT_S = 1.0
RAMP_DOWN_M = 0.15  # slow down over at least this, or 0.5 s at cruise speed if longer
BRAKE_S = 0.5  # hold PWM low with the driver awake (G2 brake mode) before it sleeps
# Braking deceleration for the stop-early prediction: a 2026-09-30 run
# braked 0.15 m from ~1.2 m/s (~4.7 m/s^2). A little low = stops a touch short.
BRAKE_DECEL = 4.0
STALL_WINDOW_S = 0.5
STALL_MIN_TICKS = 20  # a wheel moving fewer ticks than this per window after breakaway = stalled
SETTLE_S = 0.7


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--distance", type=float, default=1.0, help="metres to drive (default 1.0)")
    ap.add_argument("--duty", type=float, default=50.0,
                    help="cruise PWM %% once rolling, if above the breakaway duty (default 50)")
    ap.add_argument("--start-duty", type=float, default=25.0, help="breakaway ramp starts here (default 25)")
    ap.add_argument("--kp", type=float, default=3.0,
                    help="duty %% of correction per degree of heading error (default 3)")
    ap.add_argument("--open-loop", action="store_true", help="no heading correction")
    ap.add_argument("--reverse", action="store_true", help="drive backwards")
    ap.add_argument("--timeout", type=float, default=20.0, help="give up after this many seconds (default 20)")
    ap.add_argument("--yes", action="store_true", help="skip the safety prompt")
    args = ap.parse_args()
    if not 0 < args.distance <= 10:
        sys.exit("--distance must be between 0 and 10 m")

    dl.require_hardware_free()
    enc = dl.Encoders()
    enc.wait_ready()
    imu = dl.Imu()
    dl.confirm(f"Robot will drive {args.distance:.2f} m {'BACKWARD' if args.reverse else 'forward'}.", args.yes)

    sign = -1 if args.reverse else 1
    l0, r0 = enc.read()
    prev_l, prev_r = l0, r0
    imu.start()
    x = y = 0.0
    history = []  # (t, left progress, right progress)
    breakaway = None
    duty = args.start_duty
    at_max_since = None
    duty_sum = {"left": 0.0, "right": 0.0}
    samples = 0
    cruise_speeds = []  # m/s over the stall window while at cruise duty
    speed = 0.0
    dist = 0.0
    reason = "reached target distance"
    start = time.monotonic()

    with dl.Motors() as motors:
        try:
            while True:
                loop_start = time.monotonic()
                t = loop_start - start
                left, right = enc.read()
                yaw = imu.yaw_deg()

                pl, pr = sign * (left - l0), sign * (right - r0)
                dist = (pl + pr) / 2 * dl.M_PER_TICK
                ds = sign * ((left - prev_l) + (right - prev_r)) / 2 * dl.M_PER_TICK
                prev_l, prev_r = left, right
                x += ds * math.cos(math.radians(yaw))
                y += ds * math.sin(math.radians(yaw))

                # Cut power early by the braking distance, so it stops AT the target
                if dist + speed * speed / (2 * BRAKE_DECEL) >= args.distance:
                    break
                if t > args.timeout:
                    reason = f"ABORTED: timeout after {args.timeout:.0f} s at {dist:.2f} m"
                    break
                faults = motors.faults()
                if faults:
                    reason = f"ABORTED: motor driver fault on {', '.join(faults)} (over-current?)"
                    break

                history.append((t, pl, pr))
                while history and history[0][0] < t - STALL_WINDOW_S:
                    history.pop(0)
                span = t - history[0][0]
                moved_l, moved_r = pl - history[0][1], pr - history[0][2]

                if breakaway is None:
                    # Ramp until both wheels are really rolling
                    if span > 0.2 and min(moved_l, moved_r) / span >= MOVING_TICKS_PER_S:
                        breakaway = duty
                        print(f"rolling at {duty:.0f}% duty -- breakaway duty")
                    else:
                        duty = min(dl.MAX_DUTY, duty + RAMP_RATE / LOOP_HZ)
                        if duty >= dl.MAX_DUTY:
                            at_max_since = at_max_since or loop_start
                            if loop_start - at_max_since > MAX_BREAKAWAY_WAIT_S:
                                slow = "left" if moved_l < moved_r else "right"
                                reason = (f"ABORTED: not rolling even at {dl.MAX_DUTY:.0f}% duty ({slow} "
                                          "side slowest) -- check battery voltage under load, e-stop "
                                          "relay, wheels")
                                break
                else:
                    if span >= STALL_WINDOW_S * 0.9 and min(moved_l, moved_r) < STALL_MIN_TICKS:
                        side = "left" if moved_l < moved_r else "right"
                        reason = f"ABORTED: {side} wheel stalled (e-stop cut power? blocked?)"
                        break
                    duty = max(breakaway, args.duty)
                    remaining = args.distance - dist
                    # Speed over the last ~0.2 s (the 0.5 s stall window lags when slowing)
                    ref = next((h for h in reversed(history) if t - h[0] >= 0.2), None)
                    if ref is not None:
                        speed = ((pl - ref[1]) + (pr - ref[2])) / 2 * dl.M_PER_TICK / (t - ref[0])
                    ramp_m = max(RAMP_DOWN_M, 0.5 * speed)
                    if remaining < ramp_m:
                        duty = breakaway + (duty - breakaway) * remaining / ramp_m
                    elif span >= STALL_WINDOW_S * 0.9:
                        cruise_speeds.append(speed)

                # Yawed left (yaw > 0): speed up left / slow right to turn back.
                # Driving backwards the same wheel-speed difference turns the
                # other way, so the correction flips with the direction.
                corr = 0.0 if args.open_loop else sign * args.kp * yaw
                duty_l = max(0.0, min(dl.MAX_DUTY, duty + corr))
                duty_r = max(0.0, min(dl.MAX_DUTY, duty - corr))
                motors.set("left", sign * duty_l)
                motors.set("right", sign * duty_r)
                if breakaway is not None:
                    duty_sum["left"] += duty_l
                    duty_sum["right"] += duty_r
                    samples += 1

                time.sleep(max(0.0, 1 / LOOP_HZ - (time.monotonic() - loop_start)))
        except KeyboardInterrupt:
            reason = "ABORTED: Ctrl-C"
        except RuntimeError as e:
            reason = f"ABORTED: {e}"
        dist_at_stop, speed_at_stop = dist, speed
        motors.stop()
        fault_note = None
        if reason.startswith("ABORTED: motor driver fault"):
            fault_note = motors.fault_follow_up()
        else:
            time.sleep(BRAKE_S)  # brake like motor_node does at zero cmd_vel, not coast
    elapsed = time.monotonic() - start

    time.sleep(SETTLE_S)
    yaw = imu.yaw_deg()
    print(f"\n{reason}  ({elapsed:.1f} s, motors stopped, driver asleep)")
    try:
        left, right = enc.read()
        d_l, d_r = sign * (left - l0) * dl.M_PER_TICK, sign * (right - r0) * dl.M_PER_TICK
        print(f"  left wheel      {d_l:6.3f} m   ({sign * (left - l0):+d} ticks)")
        print(f"  right wheel     {d_r:6.3f} m   ({sign * (right - r0):+d} ticks)")
        print(f"  average         {(d_l + d_r) / 2:6.3f} m   (target {args.distance:.3f} m)")
        braked = (d_l + d_r) / 2 - dist_at_stop
        print(f"  at motor stop   {dist_at_stop:6.3f} m at {speed_at_stop:.2f} m/s -> {braked:.3f} m while braking"
              + (f" ({speed_at_stop ** 2 / (2 * braked):.1f} m/s^2)" if braked > 0.01 and speed_at_stop > 0.2 else ""))
    except RuntimeError as e:
        print(f"  (no final encoder reading: {e})")
    print(f"  heading change  {yaw:+6.1f} deg (IMU, + = turned left)")
    print(f"  sideways drift  {y * 100:+6.1f} cm  (+ = left)")
    if breakaway is not None:
        print(f"  breakaway duty  {breakaway:6.0f} %")
    if samples:
        avg_l, avg_r = duty_sum["left"] / samples, duty_sum["right"] / samples
        print(f"  average duty    left {avg_l:.1f}%  right {avg_r:.1f}%"
              + ("  (open loop)" if args.open_loop else ""))
    if cruise_speeds:
        # Second half only: the first includes the acceleration (a whole-run
        # median read 8-16% low at 50-65% duty in a simulated check).
        settled = cruise_speeds[len(cruise_speeds) // 2:]
        cruise = sorted(settled)[len(settled) // 2]
        cruise_duty = max(breakaway, args.duty)
        print(f"  cruise speed    {cruise:6.2f} m/s at {cruise_duty:.0f}% duty "
              f"(motor_node max_linear_speed_mps {cruise / (cruise_duty / 100):.1f} if linear)")
    print("  Distances assume no wheel slip -- check against a tape measure.")
    if fault_note:
        print(f"  driver fault: {fault_note}")
    print("  encoder diagnostics:")
    for line in enc.diagnostics(start):
        print(f"    {line}")
    sys.exit(0 if reason == "reached target distance" else 1)


if __name__ == "__main__":
    main()
