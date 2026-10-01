#!/usr/bin/env python3
"""Measure the RUNNING duty floor: the least PWM that keeps the robot moving
once it already moves (for motor_node's kick-start running floors).

Breaking away from rest takes far more than staying in motion (straight:
28% to start, yet 35% cruised 1.3 m/s). This kicks the wheels free at
--kick for 0.3 s, then steps the duty down by --step every --step-s while
measuring the speed, until the robot stops. The last duty that still moved
is the running floor; set motor_node's min_running_duty_cycle (straight)
or min_running_turn_duty_cycle (--pivot) a few percent above it.

Straight speed comes from the wheel encoders, pivot rate from the IMU gyro.
Drives the motors and reads the Teensy/IMU directly (no ROS) -- stop any
launch first. THE ROBOT WILL MOVE: clear space (about --max-distance ahead,
or room to spin), e-stop in hand. Aborts on driver fault, encoder loss,
the distance/angle limit, timeout or Ctrl-C; always stops the motors.

Usage:
    scripts/running_floor.py            # straight, forward
    scripts/running_floor.py --pivot    # pivot in place, counter-clockwise
"""
import argparse
import sys
import time

import drive_lib as dl

LOOP_HZ = 50
KICK_S = 0.3
MOVING_MPS = 0.05   # straight: slower than this over a whole step = stopped
MOVING_DPS = 10.0   # pivot: slower than this over a whole step = stopped
# Speed should fall roughly in proportion to duty. Falling below this share
# of that means the drive has stalled and the robot is only coasting --
# momentum alone carried a simulated robot two steps past its real floor.
STALL_RATIO = 0.6


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--pivot", action="store_true", help="measure the pivot floor (default: straight)")
    ap.add_argument("--kick", type=float, help="breakaway duty %% for the kick (default 40 straight, 70 pivot)")
    ap.add_argument("--start", type=float, help="first running duty %% (default 30 straight, 60 pivot)")
    ap.add_argument("--step", type=float, default=2.0, help="duty %% to drop per step (default 2)")
    ap.add_argument("--step-s", type=float, default=0.4, help="seconds per step (default 0.4)")
    ap.add_argument("--max-distance", type=float, default=4.0, help="straight: stop after this many m (default 4)")
    ap.add_argument("--max-angle", type=float, default=720.0, help="pivot: stop after this many deg (default 720)")
    ap.add_argument("--yes", action="store_true", help="skip the safety prompt")
    args = ap.parse_args()
    kick = args.kick if args.kick is not None else (70.0 if args.pivot else 40.0)
    duty = args.start if args.start is not None else (60.0 if args.pivot else 30.0)

    dl.require_hardware_free()
    enc = dl.Encoders()
    enc.wait_ready()
    imu = dl.Imu() if args.pivot else None
    what = "pivot in place (counter-clockwise)" if args.pivot else f"drive forward up to {args.max_distance:.1f} m"
    dl.confirm(f"Robot will {what}, stepping power down until it stops.", args.yes)

    def apply(d):
        if args.pivot:
            motors.set("left", -d)
            motors.set("right", d)
        else:
            motors.set("left", d)
            motors.set("right", d)

    l0, r0, _ = enc.read_stamped()
    if imu:
        imu.start()
    rows = []  # (duty, speed) per step
    reason = None
    last_moving = None
    with dl.Motors() as motors:
        try:
            # 1. Kick: get the wheels turning
            end = time.monotonic() + KICK_S
            while time.monotonic() < end:
                apply(kick)
                if motors.faults():
                    raise RuntimeError("motor driver fault during the kick")
                time.sleep(1 / LOOP_HZ)
            # 2. Step the duty down while measuring
            prev = None  # (duty, speed) of the previous step
            while duty > 0:
                apply(duty)

                def wait(seconds):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        if motors.faults():
                            raise RuntimeError(f"motor driver fault at {duty:.0f}% duty")
                        if imu:
                            imu.yaw_deg()  # keep its unwrap fed
                        time.sleep(1 / LOOP_HZ)

                wait(args.step_s / 2)  # let the speed settle; measure the second half only
                l_a, r_a, us_a = enc.read_stamped()
                yaw_a = imu.yaw_deg() if imu else 0.0
                wait(args.step_s / 2)
                l_b, r_b, us_b = enc.read_stamped()
                dt = max(1e-3, (us_b - us_a) / 1e6)
                if args.pivot:
                    yaw_b = imu.yaw_deg()
                    speed = (yaw_b - yaw_a) / dt
                    moving = speed > MOVING_DPS
                    done = yaw_b > args.max_angle
                else:
                    speed = ((l_b - l_a) + (r_b - r_a)) / 2 * dl.M_PER_TICK / dt
                    moving = speed > MOVING_MPS
                    done = ((l_b - l0) + (r_b - r0)) / 2 * dl.M_PER_TICK > args.max_distance
                if moving and prev and speed < STALL_RATIO * prev[1] * duty / prev[0]:
                    moving = False  # dropping far faster than the duty: stalled, coasting
                rows.append((duty, speed))
                prev = (duty, speed)
                if not moving:
                    reason = f"stopped at {duty:.0f}% duty"
                    break
                last_moving = duty
                if done:
                    reason = "hit the distance/angle limit while still moving -- lower --start and rerun"
                    break
                duty -= args.step
            else:
                reason = "still moving at 0% (coasting?) -- rerun with a longer --step-s"
        except KeyboardInterrupt:
            reason = "ABORTED: Ctrl-C"
        except RuntimeError as e:
            reason = f"ABORTED: {e}"
        motors.stop()
        time.sleep(0.5)  # brake (PWM low, driver awake) before the driver sleeps

    unit = "deg/s" if args.pivot else "m/s"
    print(f"\n{reason}")
    print(f"  {'duty':>5}  {'speed':>8} {unit}")
    for d, sp in rows:
        print(f"  {d:4.0f}%  {sp:8.2f}")
    if last_moving is not None:
        param = "min_running_turn_duty_cycle" if args.pivot else "min_running_duty_cycle"
        rec = min(dl.MAX_DUTY, last_moving + 3.0) / 100.0
        print(f"  lowest duty that kept it moving: {last_moving:.0f}%")
        print(f"  -> motor_node {param}: ~{rec:.2f} (a few % above, for margin)")
    print("  encoder diagnostics:")
    for line in enc.diagnostics(time.monotonic() - 60):
        print(f"    {line}")
    sys.exit(0 if last_moving is not None and not (reason or "").startswith("ABORTED") else 1)


if __name__ == "__main__":
    main()
