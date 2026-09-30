#!/usr/bin/env python3
"""Find where the robot's heading goes wrong: compare every heading source.

Run while mapping or bringup is up, then turn the robot (by hand, or slowly
with teleop) about 90 deg LEFT, pause, and back. Prints, twice a second, how
far each source thinks the robot has turned since the script started:

  imu orient  -- /imu orientation yaw (the BNO055's own fusion)
  imu gyro    -- /imu angular_velocity.z integrated here
  wheels      -- /odom twist.angular.z integrated (skid-steer: overstates turns)
  ekf         -- /odometry/filtered yaw (what slam_toolbox gets as odometry)
  slam        -- /pose yaw (slam_toolbox's map pose, mapping only)

and on Ctrl-C a verdict. What to look for:
  - imu orient and imu gyro must agree in SIGN and roughly in size. The EKF
    fuses both (ekf_params.yaml imu0_config yaw + yaw rate); if they
    disagree it gets conflicting headings and the map rotates.
  - a LEFT turn must read POSITIVE everywhere (ROS convention).
  - ekf should follow the IMU, not the wheels.
  - slam should follow ekf; a sudden slam jump the others don't show is the
    scan matcher snapping to a wrong fit (see the SLAM settings in the web
    dashboard: coarse_search_angle_offset, minimum_time_interval, loop closing).

Usage (on the Pi, workspace sourced):
    scripts/check_heading.py
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

SOURCES = ("imu orient", "imu gyro", "wheels", "ekf", "slam")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Unwrapped:
    """Accumulates yaw change since the first reading, across +-pi wraps."""

    def __init__(self):
        self.last = None
        self.total = 0.0

    def update(self, yaw):
        if self.last is not None:
            self.total += math.atan2(math.sin(yaw - self.last), math.cos(yaw - self.last))
        self.last = yaw


class HeadingCheck:
    def __init__(self, node):
        self.orient = {n: Unwrapped() for n in ("imu orient", "ekf", "slam")}
        self.rate_total = {"imu gyro": 0.0, "wheels": 0.0}
        self.rate_last = {}
        self.counts = {n: 0 for n in SOURCES}
        self.max_abs = {n: 0.0 for n in SOURCES}
        node.create_subscription(Imu, "imu", self.on_imu, qos_profile_sensor_data)
        node.create_subscription(Odometry, "odom", self.on_odom, 20)
        node.create_subscription(Odometry, "odometry/filtered", self.on_ekf, 20)
        node.create_subscription(PoseWithCovarianceStamped, "pose", self.on_slam, 10)

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _integrate(self, name, rate, stamp):
        last = self.rate_last.get(name)
        if last is not None and 0.0 < stamp - last < 0.5:
            self.rate_total[name] += rate * (stamp - last)
        self.rate_last[name] = stamp
        self.counts[name] += 1

    def on_imu(self, msg):
        self.orient["imu orient"].update(yaw_of(msg.orientation))
        self.counts["imu orient"] += 1
        self._integrate("imu gyro", msg.angular_velocity.z, self._stamp(msg))

    def on_odom(self, msg):
        self._integrate("wheels", msg.twist.twist.angular.z, self._stamp(msg))

    def on_ekf(self, msg):
        self.orient["ekf"].update(yaw_of(msg.pose.pose.orientation))
        self.counts["ekf"] += 1

    def on_slam(self, msg):
        self.orient["slam"].update(yaw_of(msg.pose.pose.orientation))
        self.counts["slam"] += 1

    def turned_deg(self):
        out = {}
        for n in SOURCES:
            if self.counts[n]:
                rad = self.orient[n].total if n in self.orient else self.rate_total[n]
                out[n] = math.degrees(rad)
                self.max_abs[n] = max(self.max_abs[n], abs(out[n]))
        return out


def verdict(check):
    turned = check.turned_deg()
    peak = check.max_abs
    lines = ["", "Verdict:"]
    missing = [n for n in SOURCES if not check.counts[n]]
    if missing:
        lines.append(f"  no data from: {', '.join(missing)}"
                     + ("  <-- the EKF has no heading source!" if "imu orient" in missing else ""))
    if max(peak.values(), default=0) < 20:
        lines.append("  the robot barely turned -- turn it ~90 deg and run again")
        return "\n".join(lines)
    ref = peak.get("imu gyro") or 0.0

    def compare(name, what):
        if not check.counts[name] or ref < 20:
            return
        ratio = peak[name] / ref
        # sign agreement: compare the running totals at their peaks
        sign_ok = (turned.get(name, 0) >= 0) == (turned.get("imu gyro", 0) >= 0) or abs(turned.get(name, 0)) < 5
        if not sign_ok:
            lines.append(f"  {name}: OPPOSITE SIGN to the IMU gyro -- {what}")
        elif not 0.8 <= ratio <= 1.25:
            lines.append(f"  {name}: {ratio:.2f}x the IMU gyro's turn -- {what}")
        else:
            lines.append(f"  {name}: agrees with the IMU gyro ({ratio:.2f}x)")

    compare("imu orient", "the EKF fuses both IMU yaw and yaw rate, so they fight: fix the IMU orientation "
                          "(bno055_node / imu_joint yaw) before tuning anything else")
    compare("ekf", "the EKF isn't following the IMU (check ekf_params.yaml imu0_config and /imu rate)")
    compare("slam", "slam_toolbox's scan matcher is rotating the map away from odometry "
                    "(SLAM settings: coarse_search_angle_offset, angle_variance_penalty, minimum_time_interval)")
    if check.counts["wheels"]:
        lines.append(f"  wheels: {peak['wheels'] / max(ref, 1e-6):.2f}x the IMU gyro "
                     "(skid-steer wheels slip when turning, so >1 is normal; the EKF shouldn't use wheel yaw)")
    lines.append("  a LEFT turn should read POSITIVE in every source")
    return "\n".join(lines)


def main():
    rclpy.init()
    node = rclpy.create_node("check_heading")
    check = HeadingCheck(node)
    print("Turn the robot ~90 deg LEFT, pause, then back. Ctrl-C for the verdict.\n")
    print("  time  " + "".join(f"{n:>12s}" for n in SOURCES) + "   (deg turned since start)")
    start = last = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - last >= 0.5:
                last = time.monotonic()
                t = check.turned_deg()
                print(f"{last - start:6.1f}" + "".join(
                    f"{t[n]:12.1f}" if n in t else f"{'--':>12s}" for n in SOURCES), flush=True)
    except KeyboardInterrupt:
        pass
    print(verdict(check))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
