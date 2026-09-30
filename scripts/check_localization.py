#!/usr/bin/env python3
"""Is AMCL (or slam_toolbox) localized? Checks the lidar against the map.

Projects each /scan into the map at the pose TF currently believes
(map -> lidar frame) and reports the fraction of returns that land on or
right next to an occupied map cell. Localized correctly, most returns hit
walls/bales in the map; mislocalized (wrong start pose, a kidnapped robot),
the score collapses -- even while every node reports healthy.

Run after bringup is up and BEFORE the start signal:
    scripts/check_localization.py              # 5 scans, then a verdict
    scripts/check_localization.py --watch      # keep printing (e.g. while driving)

Score guide (fraction of returns within --tolerance of an occupied cell):
    >= 0.6   localized
    0.3-0.6  doubtful -- check the start pose / wait for AMCL to converge
    <  0.3   NOT localized -- wrong start pose (see RACE_DAY.md)
Only returns up to --max-range are counted: far returns often see things
beyond the mapped area.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

GOOD, DOUBTFUL = 0.6, 0.3


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocalizationCheck:
    def __init__(self, node, max_range, tolerance):
        self.node = node
        self.max_range = max_range
        self.tolerance = tolerance
        self.map = None
        self.occupied = None
        self.scan = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, "map", self.on_map, latched)
        node.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)

    def on_map(self, msg):
        self.map = msg
        grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        occ = grid >= 65
        # Dilate by the tolerance so near-misses (map resolution, lidar noise) count
        r = max(1, int(round(self.tolerance / msg.info.resolution)))
        dil = occ.copy()
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                dil |= np.roll(np.roll(occ, dy, axis=0), dx, axis=1)
        self.occupied = dil

    def on_scan(self, msg):
        self.scan = msg

    def score(self):
        """(score, used_points, (x, y, yaw) of the lidar in the map) or a reason string."""
        if self.map is None:
            return "no /map yet"
        if self.scan is None:
            return "no /scan yet"
        scan = self.scan
        try:
            t = self.tf.lookup_transform(self.map.header.frame_id, scan.header.frame_id, rclpy.time.Time())
        except TransformException as exc:
            return f"no map -> {scan.header.frame_id} transform ({exc.__class__.__name__}): not localized yet"
        tx, ty = t.transform.translation.x, t.transform.translation.y
        tyaw = yaw_of(t.transform.rotation)
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        ok = np.isfinite(ranges) & (ranges > max(scan.range_min, 0.15)) & (ranges < min(scan.range_max, self.max_range))
        if ok.sum() < 20:
            return "too few lidar returns in range"
        a = angles[ok] + tyaw
        px = tx + ranges[ok] * np.cos(a)
        py = ty + ranges[ok] * np.sin(a)
        info = self.map.info
        gx = np.floor((px - info.origin.position.x) / info.resolution).astype(int)
        gy = np.floor((py - info.origin.position.y) / info.resolution).astype(int)
        inside = (gx >= 0) & (gx < info.width) & (gy >= 0) & (gy < info.height)
        hits = np.zeros(len(px), dtype=bool)
        hits[inside] = self.occupied[gy[inside], gx[inside]]
        return float(hits.mean()), int(ok.sum()), (tx, ty, tyaw)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--watch", action="store_true", help="keep printing until Ctrl-C")
    ap.add_argument("--scans", type=int, default=5, help="scans to score before the verdict (default 5)")
    ap.add_argument("--max-range", type=float, default=8.0, help="only count returns closer than this (m)")
    ap.add_argument("--tolerance", type=float, default=0.15, help="how close to an occupied cell counts (m)")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("check_localization")
    check = LocalizationCheck(node, args.max_range, args.tolerance)
    scores, deadline, last = [], time.monotonic() + 30.0, 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last < 0.5:
                continue
            last = now
            result = check.score()
            if isinstance(result, str):
                print(f"  waiting: {result}", flush=True)
                if not args.watch and now > deadline:
                    print("gave up after 30 s")
                    return 2
                continue
            score, used, (x, y, yaw) = result
            label = "localized" if score >= GOOD else "DOUBTFUL" if score >= DOUBTFUL else "NOT LOCALIZED"
            print(f"  scan match {score:4.0%} of {used} returns  at x {x:6.2f} y {y:6.2f} "
                  f"yaw {math.degrees(yaw):6.1f} deg  -> {label}", flush=True)
            scores.append(score)
            if not args.watch and len(scores) >= args.scans:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if not scores:
        return 2
    median = float(np.median(scores))
    if median >= GOOD:
        print(f"\nOK: localized (median scan match {median:.0%}).")
        return 0
    if median >= DOUBTFUL:
        print(f"\nDOUBTFUL (median {median:.0%}): wait a few seconds / nudge the robot and re-run; "
              "if it stays low, the start pose is off.")
        return 1
    print(f"\nNOT LOCALIZED (median {median:.0%}): AMCL's pose doesn't match the lidar. Most likely the start "
          "pose is wrong for this map: place the robot on the marked start spot, check <map>.start.yaml / "
          "initial_pose:=, then restart bringup.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
