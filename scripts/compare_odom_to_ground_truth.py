#!/usr/bin/env python3
"""Compares wheel odometry (/odom) against Gazebo's ground-truth model pose
(/model/bot/pose, from bot.urdf.xacro's sim-only PosePublisher plugin) to
test the "DiffDrive's odometry is kinematic, not physics ground truth"
hypothesis behind the frontier-exploration stall investigation (see
CLAUDE.md): if the chassis is wedged/colliding while the wheels still spin,
/odom over-reports displacement relative to what actually happened.

Both sources are compared as displacement-from-each-source's-own-start, not
raw overlaid coordinates -- /odom's frame origin (the DiffDrive plugin's
configure-time pose) and /model/bot/pose's frame (Gazebo world, offset by
the robot's spawn pose in gazebo_sim.launch.py) don't share an origin, but
they should track the same *motion* if /odom is trustworthy.

/model/bot/pose is geometry_msgs/msg/Pose (no header -- gz.msgs.Pose has no
timestamp), so this reports on a wall-clock timer instead of the
header-stamp-driven windowing measure_exploration_progress.py uses.

Usage (run against an already-launched mapping.launch.py or
bringup.launch.py with use_sim:=true):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 scripts/compare_odom_to_ground_truth.py [--duration 60] [--report-every 1.0]
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node

# Divergence beyond this (odom displacement minus ground-truth displacement,
# in meters) prints a flag -- a heuristic, not a validated threshold; adjust
# based on what a normal (non-stalled) run reports.
SLIP_FLAG_THRESHOLD_M = 0.15


class OdomGroundTruthComparer(Node):
    def __init__(self, report_every: float):
        super().__init__('odom_ground_truth_comparer')

        self._odom_start = None
        self._odom_xy = None
        self._gt_start = None
        self._gt_xy = None

        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(Pose, '/model/bot/pose', self._on_ground_truth, 20)
        self.create_timer(report_every, self._report)

    def _on_odom(self, msg: Odometry) -> None:
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self._odom_start is None:
            self._odom_start = xy
            self.get_logger().info(f'/odom start captured: x={xy[0]:.3f} y={xy[1]:.3f}')
        self._odom_xy = xy

    def _on_ground_truth(self, msg: Pose) -> None:
        xy = (msg.position.x, msg.position.y)
        if self._gt_start is None:
            self._gt_start = xy
            self.get_logger().info(f'ground-truth start captured: x={xy[0]:.3f} y={xy[1]:.3f}')
        self._gt_xy = xy

    def _report(self) -> None:
        if self._odom_xy is None or self._gt_xy is None:
            missing = [name for name, xy in (('/odom', self._odom_xy),
                                               ('/model/bot/pose', self._gt_xy)) if xy is None]
            self.get_logger().warn(f'waiting for: {", ".join(missing)}')
            return

        odom_disp = math.hypot(self._odom_xy[0] - self._odom_start[0],
                                self._odom_xy[1] - self._odom_start[1])
        gt_disp = math.hypot(self._gt_xy[0] - self._gt_start[0],
                              self._gt_xy[1] - self._gt_start[1])
        divergence = odom_disp - gt_disp

        flag = ' *** SLIP SUSPECTED ***' if divergence > SLIP_FLAG_THRESHOLD_M else ''
        print(f'odom displacement: {odom_disp:6.3f}m   '
              f'ground-truth displacement: {gt_disp:6.3f}m   '
              f'divergence: {divergence:+6.3f}m{flag}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.0,
                         help='Wall-clock seconds to run before exiting (default: 60)')
    parser.add_argument('--report-every', type=float, default=1.0,
                         help='Seconds between comparison prints (default: 1.0)')
    args = parser.parse_args()

    rclpy.init()
    node = OdomGroundTruthComparer(args.report_every)

    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
