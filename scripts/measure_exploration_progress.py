#!/usr/bin/env python3
"""Measures how far frontier_explore_node drives the robot in a fixed
window of sim time -- the metric requested alongside this script: straight-
line displacement from the robot's start pose to its pose `duration`
seconds later.

Straight-line displacement is a deliberately blunt metric: a frontier
explorer wanders to cover area, so it can rack up a long, winding path and
still land close to where it started (loop back near the start box, get
stuck oscillating between two frontiers, etc.). That is a real,
informative failure mode this metric is meant to catch, not a flaw in it --
a low displacement number over 60s is worth investigating even if the
robot never stopped moving. Cumulative path length is reported alongside
it for exactly that reason: low displacement + high path length means
"drove a lot, went nowhere", which is a meaningfully different problem
than "barely moved at all".

Uses /odom directly (not map->base_link TF) so the measurement starts
from the very first odometry sample -- slam_toolbox's map frame doesn't
exist yet at t=0, only coming into existence after the first scan match.
Elapsed time is computed from message header stamps (sim time, since
mapping.launch.py runs with use_sim_time:=true), not wall clock, so the
result doesn't depend on Gazebo's real-time factor.

Usage (run against an already-launched mapping.launch.py):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 scripts/measure_exploration_progress.py [--duration 60] [--topic /odom]
"""
import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class ExplorationProgressMeter(Node):
    def __init__(self, duration: float, topic: str):
        super().__init__('exploration_progress_meter')
        self._duration = duration
        self._start_t = None
        self._start_xy = None
        self._last_xy = None
        self._last_t = None
        self._path_length = 0.0
        self._done = False
        self.result = None
        self.create_subscription(Odometry, topic, self._on_odom, 20)

    def _on_odom(self, msg: Odometry):
        if self._done:
            return
        t = stamp_to_sec(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._start_t is None:
            self._start_t = t
            self._start_xy = (x, y)
            self._last_xy = (x, y)
            self._last_t = t
            self.get_logger().info(
                f'Start pose captured at sim t={t:.2f}s: x={x:.3f} y={y:.3f}'
            )
            return

        self._path_length += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        self._last_t = t

        elapsed = t - self._start_t
        if elapsed >= self._duration:
            dx = x - self._start_xy[0]
            dy = y - self._start_xy[1]
            displacement = math.hypot(dx, dy)
            self.result = {
                'duration_requested_s': self._duration,
                'elapsed_s': elapsed,
                'start_xy': self._start_xy,
                'end_xy': (x, y),
                'straight_line_displacement_m': displacement,
                'path_length_m': self._path_length,
                'avg_speed_m_per_s': self._path_length / elapsed if elapsed > 0 else 0.0,
            }
            self._done = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.0,
                         help='Sim-time window to measure, in seconds (default: 60)')
    parser.add_argument('--topic', type=str, default='/odom',
                         help='Odometry topic to sample (default: /odom)')
    parser.add_argument('--timeout', type=float, default=180.0,
                         help='Wall-clock giveup if the window never completes (default: 180)')
    parser.add_argument('--out', type=str, default=None,
                         help='Optional path to write the result as JSON')
    args = parser.parse_args()

    rclpy.init()
    node = ExplorationProgressMeter(args.duration, args.topic)

    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.result is None:
        print(f'FAILED: no {args.duration}s window of {args.topic} data within '
              f'{args.timeout}s wall-clock timeout (topic never published, or sim stalled).',
              file=sys.stderr)
        sys.exit(1)

    result = node.result
    print('--- Exploration progress ---')
    print(f'  Window:                 {result["elapsed_s"]:.2f}s (requested {result["duration_requested_s"]:.0f}s)')
    print(f'  Start (x, y):           ({result["start_xy"][0]:.3f}, {result["start_xy"][1]:.3f})')
    print(f'  End   (x, y):           ({result["end_xy"][0]:.3f}, {result["end_xy"][1]:.3f})')
    print(f'  Straight-line displacement: {result["straight_line_displacement_m"]:.3f} m')
    print(f'  Path length driven:     {result["path_length_m"]:.3f} m')
    print(f'  Avg speed:              {result["avg_speed_m_per_s"]:.3f} m/s')

    if args.out:
        os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
        with open(args.out, 'w') as f:
            json.dump({**result, 'timestamp': datetime.now().isoformat()}, f, indent=2)
        print(f'  Wrote: {args.out}')


if __name__ == '__main__':
    main()
