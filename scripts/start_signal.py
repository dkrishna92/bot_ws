#!/usr/bin/env python3
"""Commands the sim start signal's arm from red to green, simulating a race
official flipping the physical start-signal arm.

The arm (start_signal_arms in speed_course_cfr.world / obstacle_course_cfr
.world) is a revolute joint driven by the gz-sim-joint-position-controller-
system plugin, commanded over /start_signal/arm -- bridged ROS<->GZ by
gazebo_sim.launch.py as a std_msgs/Float64. Position 0.0 rad shows the red
face (red_visual); the joint's upper limit, pi/2 rad, rotates the red face
edge-on and brings the green face (green_visual, pre-offset by -pi/2 in its
own visual pose) forward instead.

This script only drives the sim-side signal -- it is not part of the real
robot's software. bot_perception's start_trigger_node is what should react
to the color change (via the OAK-D's bridged camera topic) and start the
robot; see that node for the detection/trigger side.

Usage (run against an already-launched gazebo_sim.launch.py / bringup.launch.py
with use_sim:=true):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 scripts/start_signal.py                     # fixed 5s delay
    python3 scripts/start_signal.py --delay 8            # fixed 8s delay
    python3 scripts/start_signal.py --randomize          # random 3-8s delay
    python3 scripts/start_signal.py --randomize --min-delay 2 --max-delay 10
"""
import argparse
import math
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

RED_POSITION = 0.0
GREEN_POSITION = math.pi / 2.0


class StartSignalCommander(Node):
    def __init__(self):
        super().__init__('start_signal_commander')
        self._pub = self.create_publisher(Float64, '/start_signal/arm', 10)

    def wait_for_bridge(self, timeout_s: float = 10.0) -> bool:
        """Block until the ROS->GZ bridge has actually discovered this
        publisher. Publishing before discovery completes silently drops the
        message -- the arm then never moves, with nothing in any log to say
        why (this is exactly what made short --delay values look like the
        signal was broken)."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def command(self, position: float) -> None:
        self._pub.publish(Float64(data=position))


def _parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--delay', type=float, default=5.0,
                         help='Seconds to hold red before flipping to green (default: 5.0)')
    parser.add_argument('--randomize', action='store_true',
                         help='Pick the delay randomly from [--min-delay, --max-delay] '
                              'instead of using --delay, e.g. to test the detector '
                              'against an unpredictable flip time like a real official')
    parser.add_argument('--min-delay', type=float, default=3.0,
                         help='Lower bound for --randomize (default: 3.0)')
    parser.add_argument('--max-delay', type=float, default=8.0,
                         help='Upper bound for --randomize (default: 8.0)')
    args = parser.parse_args()
    if args.randomize and args.min_delay > args.max_delay:
        parser.error('--min-delay must be <= --max-delay')
    return args


def main() -> None:
    args = _parse_args()
    delay = random.uniform(args.min_delay, args.max_delay) if args.randomize else args.delay

    rclpy.init()
    node = StartSignalCommander()
    try:
        if not node.wait_for_bridge():
            node.get_logger().error(
                'Nothing subscribed to /start_signal/arm -- is the sim running '
                '(gazebo_sim.launch.py brings up the ros_gz bridge)? Not commanding '
                'the arm; it would be silently dropped.'
            )
            return

        # The joint already spawns at position 0 (red), but publish it
        # explicitly so the plugin's PID controller has a real setpoint
        # from the start rather than relying on wherever the joint happened
        # to settle.
        node.command(RED_POSITION)
        node.get_logger().info(f'Start signal RED -- flipping to GREEN in {delay:.1f}s')
        time.sleep(delay)
        node.command(GREEN_POSITION)
        node.get_logger().info('Start signal GREEN')
        # Hold the command briefly so a dropped/late-joining subscriber
        # still lands on GREEN rather than the arm sitting half-rotated.
        for _ in range(5):
            time.sleep(0.2)
            node.command(GREEN_POSITION)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
