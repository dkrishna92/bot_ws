"""Wheel odometry + ultrasonic node: converts Teensy-reported quadrature
encoder ticks into nav_msgs/Odometry for robot_localization's EKF (see
CLAUDE.md, ekf_params.yaml's odom0), and Teensy-reported ultrasonic echo
pulses into sensor_msgs/Range.

The Teensy (teensy_ws) reads two quadrature encoders -- one per side,
matching bot.urdf.xacro's DiffDrive plugin's left_joint/right_joint
grouping -- and three HC-SR04-class ultrasonic sensors, reporting both
over one USB serial link as one line per sample:

    E,<left_ticks>,<right_ticks>,<micros>\n
    U,<front_left_us>,<front_right_us>,<rear_us>,<micros>\n

Ultrasonic sensing moved onto this Teensy from the Pi (2026-09-20),
retiring the old bot_ultrasonic package's bit-banged RPi.GPIO node. This
node is the sole owner of the Teensy's one serial link -- the Teensy
exposes a single virtual COM port, not a dual-serial USB config, so a
second ROS node reading the same port independently would race on
readline() calls. E-stop is a separate Arduino Nano wired directly to the
motor driver, not this Teensy, so there is no serial contention with
bot_safety's watchdog_node either (its MCU serial mirror is disabled --
see watchdog_node.py).

Publishes:
  - nav_msgs/Odometry on 'odom' (frame_id=odom, child_frame_id=base_link)
    -- the same topic/frame/message shape gz-sim's DiffDrive plugin
    publishes in sim, so this is a drop-in swap for ekf_params.yaml's
    odom0 with no downstream config changes. Does NOT publish TF:
    ekf_params.yaml (publish_tf: true) is the sole odom->base_link
    broadcaster, matching the sim path.
  - sensor_msgs/Range on 'ultrasonic/<front_left|front_right|rear>' --
    same topic names/shape as the old bot_ultrasonic node, so any future
    Nav2 range_sensor_layer config can reference them unchanged.

wheel_radius_m/track_width_m below should stay numerically in sync with
bot.urdf.xacro's wheel_radius/track_width properties. ticks_per_rev is the
real encoder spec now (Pololu #4843: 48 CPR motor shaft x 20.4:1 gear ratio
= 979.62 CPR gearbox output shaft) -- not a placeholder anymore, unlike the
project's still-unconfirmed RPLIDAR baudrate and the ultrasonic trigger/echo
pin assignment on the Teensy (see CLAUDE.md open items).
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

try:
    import serial
except ImportError:
    serial = None  # allows this module to be imported without a Teensy attached

SPEED_OF_SOUND_M_S = 343.0
ULTRASONIC_MAX_RANGE_M = 4.0
ULTRASONIC_FIELD_OF_VIEW_RAD = 0.26  # ~15 deg, typical HC-SR04
ULTRASONIC_MIN_RANGE_M = 0.02
ULTRASONIC_SENSOR_NAMES = ("front_left", "front_right", "rear")


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__("wheel_odom_node")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("ticks_per_rev", 979.62)
        self.declare_parameter("wheel_radius_m", 0.06)
        self.declare_parameter("track_width_m", 0.32)

        self._ticks_per_rev = self.get_parameter("ticks_per_rev").value
        self._wheel_radius = self.get_parameter("wheel_radius_m").value
        self._track_width = self.get_parameter("track_width_m").value
        self._m_per_tick = (2.0 * math.pi * self._wheel_radius) / self._ticks_per_rev

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_left_ticks: int | None = None
        self._last_right_ticks: int | None = None
        self._last_sample_time: float | None = None

        self._pub = self.create_publisher(Odometry, "odom", 10)
        self._range_publishers = {
            name: self.create_publisher(Range, f"ultrasonic/{name}", 10)
            for name in ULTRASONIC_SENSOR_NAMES
        }

        self._teensy_serial = None
        if serial is not None:
            try:
                self._teensy_serial = serial.Serial(
                    self.get_parameter("serial_port").value,
                    self.get_parameter("baud").value,
                    timeout=0.1,
                )
            except serial.SerialException:
                self.get_logger().warn(
                    "could not open Teensy encoder serial port -- "
                    "continuing without wheel odometry"
                )
        else:
            self.get_logger().warn(
                "pyserial not available -- running in dry-run mode "
                "(no odometry published). Expected on a non-Pi dev machine "
                "without a Teensy attached."
            )

        self.create_timer(1.0 / 100.0, self._poll_serial)

    def _poll_serial(self) -> None:
        if self._teensy_serial is None:
            return
        try:
            line = self._teensy_serial.readline().decode(errors="ignore").strip()
        except serial.SerialException:
            self.get_logger().error("lost Teensy encoder serial connection")
            return
        if not line:
            return
        self._on_line(line)

    def _on_line(self, line: str) -> None:
        parts = line.split(",")
        if not parts:
            return
        if parts[0] == "U":
            self._on_ultrasonic_line(parts)
            return
        if len(parts) != 4 or parts[0] != "E":
            return
        try:
            left_ticks = int(parts[1])
            right_ticks = int(parts[2])
        except ValueError:
            return

        now = time.monotonic()
        if self._last_left_ticks is None:
            self._last_left_ticks = left_ticks
            self._last_right_ticks = right_ticks
            self._last_sample_time = now
            return

        dt = now - self._last_sample_time
        if dt <= 0.0:
            return

        d_left = (left_ticks - self._last_left_ticks) * self._m_per_tick
        d_right = (right_ticks - self._last_right_ticks) * self._m_per_tick
        self._last_left_ticks = left_ticks
        self._last_right_ticks = right_ticks
        self._last_sample_time = now

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self._track_width

        # Midpoint integration -- same family as gz-sim's DiffDriveOdometry,
        # for behavioral parity between sim and real hardware.
        self._x += d_center * math.cos(self._theta + d_theta / 2.0)
        self._y += d_center * math.sin(self._theta + d_theta / 2.0)
        self._theta = math.atan2(
            math.sin(self._theta + d_theta), math.cos(self._theta + d_theta)
        )

        v = d_center / dt
        w = d_theta / dt
        self._publish_odom(v, w)

    def _publish_odom(self, v: float, w: float) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._theta / 2.0)

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self._pub.publish(msg)

    def _on_ultrasonic_line(self, parts: list[str]) -> None:
        if len(parts) != 5:
            return
        try:
            pulse_widths_us = [int(p) for p in parts[1:4]]
        except ValueError:
            return

        now = self.get_clock().now().to_msg()
        for name, pulse_us in zip(ULTRASONIC_SENSOR_NAMES, pulse_widths_us):
            # 0 is the Teensy's timeout/no-echo sentinel (see teensy_ws's
            # readUltrasonicPulseUs) -- distance conversion happens here,
            # not on the MCU, same "raw sample on the wire" split as ticks.
            distance_m = (pulse_us * 1e-6) * SPEED_OF_SOUND_M_S / 2.0
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = f"ultrasonic_{name}"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = ULTRASONIC_FIELD_OF_VIEW_RAD
            msg.min_range = ULTRASONIC_MIN_RANGE_M
            msg.max_range = ULTRASONIC_MAX_RANGE_M
            msg.range = (
                distance_m
                if pulse_us > 0 and distance_m <= ULTRASONIC_MAX_RANGE_M
                else ULTRASONIC_MAX_RANGE_M + 1.0  # out-of-range sentinel
            )
            self._range_publishers[name].publish(msg)

    def destroy_node(self) -> None:
        if self._teensy_serial is not None:
            self._teensy_serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
