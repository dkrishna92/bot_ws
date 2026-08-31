"""Ultrasonic sensor array node.

Publishes a standard sensor_msgs/Range per sensor so Nav2's costmap can
consume them directly via the range_sensor_layer plugin -- no custom
fusion code needed on the ROS 2 side (contrast with the earlier bare-Python
prototype, which had to hand-roll obstacle fusion).

Bit-banged GPIO timing via RPi.GPIO, same caveat as before: if this proves
noisy with more sensors, move triggering/echo timing to the safety MCU and
have it report ranges over serial instead.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None  # allows import on non-Pi dev machines

SPEED_OF_SOUND_M_S = 343.0


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_node")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("max_range_m", 4.0)
        self.declare_parameter("field_of_view_rad", 0.26)  # ~15 deg, typical HC-SR04
        # sensor_name: [trigger_pin, echo_pin, frame_id]
        self.declare_parameter("sensor_names", ["front_left", "front_right", "rear"])
        self.declare_parameter("front_left_pins", [23, 24])
        self.declare_parameter("front_right_pins", [27, 22])
        self.declare_parameter("rear_pins", [17, 4])

        self._max_range = self.get_parameter("max_range_m").value
        self._fov = self.get_parameter("field_of_view_rad").value
        self._timeout_s = 2 * self._max_range / SPEED_OF_SOUND_M_S + 0.005

        names = self.get_parameter("sensor_names").value
        self._sensors = {}
        for name in names:
            pins = self.get_parameter(f"{name}_pins").value
            self._sensors[name] = {"trigger": pins[0], "echo": pins[1]}

        self._publishers = {
            name: self.create_publisher(Range, f"ultrasonic/{name}", 10)
            for name in self._sensors
        }

        if GPIO is not None:
            GPIO.setmode(GPIO.BCM)
            for pins in self._sensors.values():
                GPIO.setup(pins["trigger"], GPIO.OUT)
                GPIO.setup(pins["echo"], GPIO.IN)
                GPIO.output(pins["trigger"], False)
            time.sleep(0.2)
        else:
            self.get_logger().warn("RPi.GPIO not available -- running in dry-run mode. "
                                    "Expected on a non-Pi dev machine.")

        rate_hz = self.get_parameter("rate_hz").value
        self.create_timer(1.0 / rate_hz, self._tick)

    def _read_one(self, trigger: int, echo: int) -> float | None:
        if GPIO is None:
            return None
        GPIO.output(trigger, True)
        time.sleep(0.00001)
        GPIO.output(trigger, False)

        start_wait = time.monotonic()
        while GPIO.input(echo) == 0:
            if time.monotonic() - start_wait > self._timeout_s:
                return None
        pulse_start = time.monotonic()

        while GPIO.input(echo) == 1:
            if time.monotonic() - pulse_start > self._timeout_s:
                return None
        pulse_end = time.monotonic()

        distance_m = (pulse_end - pulse_start) * SPEED_OF_SOUND_M_S / 2.0
        return distance_m if distance_m <= self._max_range else None

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        for name, pins in self._sensors.items():
            r = self._read_one(pins["trigger"], pins["echo"])
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = f"ultrasonic_{name}"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self._fov
            msg.min_range = 0.02
            msg.max_range = self._max_range
            msg.range = r if r is not None else self._max_range + 1.0  # out-of-range sentinel
            self._publishers[name].publish(msg)

    def destroy_node(self) -> None:
        if GPIO is not None:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
