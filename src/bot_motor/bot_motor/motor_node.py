"""Motor driver node: Pololu Dual G2 High-Power Motor Driver 18v18.

Ported from the earlier bare-Python prototype. Controlled via PWM + DIR +
SLEEP GPIO lines (no serial/I2C interface on this board -- see CLAUDE.md,
this was a corrected mistake earlier in the project, don't reintroduce a
serial-protocol assumption here).

Subscribes to geometry_msgs/Twist on /cmd_vel (standard Nav2 output topic)
and a diagnostic_msgs/DiagnosticStatus-style fault signal on /system_fault
from watchdog_node. Converts (linear, angular) to per-channel PWM duty
cycle + direction for a differential drivetrain -- replace the mixing in
cmd_vel_callback() if the final drivetrain is Ackermann + separate steering
servo instead.

pigpio (not RPi.GPIO) is used for hardware-timed PWM -- lower jitter than
software PWM, worth it for smooth low-speed control. Requires `pigpiod`
running on the Pi (`sudo systemctl enable --now pigpiod`).

Enforces a short cmd_vel timeout as a second layer of defense alongside the
MCU-based hard e-stop (see CLAUDE.md safety section) -- NOT a substitute
for the MCU's real-time cutoff.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

try:
    import pigpio
except ImportError:
    pigpio = None  # allows this module to be imported on non-Pi dev machines


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")

        self.declare_parameter("pwm_frequency_hz", 20000)
        self.declare_parameter("max_duty_cycle", 0.9)
        self.declare_parameter("cmd_vel_timeout_s", 0.3)
        self.declare_parameter("track_width_m", 0.28)
        self.declare_parameter("max_linear_speed_mps", 2.0)
        self.declare_parameter("channel_a_pwm_pin", 12)
        self.declare_parameter("channel_a_dir_pin", 5)
        self.declare_parameter("channel_a_sleep_pin", 6)
        self.declare_parameter("channel_b_pwm_pin", 13)
        self.declare_parameter("channel_b_dir_pin", 16)
        self.declare_parameter("channel_b_sleep_pin", 19)

        self._freq = self.get_parameter("pwm_frequency_hz").value
        self._max_duty = self.get_parameter("max_duty_cycle").value
        self._timeout_s = self.get_parameter("cmd_vel_timeout_s").value
        self._track_width = self.get_parameter("track_width_m").value
        self._max_v = self.get_parameter("max_linear_speed_mps").value

        self._chan_a = {
            "pwm": self.get_parameter("channel_a_pwm_pin").value,
            "dir": self.get_parameter("channel_a_dir_pin").value,
            "sleep": self.get_parameter("channel_a_sleep_pin").value,
        }
        self._chan_b = {
            "pwm": self.get_parameter("channel_b_pwm_pin").value,
            "dir": self.get_parameter("channel_b_dir_pin").value,
            "sleep": self.get_parameter("channel_b_sleep_pin").value,
        }

        self._last_cmd: Twist | None = None
        self._last_cmd_time = 0.0
        self._fault = False

        self._pi = None
        if pigpio is not None:
            self._pi = pigpio.pi()
            if not self._pi.connected:
                self.get_logger().error("could not connect to pigpiod -- is it running?")
                self._pi = None
            else:
                for chan in (self._chan_a, self._chan_b):
                    self._pi.set_mode(chan["dir"], pigpio.OUTPUT)
                    self._pi.set_mode(chan["sleep"], pigpio.OUTPUT)
                    self._pi.set_PWM_frequency(chan["pwm"], self._freq)
                    self._pi.write(chan["sleep"], 1)  # wake
        else:
            self.get_logger().warn("pigpio not available -- running in dry-run mode "
                                    "(no hardware output). Expected on a non-Pi dev machine.")

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "system_fault", self._on_fault, 10)
        self.create_timer(1.0 / 50.0, self._tick)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()

    def _on_fault(self, msg: Bool) -> None:
        if msg.data and not self._fault:
            self.get_logger().error("system fault asserted -- forcing motor stop")
        self._fault = msg.data

    def _set_channel(self, chan: dict, duty: float) -> None:
        duty = max(-self._max_duty, min(self._max_duty, duty))
        if self._pi is None:
            return
        self._pi.write(chan["dir"], 1 if duty >= 0 else 0)
        self._pi.set_PWM_dutycycle(chan["pwm"], int(abs(duty) * 255))

    def _stop_all(self) -> None:
        self._set_channel(self._chan_a, 0.0)
        self._set_channel(self._chan_b, 0.0)

    def _tick(self) -> None:
        stale = (time.monotonic() - self._last_cmd_time) > self._timeout_s
        if self._fault or stale or self._last_cmd is None:
            self._stop_all()
            return

        v = self._last_cmd.linear.x
        w = self._last_cmd.angular.z
        v_left = v - w * self._track_width / 2.0
        v_right = v + w * self._track_width / 2.0

        self._set_channel(self._chan_a, v_left / self._max_v)
        self._set_channel(self._chan_b, v_right / self._max_v)

    def destroy_node(self) -> None:
        self._stop_all()
        if self._pi is not None:
            for chan in (self._chan_a, self._chan_b):
                self._pi.write(chan["sleep"], 0)
            self._pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
