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

lgpio is used for GPIO/PWM -- pigpio (used by the earlier prototype) does
not support the Pi 5's RP1 I/O chip and isn't packaged for Ubuntu 24.04
arm64, and RPi.GPIO doesn't work on the Pi 5 either. lgpio's PWM is
software-timed and capped at 10 kHz (pwm_frequency_hz is clamped to that);
fine for the G2 driver, but RP1 hardware PWM on GPIO12/13 via sysfs is the
upgrade path if jitter ever shows up under load. No daemon needed, but the
user must be in the gpio/dialout group for /dev/gpiochip* access (see
scripts/setup_pi.sh).

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
    import lgpio
except ImportError:
    lgpio = None  # allows this module to be imported on non-Pi dev machines

LGPIO_MAX_PWM_HZ = 10000


class MotorNode(Node):
    def __init__(self, **kwargs):
        super().__init__("motor_node", **kwargs)

        self.declare_parameter("dry_run", False)
        # RP1 header GPIO is gpiochip4 on Ubuntu's 6.8 raspi kernel (gpiochip0
        # on newer Raspberry Pi OS kernels) -- check `gpioinfo` if pins don't move.
        self.declare_parameter("gpio_chip", 4)
        self.declare_parameter("pwm_frequency_hz", 10000)
        self.declare_parameter("max_duty_cycle", 0.9)
        self.declare_parameter("cmd_vel_timeout_s", 0.3)
        self.declare_parameter("track_width_m", 0.32)
        self.declare_parameter("max_linear_speed_mps", 2.0)
        self.declare_parameter("channel_a_pwm_pin", 12)
        self.declare_parameter("channel_a_dir_pin", 5)
        self.declare_parameter("channel_a_sleep_pin", 6)
        self.declare_parameter("channel_b_pwm_pin", 13)
        self.declare_parameter("channel_b_dir_pin", 16)
        self.declare_parameter("channel_b_sleep_pin", 19)

        self._freq = min(self.get_parameter("pwm_frequency_hz").value, LGPIO_MAX_PWM_HZ)
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

        self._h = None
        if self.get_parameter("dry_run").value:
            self.get_logger().warn("dry_run set -- no hardware output")
        elif lgpio is None:
            self.get_logger().warn("lgpio not available -- running in dry-run mode "
                                    "(no hardware output). Expected on a non-Pi dev machine.")
        else:
            chip = self.get_parameter("gpio_chip").value
            try:
                self._h = lgpio.gpiochip_open(chip)
                for chan in (self._chan_a, self._chan_b):
                    lgpio.gpio_claim_output(self._h, chan["dir"], 0)
                    lgpio.gpio_claim_output(self._h, chan["pwm"], 0)
                    lgpio.gpio_claim_output(self._h, chan["sleep"], 1)  # wake
            except lgpio.error as e:
                self.get_logger().error(f"could not open/claim gpiochip{chip}: {e} -- "
                                        "no hardware output (check gpio_chip and group membership)")
                if self._h is not None:
                    lgpio.gpiochip_close(self._h)
                self._h = None

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
        if self._h is None:
            return
        # _tick runs at 50 Hz; re-issuing tx_pwm restarts the software PWM
        # waveform, so only touch the pins when the command actually changes.
        if chan.get("last_duty") == duty:
            return
        chan["last_duty"] = duty
        lgpio.gpio_write(self._h, chan["dir"], 1 if duty >= 0 else 0)
        lgpio.tx_pwm(self._h, chan["pwm"], self._freq, abs(duty) * 100.0)

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
        if self._h is not None:
            for chan in (self._chan_a, self._chan_b):
                lgpio.gpio_write(self._h, chan["sleep"], 0)
            lgpio.gpiochip_close(self._h)
            self._h = None
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
