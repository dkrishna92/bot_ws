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
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

try:
    import lgpio
except ImportError:
    lgpio = None  # allows this module to be imported on non-Pi dev machines

LGPIO_MAX_PWM_HZ = 10000
# Kick-start: wheel odometry older than this counts as missing, and these
# speeds count as "wheels turning" (well above encoder noise at rest).
ODOM_STALE_S = 0.3
WHEELS_TURNING_MPS = 0.05
WHEELS_TURNING_RADPS = 0.15


class MotorNode(Node):
    def __init__(self, **kwargs):
        super().__init__("motor_node", **kwargs)

        self.declare_parameter("dry_run", False)
        # RP1 header GPIO is gpiochip4 on Ubuntu's 6.8 raspi kernel (gpiochip0
        # on newer Raspberry Pi OS kernels) -- check `gpioinfo` if pins don't move.
        self.declare_parameter("gpio_chip", 4)
        self.declare_parameter("pwm_frequency_hz", 10000)
        # The Pololu #4843 motors are rated 12 V but run off a 4S pack
        # (16.8 V full, ~16.2 V under load): 0.75 keeps the average motor
        # voltage near 12 V on a full pack (2026-09-29, was 0.9 = ~15 V).
        self.declare_parameter("max_duty_cycle", 0.75)
        # Deadband compensation -- see _set_channel's comment.
        # drive_straight.py broke away at 28% (2026-09-30), so 0.30 (was
        # 0.25, below breakaway: slow commands could leave it whining in
        # place). Once rolling, 35% cruised 1.3 m/s, so this floor also
        # makes ~1.1 m/s the slowest straight speed from these duties.
        self.declare_parameter("min_duty_cycle", 0.30)
        # Higher floor used only while the two sides turn in opposite
        # directions (pivoting in place): skid-steer wheels have to scrub
        # sideways, which needs far more torque than rolling straight, and
        # Set from scripts/rotate.py: pivoting on tile broke away at 40%
        # duty (2026-09-29, was 0.45); 2026-09-30 it broke away at 61%, so
        # 0.66. Re-measure with rotate.py on the actual course surface and
        # set this a few percent above its breakaway duty. Once turning, 61%
        # spun ~110 deg/s -- a floor this high makes small pivot
        # corrections overshoot (use Q/E arcs while mapping).
        self.declare_parameter("min_turn_duty_cycle", 0.66)
        # Kick-start (2026-10-01): the two floors above are BREAKAWAY floors
        # -- what it takes to get the wheels turning from rest. Once they
        # turn, much less keeps them turning (35% duty cruised 1.3 m/s), so
        # after kick_duration_s -- and only once wheel odometry shows the
        # wheels actually turning -- the floor drops to these running
        # floors. That allows straight speeds well below the ~1.1 m/s the
        # breakaway floor forced, and pivots slower than its ~110 deg/s.
        # If the wheels stop while still commanded (stall), it kicks again.
        # Without fresh wheel odometry the breakaway floors stay on (the old
        # behaviour); setting the running floors equal to the breakaway
        # ones also restores it. Defaults are estimates -- check with
        # teleop / drive_straight.py on the course surface.
        # Measured 2026-10-01 with running_floor.py: kept rolling down to 8%
        # duty (0.19 m/s), stalled at 6% -> 0.11 (~0.4 m/s slowest straight).
        self.declare_parameter("min_running_duty_cycle", 0.11)
        # Measured 2026-10-01 with running_floor.py --pivot: kept turning down
        # to 50% (42 deg/s), stalled at 48% -> 0.53 (~80 deg/s slowest pivot;
        # the curve is steep: 60% already spun 147 deg/s).
        self.declare_parameter("min_running_turn_duty_cycle", 0.53)
        self.declare_parameter("kick_duration_s", 0.15)
        self.declare_parameter("odom_topic", "odom")  # wheel_odom_node's raw wheel odometry
        self.declare_parameter("cmd_vel_timeout_s", 0.3)
        self.declare_parameter("track_width_m", 0.33)  # measured 2026-09-30 (was 0.32)
        # Wheel speed at 100% duty (open loop: duty = v / this). Measured
        # 2026-09-30 with drive_straight.py: settled 1.91 m/s at 50% on a
        # 5 m run (tape-checked) -> 3.8 on a full pack. Was 2.0, which drove
        # everything ~1.9x faster than commanded.
        self.declare_parameter("max_linear_speed_mps", 3.8)
        # Pin map fixed by the Pololu Dual G2 for Raspberry Pi board (BCM GPIO
        # numbers): motor 1 = channel A = left, motor 2 = channel B = right.
        # FLT is the driver's open-drain fault output (low = fault), read here
        # with the Pi's pull-up; SLP must be high to enable a channel.
        self.declare_parameter("channel_a_pwm_pin", 12)
        self.declare_parameter("channel_a_dir_pin", 24)
        self.declare_parameter("channel_a_sleep_pin", 22)
        self.declare_parameter("channel_a_fault_pin", 5)
        self.declare_parameter("channel_b_pwm_pin", 13)
        self.declare_parameter("channel_b_dir_pin", 25)
        self.declare_parameter("channel_b_sleep_pin", 23)
        self.declare_parameter("channel_b_fault_pin", 6)
        # The two motors face opposite ways (mirror-image mounting), so the
        # same DIR level turns them in opposite directions. Right channel was
        # inverted for this reason (checked 2026-09-27), and left also
        # inverted 2026-09-28 after a new left motor's leads landed reversed
        # (scripts/motor_test.py caught it: left counted backwards on
        # "forward"). Right flipped back to NOT inverted the same day: a
        # direct visual check caught the right wheel spinning backwards
        # relative to command even though motor_test.py said "OK" -- that
        # script only checks self-consistency between commanded direction
        # and encoder count, so it can't catch (and was fooled by) the
        # motor's real direction AND the right encoder's sign both being
        # wrong at once (see teensy_ws's RIGHT_ENCODER_SIGN fix for the
        # other half of that). If a motor is ever rewired/replaced again,
        # re-run motor_test.py AND visually watch the wheel -- don't trust
        # the script's encoder-only verdict alone.
        self.declare_parameter("channel_a_inverted", True)
        self.declare_parameter("channel_b_inverted", False)

        self._freq = min(self.get_parameter("pwm_frequency_hz").value, LGPIO_MAX_PWM_HZ)
        self._max_duty = self.get_parameter("max_duty_cycle").value
        self._min_duty = self.get_parameter("min_duty_cycle").value
        self._min_turn_duty = self.get_parameter("min_turn_duty_cycle").value
        self._min_run_duty = self.get_parameter("min_running_duty_cycle").value
        self._min_run_turn_duty = self.get_parameter("min_running_turn_duty_cycle").value
        self._kick_s = self.get_parameter("kick_duration_s").value
        self._timeout_s = self.get_parameter("cmd_vel_timeout_s").value
        self._track_width = self.get_parameter("track_width_m").value
        self._max_v = self.get_parameter("max_linear_speed_mps").value

        self._chan_a = {
            "pwm": self.get_parameter("channel_a_pwm_pin").value,
            "dir": self.get_parameter("channel_a_dir_pin").value,
            "sleep": self.get_parameter("channel_a_sleep_pin").value,
            "fault": self.get_parameter("channel_a_fault_pin").value,
            "inverted": self.get_parameter("channel_a_inverted").value,
        }
        self._chan_b = {
            "pwm": self.get_parameter("channel_b_pwm_pin").value,
            "dir": self.get_parameter("channel_b_dir_pin").value,
            "sleep": self.get_parameter("channel_b_sleep_pin").value,
            "fault": self.get_parameter("channel_b_fault_pin").value,
            "inverted": self.get_parameter("channel_b_inverted").value,
        }

        self._last_cmd: Twist | None = None
        self._last_cmd_time = 0.0
        self._fault = False
        self._driver_fault = False
        # Kick-start state: last drive mode ("stop" / "drive" / "pivot"), when
        # the current kick ends, and the latest wheel-odometry twist.
        self._mode = "stop"
        self._kick_until = 0.0
        self._odom_time: float | None = None
        self._odom_v = 0.0
        self._odom_w = 0.0

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
                    lgpio.gpio_claim_input(self._h, chan["fault"], lgpio.SET_PULL_UP)
            except lgpio.error as e:
                self.get_logger().error(f"could not open/claim gpiochip{chip}: {e} -- "
                                        "no hardware output (check gpio_chip and group membership)")
                if self._h is not None:
                    lgpio.gpiochip_close(self._h)
                self._h = None

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "system_fault", self._on_fault, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)
        self.create_timer(1.0 / 50.0, self._tick)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_time = time.monotonic()
        self._odom_v = msg.twist.twist.linear.x
        self._odom_w = msg.twist.twist.angular.z

    def _wheels_turning(self, now: float) -> bool | None:
        """From wheel odometry: are the wheels turning? None if there's no
        fresh odometry (then the kick-start can't tell, see _floor)."""
        if self._odom_time is None or now - self._odom_time > ODOM_STALE_S:
            return None
        return abs(self._odom_v) > WHEELS_TURNING_MPS or abs(self._odom_w) > WHEELS_TURNING_RADPS

    def _floor(self, mode: str, now: float) -> float:
        """Duty floor for this tick: the breakaway floor while kicking, the
        running floor once the wheels turn. Kicks on every start or mode
        change, and again whenever the wheels stop while still commanded."""
        if mode == "pivot":
            kick, running = self._min_turn_duty, self._min_run_turn_duty
        else:
            kick, running = self._min_duty, self._min_run_duty
        if mode != self._mode:
            self._kick_until = now + self._kick_s
        turning = self._wheels_turning(now)
        if turning is None:
            return kick  # no wheel feedback: always the breakaway floor (old behaviour)
        if not turning and now >= self._kick_until:
            self._kick_until = now + self._kick_s  # stopped/stalled while commanded: kick again
        return kick if now < self._kick_until else running

    def _on_fault(self, msg: Bool) -> None:
        if msg.data and not self._fault:
            self.get_logger().error("system fault asserted -- forcing motor stop")
        self._fault = msg.data

    def _set_channel(self, chan: dict, duty: float, min_duty: float | None = None) -> None:
        duty = max(-self._max_duty, min(self._max_duty, duty))
        if min_duty is None:
            min_duty = self._min_duty
        if self._h is None:
            return
        # _tick runs at 50 Hz; re-issuing tx_pwm restarts the software PWM
        # waveform, so only touch the pins when the command actually changes.
        if chan.get("last_duty") == (duty, min_duty):
            return
        chan["last_duty"] = (duty, min_duty)
        forward = (duty >= 0) != chan["inverted"]
        lgpio.gpio_write(self._h, chan["dir"], 1 if forward else 0)
        # Below min_duty_cycle, PWM is too weak to overcome static/scrub
        # friction at all -- the motor draws current and whines in place
        # instead of actually turning (real symptom, 2026-09-28, worst
        # during in-place rotation: track_width_m is narrow enough that
        # even Nav2's max_vel_theta maps to ~8% duty via the naive
        # v/max_linear_speed_mps scaling below -- nowhere near this
        # motor's stall-adjacent torque needs for scrub friction, see
        # CLAUDE.md's Hardware section). Any nonzero commanded duty is
        # clamped up to at least this floor instead of letting that
        # scaling produce an arbitrarily weak signal.
        effective_duty = abs(duty)
        if effective_duty > 0.0:
            effective_duty = min(max(effective_duty, min_duty), self._max_duty)
        lgpio.tx_pwm(self._h, chan["pwm"], self._freq, effective_duty * 100.0)

    def _stop_all(self) -> None:
        self._set_channel(self._chan_a, 0.0)
        self._set_channel(self._chan_b, 0.0)

    def _read_driver_fault(self) -> bool:
        """True while either G2 channel pulls its FLT pin low."""
        if self._h is None:
            return False
        faulted = [name for name, chan in (("left", self._chan_a), ("right", self._chan_b))
                   if lgpio.gpio_read(self._h, chan["fault"]) == 0]
        if faulted and not self._driver_fault:
            self.get_logger().error(f"motor driver FLT asserted ({', '.join(faulted)}) -- "
                                    "stopping motors (over-current, over-temperature or under-voltage)")
        elif not faulted and self._driver_fault:
            self.get_logger().info("motor driver FLT cleared")
        self._driver_fault = bool(faulted)
        return self._driver_fault

    def _tick(self) -> None:
        now = time.monotonic()
        stale = (now - self._last_cmd_time) > self._timeout_s
        driver_fault = self._read_driver_fault()
        if self._fault or driver_fault or stale or self._last_cmd is None:
            self._mode = "stop"
            self._stop_all()
            return

        v = self._last_cmd.linear.x
        w = self._last_cmd.angular.z
        v_left = v - w * self._track_width / 2.0
        v_right = v + w * self._track_width / 2.0

        duty_l, duty_r = v_left / self._max_v, v_right / self._max_v
        if v_left * v_right < 0:
            mode = "pivot"
        elif duty_l == 0.0 and duty_r == 0.0:
            mode = "stop"
        else:
            mode = "drive"
        floor = self._floor(mode, now) if mode != "stop" else 0.0
        self._mode = mode
        if mode == "pivot":
            # Sides turning opposite ways = pivoting, which needs the higher floor
            self._set_channel(self._chan_a, duty_l, floor)
            self._set_channel(self._chan_b, duty_r, floor)
            return
        # Driving straight or along an arc: lift BOTH sides together so the
        # faster one reaches min_duty_cycle, keeping their ratio -- the ratio
        # is the arc. Flooring each side separately turned every slow gentle
        # arc into a straight line (e.g. 0.3 m/s teleop: 0.17 / 0.13 duty ->
        # both 0.25), so the robot could only change direction by pivoting,
        # which jerks and smears the map (2026-10-01).
        big = max(abs(duty_l), abs(duty_r))
        if 0.0 < big < floor:
            scale = floor / big
            duty_l, duty_r = duty_l * scale, duty_r * scale
        # No per-side floor on top: the outer side is already >= min_duty,
        # and the inner one must stay proportionally slower.
        self._set_channel(self._chan_a, duty_l, 0.0)
        self._set_channel(self._chan_b, duty_r, 0.0)

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
