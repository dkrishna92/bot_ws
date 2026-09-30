"""Behavior tests for motor_node's cmd_vel timeout / fault-stop safety logic.

Runs in dry-run mode (dry_run param) -- _set_channel is monkeypatched per
test to record commanded duty cycles instead of touching real GPIO.
"""
import pytest
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from bot_motor.motor_node import MotorNode


@pytest.fixture
def node():
    rclpy.init()
    n = MotorNode(parameter_overrides=[Parameter("dry_run", value=True)])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _capture_duty(node):
    calls = []
    node._set_channel = lambda chan, duty, min_duty=None: calls.append(duty)
    return calls


def _capture_floor(node):
    floors = []
    node._set_channel = lambda chan, duty, min_duty=None: floors.append(min_duty)
    return floors


def test_no_cmd_received_yet_stops(node):
    calls = _capture_duty(node)
    node._tick()
    assert calls == [0.0, 0.0]


def test_stale_cmd_vel_stops(node):
    msg = Twist()
    msg.linear.x = 1.0
    node._on_cmd_vel(msg)
    node._last_cmd_time -= node._timeout_s + 0.1  # simulate elapsed timeout
    calls = _capture_duty(node)
    node._tick()
    assert calls == [0.0, 0.0]


def test_system_fault_forces_stop_even_with_fresh_cmd(node):
    msg = Twist()
    msg.linear.x = 1.0
    node._on_cmd_vel(msg)
    node._on_fault(Bool(data=True))
    calls = _capture_duty(node)
    node._tick()
    assert calls == [0.0, 0.0]


def test_fresh_cmd_vel_drives_both_channels(node):
    msg = Twist()
    msg.linear.x = 2.0  # well above the duty floor
    msg.angular.z = 0.0
    node._on_cmd_vel(msg)
    calls = _capture_duty(node)
    node._tick()
    assert calls == pytest.approx([2.0 / node._max_v, 2.0 / node._max_v])  # v / max_linear_speed_mps


def test_angular_velocity_differentiates_channels(node):
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 1.0
    node._on_cmd_vel(msg)
    calls = _capture_duty(node)
    node._tick()
    assert calls[0] < 0 < calls[1]  # left channel reverses, right advances


class _FakeLgpio:
    """Stand-in for the lgpio module -- dry_run=True short-circuits
    _set_channel before it ever calls lgpio (self._h stays None), so
    testing the deadband/min_duty_cycle logic below needs to actually
    reach those calls instead."""
    def __init__(self):
        self.tx_pwm_calls = []

    def gpio_write(self, h, pin, level):
        pass

    def tx_pwm(self, h, pin, freq, duty):
        self.tx_pwm_calls.append(duty)


def _make_reachable(node, monkeypatch):
    """Point _set_channel at a fake lgpio and a non-None handle so its
    body actually runs, without touching real GPIO."""
    fake = _FakeLgpio()
    monkeypatch.setattr("bot_motor.motor_node.lgpio", fake)
    monkeypatch.setattr(node, "_h", object())
    return fake


def test_small_commanded_duty_is_clamped_to_min_duty_cycle(node, monkeypatch):
    fake = _make_reachable(node, monkeypatch)

    node._set_channel(node._chan_a, 0.01)  # much smaller than min_duty_cycle

    assert fake.tx_pwm_calls == [pytest.approx(node._min_duty * 100.0)]


def test_duty_above_min_duty_cycle_is_unaffected(node, monkeypatch):
    fake = _make_reachable(node, monkeypatch)

    node._set_channel(node._chan_a, 0.5)

    assert fake.tx_pwm_calls == [pytest.approx(50.0)]


def test_zero_duty_is_not_clamped_up_to_min_duty_cycle(node, monkeypatch):
    fake = _make_reachable(node, monkeypatch)

    node._set_channel(node._chan_a, 0.0)

    assert fake.tx_pwm_calls == [pytest.approx(0.0)]


def test_pivot_in_place_uses_turn_floor(node):
    msg = Twist()
    msg.angular.z = 1.0  # sides turn opposite ways
    node._on_cmd_vel(msg)
    floors = _capture_floor(node)
    node._tick()
    assert floors == [node._min_turn_duty, node._min_turn_duty]


def test_fast_arc_is_passed_through_unchanged(node):
    msg = Twist()
    msg.linear.x = 2.0
    msg.angular.z = 0.5  # both sides forward, both well above the floor
    node._on_cmd_vel(msg)
    calls = _capture_duty(node)
    node._tick()
    half = node._track_width / 2.0
    assert calls == pytest.approx([(2.0 - 0.5 * half) / node._max_v, (2.0 + 0.5 * half) / node._max_v])


def _capture_both(node):
    calls = []
    node._set_channel = lambda chan, duty, min_duty=None: calls.append((duty, min_duty))
    return calls


def test_slow_gentle_arc_keeps_its_wheel_ratio(node):
    """0.3 m/s with a 1 m radius (Q/E teleop): ~0.125 / 0.175 duty. Flooring
    each side to 0.25 would drive straight; scaling both keeps the arc."""
    msg = Twist()
    msg.linear.x = 0.3
    msg.angular.z = 0.3
    node._on_cmd_vel(msg)
    calls = _capture_both(node)
    node._tick()
    (left, lf), (right, rf) = calls
    assert right == pytest.approx(node._min_duty)             # outer side lifted to the floor
    d = 0.3 * node._track_width / 2.0
    assert left / right == pytest.approx((0.3 - d) / (0.3 + d))  # same ratio as asked
    assert lf == 0.0 and rf == 0.0                            # no per-side floor on top


def test_slow_straight_still_reaches_the_floor(node):
    msg = Twist()
    msg.linear.x = 0.2
    node._on_cmd_vel(msg)
    calls = _capture_both(node)
    node._tick()
    assert [d for d, _ in calls] == pytest.approx([node._min_duty, node._min_duty])


def test_small_pivot_duty_is_clamped_to_min_turn_duty_cycle(node, monkeypatch):
    fake = _make_reachable(node, monkeypatch)

    node._set_channel(node._chan_a, -0.08, node._min_turn_duty)  # ~1 rad/s pivot

    assert fake.tx_pwm_calls == [pytest.approx(node._min_turn_duty * 100.0)]
