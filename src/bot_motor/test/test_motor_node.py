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
    node._set_channel = lambda chan, duty: calls.append(duty)
    return calls


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
    msg.linear.x = 1.0
    msg.angular.z = 0.0
    node._on_cmd_vel(msg)
    calls = _capture_duty(node)
    node._tick()
    assert calls == pytest.approx([0.5, 0.5])  # v / max_linear_speed_mps (2.0)


def test_angular_velocity_differentiates_channels(node):
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 1.0
    node._on_cmd_vel(msg)
    calls = _capture_duty(node)
    node._tick()
    assert calls[0] < 0 < calls[1]  # left channel reverses, right advances
