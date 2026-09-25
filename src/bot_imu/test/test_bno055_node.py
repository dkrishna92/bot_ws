"""Behavior tests for bno055_node's Imu message construction.

Runs in dry-run mode (smbus2 absent) -- _read_i16 is monkeypatched per
test so _tick's message-building logic is exercised without a real I2C
bus (which would be flaky/impossible to simulate here).
"""
import pytest
import rclpy

from bot_imu.bno055_node import Bno055Node, _QUA_LSB_PER_UNIT


@pytest.fixture
def node():
    rclpy.init()
    n = Bno055Node()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


class _FakeBus:
    def close(self):
        pass


def _capture_publisher(node):
    captured = []
    node._publisher.publish = lambda msg: captured.append(msg)
    return captured


def test_tick_is_noop_without_a_bus(node):
    captured = _capture_publisher(node)
    node._tick()
    assert captured == []


def test_quaternion_and_rates_are_scaled_and_published(node):
    captured = _capture_publisher(node)
    node._bus = _FakeBus()  # any non-None sentinel; _read_i16 is stubbed below

    raw_by_addr = {
        0x20: int(1.0 / _QUA_LSB_PER_UNIT),  # w
        0x22: 0,  # x
        0x24: 0,  # y
        0x26: 0,  # z
        0x14: 900,  # gyro x -> 1.0 rad/s (900 LSB/rps)
        0x16: 0,
        0x18: 0,
        0x08: 100,  # accel x -> 1.0 m/s^2 (100 LSB per m/s^2)
        0x0A: 0,
        0x0C: 0,
    }
    node._read_i16 = lambda addr: raw_by_addr[addr]

    node._tick()

    assert len(captured) == 1
    msg = captured[0]
    assert msg.header.frame_id == "imu_link"
    assert msg.orientation.w == pytest.approx(1.0)
    assert msg.angular_velocity.x == pytest.approx(1.0)
    assert msg.linear_acceleration.x == pytest.approx(1.0)
    assert msg.orientation_covariance[0] == pytest.approx(0.02 ** 2)
