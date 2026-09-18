"""Behavior tests for ultrasonic_node's Range message construction.

Runs in dry-run mode (RPi.GPIO absent) -- _read_one is monkeypatched per
test so _tick's message-building logic is exercised without real GPIO
pulse timing (which would be flaky to simulate here).
"""
import pytest
import rclpy

from bot_ultrasonic.ultrasonic_node import UltrasonicNode


@pytest.fixture
def node():
    rclpy.init()
    n = UltrasonicNode()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def _capture_publishers(node):
    captured = {name: [] for name in node._range_publishers}
    for name, pub in node._range_publishers.items():
        pub.publish = lambda msg, _bucket=captured[name]: _bucket.append(msg)
    return captured


def test_valid_reading_is_published_as_is(node):
    published = _capture_publishers(node)
    node._read_one = lambda trigger, echo: 1.23
    node._tick()
    for msgs in published.values():
        assert msgs[0].range == pytest.approx(1.23)


def test_out_of_range_reading_uses_sentinel(node):
    published = _capture_publishers(node)
    node._read_one = lambda trigger, echo: None
    node._tick()
    for msgs in published.values():
        assert msgs[0].range == pytest.approx(node._max_range + 1.0)


def test_message_metadata_matches_sensor(node):
    published = _capture_publishers(node)
    node._read_one = lambda trigger, echo: 0.5
    node._tick()
    for name, msgs in published.items():
        msg = msgs[0]
        assert msg.header.frame_id == f"ultrasonic_{name}"
        assert msg.min_range == pytest.approx(0.02)
        assert msg.max_range == pytest.approx(node._max_range)
        assert msg.field_of_view == pytest.approx(node._fov)
        assert msg.radiation_type == msg.ULTRASOUND
