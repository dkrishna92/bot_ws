"""Behavior tests for wheel_odom_node's tick-to-odometry math.

Runs in dry-run mode (pyserial absent or port unavailable) -- _on_line's
integration is exercised directly rather than through real serial I/O.
"""
import math

import pytest
import rclpy

from bot_odometry.wheel_odom_node import WheelOdomNode


@pytest.fixture
def node():
    rclpy.init()
    n = WheelOdomNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _last_published(node):
    captured = []
    node._pub.publish = lambda msg: captured.append(msg)
    return captured


def test_first_line_seeds_state_without_publishing(node):
    published = _last_published(node)
    node._on_line("E,0,0,1000")
    assert published == []


def test_equal_tick_deltas_drive_straight(node):
    published = _last_published(node)
    node._on_line("E,0,0,0")
    node._last_sample_time -= 0.1  # force a positive dt without a real sleep
    node._on_line(f"E,{node._ticks_per_rev},{node._ticks_per_rev},100000")

    assert len(published) == 1
    msg = published[0]
    assert msg.pose.pose.position.x == pytest.approx(
        2.0 * math.pi * node._wheel_radius, rel=1e-6
    )
    assert msg.pose.pose.position.y == pytest.approx(0.0, abs=1e-9)
    assert msg.twist.twist.angular.z == pytest.approx(0.0, abs=1e-9)


def test_unequal_tick_deltas_turn_in_place_sign(node):
    published = _last_published(node)
    node._on_line("E,0,0,0")
    node._last_sample_time -= 0.1
    node._on_line(f"E,0,{node._ticks_per_rev},100000")

    assert len(published) == 1
    assert published[0].twist.twist.angular.z > 0.0


def test_malformed_line_is_ignored(node):
    published = _last_published(node)
    node._on_line("garbage")
    assert published == []


def _capture_range_publishers(node):
    captured = {name: [] for name in node._range_publishers}
    for name, pub in node._range_publishers.items():
        pub.publish = lambda msg, _bucket=captured[name]: _bucket.append(msg)
    return captured


def test_ultrasonic_line_publishes_converted_distances(node):
    published = _capture_range_publishers(node)
    # 1000us round trip -> ~0.1715m at 343 m/s
    node._on_line("U,1000,1000,1000,0")
    for name, msgs in published.items():
        assert len(msgs) == 1
        assert msgs[0].range == pytest.approx(0.1715, rel=1e-3)
        assert msgs[0].header.frame_id == f"ultrasonic_{name}"


def test_ultrasonic_timeout_sentinel_reports_out_of_range(node):
    published = _capture_range_publishers(node)
    node._on_line("U,0,0,0,0")
    for msgs in published.values():
        assert msgs[0].range == pytest.approx(5.0)  # max_range (4.0) + 1.0 sentinel


def test_malformed_ultrasonic_line_is_ignored(node):
    published = _capture_range_publishers(node)
    node._on_line("U,not,a,number,0")
    for msgs in published.values():
        assert msgs == []
