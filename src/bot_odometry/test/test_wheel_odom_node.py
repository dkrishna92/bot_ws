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
    # The Teensy sends whole tick counts; ticks_per_rev is fractional
    # (979.62), so one wheel turn is ~980 ticks.
    ticks = round(node._ticks_per_rev)
    node._on_line(f"E,{ticks},{ticks},100000")

    assert len(published) == 1
    msg = published[0]
    assert msg.pose.pose.position.x == pytest.approx(
        2.0 * math.pi * node._wheel_radius * ticks / node._ticks_per_rev, rel=1e-6
    )
    assert msg.pose.pose.position.y == pytest.approx(0.0, abs=1e-9)
    assert msg.twist.twist.angular.z == pytest.approx(0.0, abs=1e-9)


def test_unequal_tick_deltas_turn_in_place_sign(node):
    published = _last_published(node)
    node._on_line("E,0,0,0")
    node._last_sample_time -= 0.1
    node._on_line(f"E,0,{round(node._ticks_per_rev)},100000")

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
    node._on_line("U,1000,1000,0")
    for name, msgs in published.items():
        assert len(msgs) == 1
        assert msgs[0].range == pytest.approx(0.1715, rel=1e-3)
        assert msgs[0].header.frame_id == f"ultrasonic_{name}"


def test_ultrasonic_timeout_sentinel_reports_out_of_range(node):
    published = _capture_range_publishers(node)
    node._on_line("U,0,0,0")
    for msgs in published.values():
        assert msgs[0].range == pytest.approx(5.0)  # max_range (4.0) + 1.0 sentinel


def test_malformed_ultrasonic_line_is_ignored(node):
    published = _capture_range_publishers(node)
    node._on_line("U,not,a,0")
    for msgs in published.values():
        assert msgs == []


def test_fractional_tick_line_is_ignored(node):
    """The Teensy only sends integers; anything else is a corrupt line."""
    published = _last_published(node)
    node._on_line("E,0,0,0")
    node._last_sample_time -= 0.1
    node._on_line("E,979.62,979.62,100000")
    assert published == []


class _FakeSerial:
    """Bytes waiting on the Teensy port, read the way pyserial does."""

    def __init__(self):
        self.buf = b""

    @property
    def in_waiting(self):
        return len(self.buf)

    def read(self, n):
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def close(self):
        pass


def test_poll_drains_a_backlog_and_uses_only_the_newest_encoder_line(node):
    """2026-10-01: one readline per tick fell 2.4 s behind under load. A
    backlog must be consumed in one poll, from the newest E line only."""
    published = _last_published(node)
    ranges = []
    node._range_publishers["left"].publish = lambda msg: ranges.append(msg)
    node._teensy_serial = port = _FakeSerial()
    port.buf = b"E,0,0,0\n"
    node._poll_serial()
    node._last_sample_time -= 0.1
    ticks = round(node._ticks_per_rev)
    port.buf = b"".join(f"E,{i * 10},{i * 10},{i}\nU,1000,1000,{i}\n".encode() for i in range(1, 60))
    port.buf += f"E,{ticks},{ticks},100000\nE,{ticks + 5},{ticks + 5},100".encode()  # last line partial
    node._poll_serial()

    assert port.in_waiting == 0
    assert len(published) == 1  # one odometry update for the whole backlog
    assert published[0].pose.pose.position.x == pytest.approx(
        2.0 * math.pi * node._wheel_radius * ticks / node._ticks_per_rev, rel=1e-6)
    assert len(ranges) == 59  # ultrasonic lines all still published
    assert node._rx == f"E,{ticks + 5},{ticks + 5},100".encode()  # kept for the next poll


def test_poll_with_nothing_waiting_does_nothing(node):
    published = _last_published(node)
    node._teensy_serial = _FakeSerial()
    node._poll_serial()
    assert published == []
