"""Behavior tests for watchdog_node's fault-computation logic.

Runs in dry-run mode (RPi.GPIO / pyserial absent or port unavailable) --
_tick's fault decision is exercised directly rather than through real GPIO
or MCU serial I/O.
"""
import time

import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from bot_safety.watchdog_node import WatchdogNode


@pytest.fixture
def node():
    rclpy.init()
    n = WatchdogNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _last_published(node):
    captured = []
    node._pub.publish = lambda msg: captured.append(msg.data)
    return captured


def test_no_scan_ever_received_faults(node):
    published = _last_published(node)
    node._tick()
    assert published == [True]


def test_fresh_scan_clears_lidar_fault(node):
    node._on_scan(LaserScan())
    published = _last_published(node)
    node._tick()
    assert published == [False]


def test_scan_going_stale_faults(node):
    node._on_scan(LaserScan())
    node._last_scan_time = node.get_clock().now() - rclpy.duration.Duration(
        seconds=node._scan_timeout_s + 0.5
    )
    published = _last_published(node)
    node._tick()
    assert published == [True]


def test_mcu_estop_line_forces_fault_even_with_fresh_scan(node):
    node._on_scan(LaserScan())
    node._mcu_estop_active = True
    node._check_mcu_status = lambda: True
    published = _last_published(node)
    node._tick()
    assert published == [True]
