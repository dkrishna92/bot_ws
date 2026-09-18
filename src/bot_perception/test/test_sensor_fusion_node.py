"""Behavior tests for sensor_fusion_node's frame transform and fuse/publish logic.

TF lookups are monkeypatched with synthetic TransformStamped objects so
these run without a real TF tree (robot_state_publisher/static publishers).
"""
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Point32, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import LookupException

from bot_perception.sensor_fusion_node import SensorFusionNode


@pytest.fixture
def node():
    rclpy.init()
    n = SensorFusionNode()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def _identity_transform():
    t = TransformStamped()
    t.transform.rotation.w = 1.0
    return t


def _translation_transform(x, y, z):
    t = TransformStamped()
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.w = 1.0
    return t


def test_transform_points_identity(node):
    node._tf_buffer.lookup_transform = lambda to_frame, from_frame, time: _identity_transform()
    pts = np.array([[1.0, 2.0, 3.0]])
    assert np.allclose(node._transform_points(pts, "camera_link", "lidar_link"), pts)


def test_transform_points_applies_translation(node):
    node._tf_buffer.lookup_transform = lambda to_frame, from_frame, time: _translation_transform(1.0, 0.0, 0.0)
    pts = np.array([[0.0, 0.0, 0.0]])
    out = node._transform_points(pts, "camera_link", "lidar_link")
    assert np.allclose(out, [[1.0, 0.0, 0.0]])


def test_transform_points_same_frame_is_noop(node):
    pts = np.array([[5.0, 6.0, 7.0]])
    assert np.allclose(node._transform_points(pts, "lidar_link", "lidar_link"), pts)


def test_transform_points_missing_tf_returns_empty(node):
    def _raise(*args, **kwargs):
        raise LookupException("no transform yet")
    node._tf_buffer.lookup_transform = _raise
    out = node._transform_points(np.array([[1.0, 2.0, 3.0]]), "camera_link", "lidar_link")
    assert out.shape == (0, 3)


def test_lidar_to_3d_filters_out_of_range_readings(node):
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.1, 5.0
    scan.angle_min, scan.angle_increment = 0.0, 0.0
    scan.ranges = [1.0, 0.0, 10.0]  # only the first reading is in range
    assert node._lidar_to_3d(scan).shape == (1, 3)


def test_fuse_and_publish_uses_point32_and_lidar_frame(node):
    node._tf_buffer.lookup_transform = lambda to_frame, from_frame, time: _identity_transform()
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.1, 5.0
    scan.angle_min, scan.angle_increment = 0.0, 0.0
    scan.ranges = [1.0]
    node._latest_scan = scan
    node._latest_depth = None

    published = []
    node._pub_pointcloud.publish = lambda msg: published.append(msg)
    node._fuse_and_publish()

    assert len(published) == 1
    assert isinstance(published[0].points[0], Point32)
    assert published[0].header.frame_id == node._lidar_frame
