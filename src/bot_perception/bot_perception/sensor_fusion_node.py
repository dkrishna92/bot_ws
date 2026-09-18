"""Sensor fusion node: combines RPLidar 2D scans and OAK-D S2 RGB/depth.

Subscribes to:
- /scan (sensor_msgs/LaserScan) from rplidar_ros
- /oak/rgb/image_raw (sensor_msgs/Image) from depthai_ros_driver
- /oak/spatial_detections (... or spatial depth results)

Publishes:
- /perception/obstacles (geometry_msgs/PointCloud) - fused 3D points from lidar + depth
- /perception/detections (custom msg with spatial positions in 3D)

This node demonstrates sensor fusion by combining the planar lidar scan with
depth/RGB from the OAK-D to build a richer 3D understanding of the environment
for Nav2's costmap and obstacle avoidance logic.
"""
from __future__ import annotations

import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Image, PointCloud, PointField
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

try:
    import depthai as dai
except ImportError:
    dai = None


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__("sensor_fusion_node")

        self.declare_parameter("lidar_frame_id", "lidar_link")
        # camera_link is the only camera frame currently in bot.urdf.xacro --
        # update once depthai_ros_driver's real stereo depth frame is wired in.
        self.declare_parameter("depth_frame_id", "camera_link")
        self.declare_parameter("buffer_size", 5)
        self.declare_parameter("depth_confidence_threshold", 200)

        self._lidar_frame = self.get_parameter("lidar_frame_id").value
        self._depth_frame = self.get_parameter("depth_frame_id").value
        self._buffer_size = self.get_parameter("buffer_size").value
        self._depth_confidence = self.get_parameter("depth_confidence_threshold").value

        self._bridge = CvBridge()
        self._latest_scan: LaserScan | None = None
        self._latest_depth: np.ndarray | None = None
        self._scan_buffer = deque(maxlen=self._buffer_size)
        self._depth_buffer = deque(maxlen=self._buffer_size)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub_pointcloud = self.create_publisher(
            PointCloud, "perception/obstacles", 10
        )

        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.create_subscription(Image, "/oak/rgb/image_raw", self._on_rgb, 10)
        # Note: depth image subscribe would typically be to /oak/stereo/depth or similar
        self.create_subscription(Image, "/oak/stereo/depth", self._on_depth, 10)

        self.create_timer(0.1, self._fuse_and_publish)

    def _on_scan(self, msg: LaserScan) -> None:
        """Store latest RPLidar scan."""
        self._latest_scan = msg
        self._scan_buffer.append(msg)

    def _on_rgb(self, msg: Image) -> None:
        """RGB image callback (currently unused, but available for future CV work)."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Could do object detection here, store bounding boxes, etc.
        except Exception as e:
            self.get_logger().warn(f"RGB conversion error: {e}")

    def _on_depth(self, msg: Image) -> None:
        """Store latest depth map from OAK-D."""
        try:
            # Depth images are typically uint16 (millimeters)
            depth_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self._latest_depth = depth_frame
            self._depth_buffer.append(depth_frame)
        except Exception as e:
            self.get_logger().warn(f"Depth conversion error: {e}")

    def _lidar_to_3d(self, scan: LaserScan, z: float = 0.1) -> np.ndarray:
        """Convert 2D lidar scan to 3D points (assumes flat ground at z=0.1m)."""
        points = []
        for i, range_val in enumerate(scan.ranges):
            if scan.range_min <= range_val <= scan.range_max:
                angle = scan.angle_min + i * scan.angle_increment
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                points.append([x, y, z])
        return np.array(points) if points else np.zeros((0, 3))

    def _depth_to_3d(self, depth_frame: np.ndarray) -> np.ndarray:
        """Convert depth map to 3D points using intrinsics.

        OAK-D intrinsics vary by model -- this is a placeholder that assumes
        a standard depth camera FOV. Replace with actual intrinsic matrix
        once camera calibration is available.
        """
        if depth_frame is None:
            return np.zeros((0, 3))

        # Placeholder intrinsics (typical for OAK-D at VGA resolution)
        fx, fy = 382.0, 382.0
        cx, cy = 320.0, 240.0

        h, w = depth_frame.shape
        points = []

        for v in range(0, h, 2):  # Skip every other pixel for efficiency
            for u in range(0, w, 2):
                d = depth_frame[v, u] / 1000.0  # Convert mm to meters
                if 0.1 < d < 5.0:  # Filter out invalid/out-of-range
                    x = (u - cx) * d / fx
                    y = (v - cy) * d / fy
                    z = d
                    points.append([x, y, z])

        return np.array(points) if points else np.zeros((0, 3))

    def _transform_points(self, points: np.ndarray, from_frame: str, to_frame: str) -> np.ndarray:
        """Transform a (N,3) point array between TF frames via a single lookup."""
        if points.shape[0] == 0 or from_frame == to_frame:
            return points
        try:
            t = self._tf_buffer.lookup_transform(to_frame, from_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(f"no transform {from_frame} -> {to_frame} yet: {exc}")
            return np.zeros((0, 3))
        tr = t.transform.translation
        q = t.transform.rotation
        rotation = np.array([
            [1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
            [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x ** 2 + q.z ** 2), 2 * (q.y * q.z - q.x * q.w)],
            [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x ** 2 + q.y ** 2)],
        ])
        return points @ rotation.T + np.array([tr.x, tr.y, tr.z])

    def _fuse_and_publish(self) -> None:
        """Fuse lidar and depth data, publish as PointCloud."""
        if self._latest_scan is None:
            return

        # Get 3D points from lidar (assumes planar environment)
        lidar_points = self._lidar_to_3d(self._latest_scan)

        # Get 3D points from depth camera, transformed out of the camera frame
        # into the lidar frame so the two point sets are actually comparable
        depth_points_cam = (
            self._depth_to_3d(self._latest_depth)
            if self._latest_depth is not None
            else np.zeros((0, 3))
        )
        depth_points = self._transform_points(depth_points_cam, self._depth_frame, self._lidar_frame)

        # Combine: depth-based 3D + lidar planar points
        all_points = np.vstack([depth_points, lidar_points])

        if all_points.shape[0] == 0:
            return

        # Build PointCloud message
        msg = PointCloud()
        msg.header = Header()
        msg.header.stamp = self._latest_scan.header.stamp
        msg.header.frame_id = self._lidar_frame

        # Point32, not Point -- PointCloud.points requires geometry_msgs/Point32
        msg.points = [
            Point32(x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
            for pt in all_points
        ]

        self._pub_pointcloud.publish(msg)

    def destroy_node(self) -> None:
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
