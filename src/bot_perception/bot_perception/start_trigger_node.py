"""Vision-based start signal detector.

Subscribes to the OAK-D's image topic (published by depthai_ros_driver) and
publishes a latched Bool once the start signal is detected. Path follower /
Nav2 behavior tree should gate on this topic before beginning a run.

TODO: same caveat as the earlier prototype -- confirm what the actual start
signal looks like once course details are available, and retune the HSV
range / ROI in config/perception_params.yaml accordingly.
"""
from __future__ import annotations

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class StartTriggerNode(Node):
    def __init__(self):
        super().__init__("start_trigger_node")

        self.declare_parameter("roi", [0.35, 0.1, 0.65, 0.4])  # x1,y1,x2,y2 normalized
        self.declare_parameter("hsv_lower", [35, 80, 80])
        self.declare_parameter("hsv_upper", [85, 255, 255])
        self.declare_parameter("detect_threshold", 0.15)
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")

        self._roi = self.get_parameter("roi").value
        self._hsv_lower = np.array(self.get_parameter("hsv_lower").value)
        self._hsv_upper = np.array(self.get_parameter("hsv_upper").value)
        self._threshold = self.get_parameter("detect_threshold").value

        self._bridge = CvBridge()
        self._latched = False
        self._pub = self.create_publisher(Bool, "start_signal", 10)
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                  self._on_image, 10)

    def _detect(self, frame: np.ndarray) -> float:
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = self._roi
        roi = frame[int(y1 * h):int(y2 * h), int(x1 * w):int(x2 * w)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._hsv_lower, self._hsv_upper)
        return float(np.count_nonzero(mask)) / mask.size

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frac = self._detect(frame)
        detected = frac > self._threshold
        # latch once detected -- one-time event per run, don't flicker off
        self._latched = self._latched or detected
        self._pub.publish(Bool(data=self._latched))

    def reset(self) -> None:
        """Call before a new run (e.g. from a service or a course-reset topic)."""
        self._latched = False


def main(args=None):
    rclpy.init(args=args)
    node = StartTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
