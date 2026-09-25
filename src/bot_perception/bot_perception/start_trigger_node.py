"""Vision-based start signal detector.

Subscribes to the OAK-D's image topic (published by depthai_ros_driver on
real hardware, or bridged from Gazebo's camera_sensor in sim -- see
bringup.launch.py, which passes the right image_topic for each) and
publishes a latched Bool on start_signal once the course's start-signal arm
(see bot_gazebo's start_signal_arms model / scripts/start_signal.py) is
observed flipping from red to green.

Detecting an actual red->green *transition* (both a red baseline and a
green confirmation, held for confirm_frames consecutive frames) rather than
just thresholding on green avoids false-triggering on a stray green pixel
in the background before the real signal flips.

This node only handles detection -- bot_navigation's lap_navigator_node
subscribes to start_signal and owns actually sending Nav2 the race route,
keeping vision detection and course/lap logic in separate packages (see
CLAUDE.md's per-concern package convention).

TODO: the ROI/HSV defaults below were measured empirically against the sim
(speed_course_cfr, from that world's default spawn pose) -- they depend on
where the signal happens to land in frame from that particular spawn, so
they are NOT portable as-is to the real course. Recapture a frame from the
real OAK-D at the real start position and retune both the ROI and the HSV
ranges against actual ambient lighting and the camera's color response
before race day.
"""
from __future__ import annotations

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class StartTriggerNode(Node):
    def __init__(self):
        super().__init__("start_trigger_node")

        # x1,y1,x2,y2 normalized. Measured against the sim: from the speed
        # course spawn the signal arm lands upper-LEFT of frame (the arm is
        # ~2m ahead and ~0.9m to the robot's left), not centered, and it
        # fills ~25% of this window. The lower bound matters as much as the
        # upper one -- this world's ground plane is itself green (it starts
        # at y~0.29 in frame and matches the green range below), so an ROI
        # reaching that far down would read "green" permanently and trigger
        # the instant the node starts.
        self.declare_parameter("roi", [0.10, 0.00, 0.34, 0.20])
        # S/V floors are deliberately low: the arm renders shaded in sim,
        # measured well under a floor of 80 (which detected only 0.3% of
        # the green arm vs 25.5% at these bounds).
        self.declare_parameter("green_hsv_lower", [35, 60, 40])
        self.declare_parameter("green_hsv_upper", [90, 255, 255])
        # Red wraps around hue 0/180 in OpenCV's 0-180 hue range, so it
        # needs two ranges (one near 0, one near 180) rather than one.
        self.declare_parameter("red_hsv_lower1", [0, 60, 40])
        self.declare_parameter("red_hsv_upper1", [12, 255, 255])
        self.declare_parameter("red_hsv_lower2", [168, 60, 40])
        self.declare_parameter("red_hsv_upper2", [180, 255, 255])
        self.declare_parameter("detect_threshold", 0.15)
        # Consecutive matching frames required before latching -- debounces
        # a single-frame flicker/misread into a real transition, similar in
        # spirit to feather_ws's switch debounce (see CLAUDE.md).
        self.declare_parameter("confirm_frames", 3)
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")

        self._roi = self.get_parameter("roi").value
        self._green_lower = np.array(self.get_parameter("green_hsv_lower").value)
        self._green_upper = np.array(self.get_parameter("green_hsv_upper").value)
        self._red_lower1 = np.array(self.get_parameter("red_hsv_lower1").value)
        self._red_upper1 = np.array(self.get_parameter("red_hsv_upper1").value)
        self._red_lower2 = np.array(self.get_parameter("red_hsv_lower2").value)
        self._red_upper2 = np.array(self.get_parameter("red_hsv_upper2").value)
        self._threshold = self.get_parameter("detect_threshold").value
        self._confirm_frames = self.get_parameter("confirm_frames").value

        self._bridge = CvBridge()
        self._latched = False
        self._green_streak = 0

        # transient_local + depth 1 makes this an actually latched topic --
        # a subscriber started after the trigger already fired (e.g. a BT
        # node) still gets the last published value instead of only seeing
        # it if it happened to already be listening at the moment it changed.
        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(Bool, "start_signal", latched_qos)
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                  self._on_image, 10)

    def _roi_hsv(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = self._roi
        roi = frame[int(y1 * h):int(y2 * h), int(x1 * w):int(x2 * w)]
        return cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    @staticmethod
    def _color_fraction(hsv: np.ndarray, lower, upper) -> float:
        mask = cv2.inRange(hsv, lower, upper)
        return float(np.count_nonzero(mask)) / mask.size

    def _red_fraction(self, hsv: np.ndarray) -> float:
        mask1 = cv2.inRange(hsv, self._red_lower1, self._red_upper1)
        mask2 = cv2.inRange(hsv, self._red_lower2, self._red_upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        return float(np.count_nonzero(mask)) / mask.size

    def _detect(self, frame: np.ndarray) -> float:
        """Green-channel fraction in the ROI."""
        return self._color_fraction(self._roi_hsv(frame), self._green_lower, self._green_upper)

    def _on_image(self, msg: Image) -> None:
        if self._latched:
            self._pub.publish(Bool(data=True))
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = self._roi_hsv(frame)
        green_frac = self._color_fraction(hsv, self._green_lower, self._green_upper)
        red_frac = self._red_fraction(hsv)

        # Require the arm to actually read as green AND no longer read as
        # red -- not just "some green pixels appeared somewhere in the
        # ROI" -- so a stray green reflection/background object next to a
        # still-red arm can't false-trigger the start.
        transitioning = green_frac > self._threshold and red_frac <= self._threshold
        self._green_streak = self._green_streak + 1 if transitioning else 0

        if self._green_streak >= self._confirm_frames:
            self._latched = True
            self.get_logger().info(
                f"Start signal red->green confirmed (green={green_frac:.2f}, "
                f"red={red_frac:.2f}) -- triggering start."
            )
            self._pub.publish(Bool(data=True))
        else:
            self._pub.publish(Bool(data=False))

    def reset(self) -> None:
        """Call before a new run (e.g. from a service or a course-reset topic)."""
        self._latched = False
        self._green_streak = 0


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
