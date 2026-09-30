"""Vision-based start signal detector.

Subscribes to the OAK-D's image topic (published by depthai_ros_driver on
real hardware, or bridged from Gazebo's camera_sensor in sim -- see
bringup.launch.py, which passes the right image_topic for each) and
publishes a latched Bool on start_signal once the course's start-signal arm
(see bot_gazebo's start_signal_arms model / scripts/start_signal.py) is
observed flipping from red to green.

Detecting an actual red->green *transition* rather than just thresholding
on green avoids false-triggering before the real signal flips. The node
only arms after (1) warmup_s of frames -- the OAK's auto-exposure washes
the first frames out -- and (2) red_confirm_frames consecutive red
frames; then confirm_frames consecutive green frames trigger. Both guards
came from the first real-signal test (2026-09-29, scripts/
test_start_signal.py): a washed-out first frame made the signal's blue
board read as 83% "green" and would have started the race at launch. The
real signal is a blue board with a green disc above a red one, so the
green hue range also stops at 80 (the board measured hue 100-105, drifting
to ~90 when overexposed). If the node starts while the signal already
shows green, it waits for red first -- it will not start on a signal it
never saw change.

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

import time

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, qos_profile_sensor_data
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
        # Upper hue 80, not 90: the real signal's blue board (hue 100-105)
        # drifted to ~90 when overexposed and read as green.
        self.declare_parameter("green_hsv_upper", [80, 255, 255])
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
        # Arming guards -- see the module docstring. Frames during warmup_s
        # (from the first frame) are ignored; then red must be seen for
        # red_confirm_frames in a row before green can trigger.
        self.declare_parameter("warmup_s", 2.0)
        self.declare_parameter("red_confirm_frames", 3)
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
        self._warmup_s = self.get_parameter("warmup_s").value
        self._red_confirm_frames = self.get_parameter("red_confirm_frames").value

        self._bridge = CvBridge()
        self._latched = False
        self._green_streak = 0
        self._red_streak = 0
        self._armed = False
        self._first_frame_time: float | None = None

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
        self._image_topic = self.get_parameter("image_topic").value
        self._sub = None
        self._subscribe()

    def _subscribe(self) -> None:
        if self._sub is None:
            # sensor_data QoS (BEST_EFFORT): camera image streams are
            # published best-effort, so a default RELIABLE sub would get no
            # frames and the start would never trigger.
            self._sub = self.create_subscription(Image, self._image_topic,
                                                 self._on_image, qos_profile_sensor_data)

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

    def _update(self, green_frac: float, red_frac: float, now: float) -> bool:
        """One frame's colour fractions -> True when the start should fire.
        The whole trigger rule lives here so scripts/test_start_signal.py can
        run exactly this against the real camera."""
        if self._first_frame_time is None:
            self._first_frame_time = now
        if now - self._first_frame_time < self._warmup_s:
            self._red_streak = self._green_streak = 0
            return False
        # A colour counts only if the other one is absent -- a stray green
        # reflection next to a still-red signal can't read as "green".
        is_red = red_frac > self._threshold and green_frac <= self._threshold
        is_green = green_frac > self._threshold and red_frac <= self._threshold
        if not self._armed:
            self._red_streak = self._red_streak + 1 if is_red else 0
            if self._red_streak >= self._red_confirm_frames:
                self._armed = True
                self.get_logger().info("Start signal reads RED -- armed, waiting for green.")
            return False
        self._green_streak = self._green_streak + 1 if is_green else 0
        return self._green_streak >= self._confirm_frames

    def _on_image(self, msg: Image) -> None:
        if self._latched:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = self._roi_hsv(frame)
        green_frac = self._color_fraction(hsv, self._green_lower, self._green_upper)
        red_frac = self._red_fraction(hsv)

        if self._update(green_frac, red_frac, time.monotonic()):
            self._latched = True
            self.get_logger().info(
                f"Start signal red->green confirmed (green={green_frac:.2f}, "
                f"red={red_frac:.2f}) -- triggering start."
            )
            # Latched (TRANSIENT_LOCAL) publish reaches late subscribers, so
            # stop decoding frames at camera rate once the race has started.
            self._pub.publish(Bool(data=True))
            if self._sub is not None:
                self.destroy_subscription(self._sub)
                self._sub = None
        else:
            self._pub.publish(Bool(data=False))

    def reset(self) -> None:
        """Call before a new run (e.g. from a service or a course-reset topic)."""
        self._latched = False
        self._green_streak = 0
        self._red_streak = 0
        self._armed = False
        self._first_frame_time = None
        self._subscribe()


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
