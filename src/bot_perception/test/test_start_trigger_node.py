"""Behavior tests for start_trigger_node's HSV detection / latching logic."""
import numpy as np
import cv2
import pytest
import rclpy
from cv_bridge import CvBridge

from bot_perception.start_trigger_node import StartTriggerNode


@pytest.fixture
def node():
    rclpy.init()
    n = StartTriggerNode()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def _solid_frame(hsv_color, size=(100, 100)):
    hsv = np.zeros((*size, 3), dtype=np.uint8)
    hsv[..., 0], hsv[..., 1], hsv[..., 2] = hsv_color
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def test_detect_high_fraction_for_matching_color(node):
    frame = _solid_frame((60, 200, 200))  # within hsv_lower/upper range
    assert node._detect(frame) == pytest.approx(1.0, abs=0.05)


def test_detect_low_fraction_for_non_matching_color(node):
    frame = _solid_frame((0, 200, 200))  # red, outside configured hue range
    assert node._detect(frame) == pytest.approx(0.0, abs=0.05)


def test_stays_unlatched_without_detection(node):
    bridge = CvBridge()
    published = []
    node._pub.publish = lambda msg: published.append(msg.data)
    node._on_image(bridge.cv2_to_imgmsg(_solid_frame((0, 200, 200)), encoding="bgr8"))
    assert published[-1] is False


def test_latches_on_detection_and_stays_latched(node):
    bridge = CvBridge()
    published = []
    node._pub.publish = lambda msg: published.append(msg.data)

    node._on_image(bridge.cv2_to_imgmsg(_solid_frame((60, 200, 200)), encoding="bgr8"))
    assert published[-1] is True

    node._on_image(bridge.cv2_to_imgmsg(_solid_frame((0, 200, 200)), encoding="bgr8"))
    assert published[-1] is True  # stays latched despite a non-matching frame


def test_reset_clears_latch(node):
    bridge = CvBridge()
    node._on_image(bridge.cv2_to_imgmsg(_solid_frame((60, 200, 200)), encoding="bgr8"))
    assert node._latched is True
    node.reset()
    assert node._latched is False
