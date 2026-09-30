"""Behavior tests for start_trigger_node's HSV transition / latching logic."""
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
    n._warmup_s = 0.0  # frames arrive instantly here; warmup has its own test
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def _solid_frame(hsv_color, size=(100, 100)):
    hsv = np.zeros((*size, 3), dtype=np.uint8)
    hsv[..., 0], hsv[..., 1], hsv[..., 2] = hsv_color
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def _feed(node, bridge, hsv_color):
    node._on_image(bridge.cv2_to_imgmsg(_solid_frame(hsv_color), encoding="bgr8"))


def test_detect_high_fraction_for_matching_color(node):
    frame = _solid_frame((60, 200, 200))  # within green_hsv_lower/upper range
    assert node._detect(frame) == pytest.approx(1.0, abs=0.05)


def test_detect_low_fraction_for_non_matching_color(node):
    frame = _solid_frame((0, 200, 200))  # red, outside the green hue range
    assert node._detect(frame) == pytest.approx(0.0, abs=0.05)


def test_red_frame_never_latches(node):
    bridge = CvBridge()
    published = []
    node._pub.publish = lambda msg: published.append(msg.data)

    for _ in range(10):
        _feed(node, bridge, (0, 200, 200))  # solid red the whole time

    assert node._latched is False
    assert published[-1] is False


def test_single_green_frame_does_not_latch(node):
    """A lone matching frame shouldn't trigger -- confirm_frames debounces
    against a one-frame flicker/false read."""
    bridge = CvBridge()
    published = []
    node._pub.publish = lambda msg: published.append(msg.data)

    _feed(node, bridge, (60, 200, 200))

    assert node._latched is False
    assert published[-1] is False


def test_latches_after_confirm_frames_of_red_to_green_transition(node):
    bridge = CvBridge()
    published = []
    node._pub.publish = lambda msg: published.append(msg.data)

    for _ in range(5):
        _feed(node, bridge, (0, 200, 200))  # red baseline
    assert node._latched is False

    confirm_frames = node.get_parameter("confirm_frames").value
    for _ in range(confirm_frames):
        _feed(node, bridge, (60, 200, 200))  # flips to green

    assert node._latched is True
    assert published[-1] is True


def test_stays_latched_after_trigger_even_if_signal_reverts(node):
    bridge = CvBridge()
    published = []
    node._pub.publish = lambda msg: published.append(msg.data)

    for _ in range(5):
        _feed(node, bridge, (0, 200, 200))  # red baseline
    confirm_frames = node.get_parameter("confirm_frames").value
    for _ in range(confirm_frames):
        _feed(node, bridge, (60, 200, 200))
    assert node._latched is True

    _feed(node, bridge, (0, 200, 200))  # reverts to red after the fact
    assert published[-1] is True  # stays latched


def test_reset_clears_latch_and_streak(node):
    bridge = CvBridge()
    for _ in range(5):
        _feed(node, bridge, (0, 200, 200))
    confirm_frames = node.get_parameter("confirm_frames").value
    for _ in range(confirm_frames):
        _feed(node, bridge, (60, 200, 200))
    assert node._latched is True

    node.reset()
    assert node._latched is False
    assert node._green_streak == 0
    assert node._armed is False


def test_green_without_seeing_red_first_never_latches(node):
    """Real-signal test 2026-09-29: a washed-out first frame read as green.
    Without a red baseline first, nothing may trigger."""
    bridge = CvBridge()
    for _ in range(20):
        _feed(node, bridge, (60, 200, 200))
    assert node._latched is False


def test_frames_during_warmup_are_ignored(node):
    node._warmup_s = 2.0
    t = [100.0]
    # red + green inside the warmup window must not arm or trigger
    for colour in [(0, 200, 200)] * 5 + [(60, 200, 200)] * 5:
        assert node._update(*_fractions(node, colour), t[0]) is False
        t[0] += 0.05
    assert node._armed is False
    # after warmup: red arms, then green triggers
    t[0] = 103.0
    for _ in range(node._red_confirm_frames):
        node._update(*_fractions(node, (0, 200, 200)), t[0])
    assert node._armed is True
    fired = [node._update(*_fractions(node, (60, 200, 200)), t[0]) for _ in range(node._confirm_frames)]
    assert fired[-1] is True


def _fractions(node, hsv_color):
    hsv = node._roi_hsv(_solid_frame(hsv_color))
    return (node._color_fraction(hsv, node._green_lower, node._green_upper), node._red_fraction(hsv))


@pytest.mark.parametrize("hue", [90, 102])
def test_blue_board_does_not_read_as_green(node, hue):
    """The real signal's board is blue (hue 100-105, ~90 when overexposed)."""
    green, _ = _fractions(node, (hue, 200, 250))
    assert green == pytest.approx(0.0, abs=0.01)
