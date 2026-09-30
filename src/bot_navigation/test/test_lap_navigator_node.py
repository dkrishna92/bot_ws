"""Behavior tests for lap_navigator_node's route-building and trigger logic."""
import math

import pytest
import rclpy
from std_msgs.msg import Bool

from bot_navigation.lap_navigator_node import LapNavigatorNode, _build_route, _route_for_course


@pytest.fixture
def node():
    rclpy.init()
    n = LapNavigatorNode()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_build_route_repeats_checkpoints_per_lap():
    checkpoints = [(0.0, 0.0), (1.0, 0.0)]
    route = _build_route(checkpoints, num_laps=3)
    assert len(route) == 6
    assert [(x, y) for x, y, _ in route] == checkpoints * 3


def test_build_route_yaw_points_toward_next_checkpoint():
    checkpoints = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    route = _build_route(checkpoints, num_laps=1)
    # 0,0 -> 1,0 : heading is +X (yaw 0)
    assert route[0][2] == pytest.approx(0.0)
    # 1,0 -> 1,1 : heading is +Y (yaw pi/2)
    assert route[1][2] == pytest.approx(math.pi / 2)
    # 1,1 -> wraps back to 0,0 : heading is -X,-Y (yaw -3pi/4)
    assert route[2][2] == pytest.approx(-3 * math.pi / 4)


def test_known_course_loads_a_nonempty_route(node):
    assert len(node._route) > 0


def test_route_for_unknown_course_is_empty():
    assert _route_for_course("not_a_real_course", num_laps_override=0) == []


def test_route_for_known_course_uses_course_default_lap_count():
    route = _route_for_course("obstacle_course_cfr", num_laps_override=0)
    checkpoints_per_lap = 7  # len(_COURSE_CHECKPOINTS["obstacle_course_cfr"])
    assert len(route) == checkpoints_per_lap * 2  # obstacle course is 2 laps


def test_num_laps_override_wins_over_course_default():
    route = _route_for_course("obstacle_course_cfr", num_laps_override=5)
    checkpoints_per_lap = 7
    assert len(route) == checkpoints_per_lap * 5


def _stub_nav_ready(node, ready: bool):
    node._nav_client.wait_for_server = lambda timeout_sec=0: ready


def test_does_not_send_before_start_signal(node):
    sent = []
    node._send_route = lambda: sent.append(True)
    _stub_nav_ready(node, True)

    node._on_start_signal(Bool(data=False))
    assert sent == []
    assert node._sent is False


def test_sends_once_on_start_signal_and_ignores_repeats(node):
    sent = []
    node._send_route = lambda: sent.append(True)
    _stub_nav_ready(node, True)

    node._on_start_signal(Bool(data=True))
    assert sent == [True]
    assert node._sent is True

    node._on_start_signal(Bool(data=True))
    assert sent == [True]  # not sent again


def test_waits_for_nav2_then_sends_when_server_comes_up(node):
    """Nav2's bt_navigator may still be starting when the signal fires --
    the route must be held and retried, not dropped."""
    sent = []
    node._send_route = lambda: sent.append(True)
    _stub_nav_ready(node, False)

    node._on_start_signal(Bool(data=True))
    assert sent == []            # nothing sent yet
    assert node._pending is True  # but the run is queued
    assert node._sent is False

    node._try_send()             # timer tick, server still down
    assert sent == []

    _stub_nav_ready(node, True)
    node._try_send()             # timer tick, server now up
    assert sent == [True]
    assert node._sent is True
    assert node._pending is False


def test_route_file_replaces_builtin_checkpoints(tmp_path):
    route = tmp_path / "speed.yaml"
    route.write_text("laps: 2\ncheckpoints:\n  - [0.0, 0.0]\n  - [5.0, 0.0]\n  - [5.0, 3.0]\n")
    rclpy.init(args=["--ros-args", "-p", f"route_file:={route}"])
    try:
        n = LapNavigatorNode()
        assert len(n._route) == 6  # 3 checkpoints x 2 laps from the file
        assert n._route[0][:2] == (0.0, 0.0) and n._route[2][:2] == (5.0, 3.0)
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_unreadable_route_file_loads_no_route(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"route_file:={tmp_path / 'missing.yaml'}"])
    try:
        n = LapNavigatorNode()
        assert n._route == []
        n.destroy_node()
    finally:
        rclpy.shutdown()


class _Result:
    def __init__(self, status):
        self.status = status


class _Future:
    def __init__(self, status):
        self._r = _Result(status)

    def result(self):
        return self._r


def test_abort_resends_from_first_unreached_checkpoint(node):
    from action_msgs.msg import GoalStatus
    total = len(node._route)
    node._sent, node._sent_from, node._remaining = True, 0, total - 2  # reached 2 checkpoints

    node._on_result(_Future(GoalStatus.STATUS_ABORTED))

    assert node._next_index == 2
    assert node._pending is True and node._sent is False
    assert node._retries_left == node.get_parameter("max_retries").value - 1


def test_success_and_cancel_do_not_resend(node):
    from action_msgs.msg import GoalStatus
    for status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
        node._sent, node._pending = True, False
        node._on_result(_Future(status))
        assert node._pending is False


def test_gives_up_when_retries_run_out(node):
    from action_msgs.msg import GoalStatus
    node._retries_left = 0
    node._sent, node._pending, node._remaining = True, False, 1
    node._on_result(_Future(GoalStatus.STATUS_ABORTED))
    assert node._pending is False
