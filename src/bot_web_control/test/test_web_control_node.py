"""Pure-logic tests for web_control_node.py.

Builds WebControlNode via __new__ to bypass Node.__init__ (which would start
rclpy machinery, the HTTP server thread, and the /cmd_vel publisher) -- only
the PNG encoding, occupancy-grid rendering, teleop scaling/clamping, and
launch/stop process bookkeeping is under test here, not ROS or HTTP runtime
behavior.
"""
from __future__ import annotations

import threading
import zlib

from nav_msgs.msg import OccupancyGrid

from bot_web_control.web_control_node import WebControlNode, encode_grayscale_png


class _FakePublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


def _make_node(linear_speed=0.3, angular_speed=1.0):
    node = WebControlNode.__new__(WebControlNode)
    node._lock = threading.Lock()
    node._latest_map = None
    node._last_teleop_time = 0.0
    node._zeroed_since_stale = True
    node._proc = None
    node._proc_label = None
    node._linear_speed = linear_speed
    node._angular_speed = angular_speed
    node._workspace_root = ""
    node._use_sim = True
    node._world = "speed_course_cfr"
    node._checkpoint_path = "src/bot_bringup/config/maps/checkpoint"
    node._cmd_pub = _FakePublisher()
    return node


def _make_grid(data, width, height):
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.data = data
    return msg


def test_encode_grayscale_png_round_trips_via_zlib():
    width, height = 3, 2
    pixels = bytes([0, 128, 255, 205, 254, 0])
    png = encode_grayscale_png(width, height, pixels)

    assert png.startswith(b"\x89PNG\r\n\x1a\n")

    # Locate the IDAT chunk and decompress it back to raw scanlines to
    # confirm the pixel bytes actually made it through unmodified.
    idat_start = png.index(b"IDAT") + 4
    # Chunk length is the 4 bytes immediately before the tag.
    length = int.from_bytes(png[idat_start - 8:idat_start - 4], "big")
    idat_data = png[idat_start:idat_start + length]
    raw = zlib.decompress(idat_data)

    # Each row is prefixed with a filter-type byte (0 = None here).
    expected = bytearray()
    for row in range(height):
        expected.append(0)
        expected.extend(pixels[row * width:(row + 1) * width])
    assert bytes(raw) == bytes(expected)


def test_render_map_png_returns_none_without_a_map():
    node = _make_node()
    assert node.render_map_png() is None


def test_render_map_png_maps_occupancy_values_to_greyscale():
    node = _make_node()
    # -1 unknown, 0 free, 100 occupied -- one row so orientation doesn't
    # matter for this check (row-flip is exercised by size below).
    node._latest_map = _make_grid([-1, 0, 100], width=3, height=1)

    png = node.render_map_png()

    assert png is not None
    idat_start = png.index(b"IDAT") + 4
    length = int.from_bytes(png[idat_start - 8:idat_start - 4], "big")
    raw = zlib.decompress(png[idat_start:idat_start + length])
    row = raw[1:]  # strip the filter-type byte
    assert row[0] == 205  # unknown -> grey
    assert row[1] == 254  # free -> white
    assert row[2] == 0    # occupied -> black


def test_render_map_png_flips_rows_top_to_bottom():
    # Row 0 (occupancy convention: min-y) is all-free; row 1 is all-occupied.
    # Rendered image should show row 1's content on top (image row 0).
    node = _make_node()
    node._latest_map = _make_grid([0, 0, 100, 100], width=2, height=2)

    png = node.render_map_png()

    idat_start = png.index(b"IDAT") + 4
    length = int.from_bytes(png[idat_start - 8:idat_start - 4], "big")
    raw = zlib.decompress(png[idat_start:idat_start + length])
    top_row = raw[1:3]      # skip filter byte, first image row
    bottom_row = raw[4:6]   # skip filter byte, second image row
    assert top_row[0] == 0      # occupied (was grid row 1) now on top
    assert bottom_row[0] == 254  # free (was grid row 0) now on bottom


def test_on_teleop_cmd_scales_by_configured_speed():
    node = _make_node(linear_speed=0.5, angular_speed=2.0)
    node.on_teleop_cmd(1.0, -1.0)
    msg = node._cmd_pub.published[-1]
    assert msg.linear.x == 0.5
    assert msg.angular.z == -2.0


def test_on_teleop_cmd_clamps_out_of_range_fractions():
    node = _make_node(linear_speed=0.5, angular_speed=2.0)
    node.on_teleop_cmd(5.0, -5.0)
    msg = node._cmd_pub.published[-1]
    assert msg.linear.x == 0.5
    assert msg.angular.z == -2.0


def test_on_teleop_cmd_updates_watchdog_state():
    node = _make_node()
    assert node._zeroed_since_stale is True
    node.on_teleop_cmd(1.0, 0.0)
    assert node._zeroed_since_stale is False
    assert node._last_teleop_time > 0.0


def test_status_reports_idle_when_no_process_tracked():
    node = _make_node()
    status = node.status()
    assert status["running"] is None
    assert status["map_available"] is False


def test_status_reports_map_available_once_a_map_arrives():
    node = _make_node()
    node._latest_map = _make_grid([0], width=1, height=1)
    assert node.status()["map_available"] is True


class _FakePopen:
    """Captures the cmd it was constructed with instead of launching anything."""
    def __init__(self, cmd, **kwargs):
        self.cmd = cmd
        self.pid = 12345

    def poll(self):
        return None


def test_start_launch_mapping_without_resume_has_no_resume_args(monkeypatch):
    node = _make_node()
    node._checkpoint_path = "src/bot_bringup/config/maps/checkpoint"
    captured = {}
    monkeypatch.setattr(
        "bot_web_control.web_control_node.subprocess.Popen",
        lambda cmd, **kw: captured.setdefault("cmd", cmd) or _FakePopen(cmd, **kw),
    )

    ok, _ = node.start_launch("mapping", resume=False)

    assert ok is True
    assert "teleop:=true" in captured["cmd"]
    assert not any(a.startswith("resume_map:=") for a in captured["cmd"])
    assert not any(a.startswith("resume_pose:=") for a in captured["cmd"])


def test_start_launch_mapping_with_resume_forwards_checkpoint_and_pose(monkeypatch):
    node = _make_node()
    node._checkpoint_path = "src/bot_bringup/config/maps/checkpoint"
    captured = {}
    monkeypatch.setattr(
        "bot_web_control.web_control_node.subprocess.Popen",
        lambda cmd, **kw: captured.setdefault("cmd", cmd) or _FakePopen(cmd, **kw),
    )

    ok, _ = node.start_launch("mapping", resume=True, resume_pose="1.2,3.4,0.5")

    assert ok is True
    assert "resume_map:=src/bot_bringup/config/maps/checkpoint" in captured["cmd"]
    assert "resume_pose:=1.2,3.4,0.5" in captured["cmd"]


def test_start_launch_mapping_resume_defaults_pose_when_not_given(monkeypatch):
    node = _make_node()
    captured = {}
    monkeypatch.setattr(
        "bot_web_control.web_control_node.subprocess.Popen",
        lambda cmd, **kw: captured.setdefault("cmd", cmd) or _FakePopen(cmd, **kw),
    )

    ok, _ = node.start_launch("mapping", resume=True, resume_pose=None)

    assert ok is True
    assert "resume_pose:=0.0,0.0,0.0" in captured["cmd"]


def test_start_launch_refuses_when_already_running():
    node = _make_node()
    node._proc = _FakePopen(["already", "running"])
    node._proc_label = "bringup"

    ok, msg = node.start_launch("mapping")

    assert ok is False
    assert "bringup" in msg


class _FakeFuture:
    def __init__(self):
        self._callback = None

    def add_done_callback(self, cb):
        self._callback = cb
        cb(self)  # synchronous completion for the test


class _FakeSaveMapClient:
    def __init__(self, available=True):
        self._available = available
        self.last_request = None

    def wait_for_service(self, timeout_sec=0.0):
        return self._available

    def call_async(self, request):
        self.last_request = request
        return _FakeFuture()


def test_save_map_uses_checkpoint_path_by_default():
    node = _make_node()
    node._checkpoint_path = "src/bot_bringup/config/maps/checkpoint"
    node._save_map_client = _FakeSaveMapClient()

    ok, msg = node.save_map(None)

    assert ok is True
    assert node._save_map_client.last_request.name.data == "src/bot_bringup/config/maps/checkpoint"
    assert "checkpoint" in msg


def test_save_map_uses_explicit_name_when_given():
    node = _make_node()
    node._save_map_client = _FakeSaveMapClient()

    ok, _ = node.save_map("my_custom_map")

    assert ok is True
    assert node._save_map_client.last_request.name.data == "my_custom_map"


def test_save_map_reports_failure_when_service_unavailable():
    node = _make_node()
    node._save_map_client = _FakeSaveMapClient(available=False)

    ok, msg = node.save_map(None)

    assert ok is False
    assert "not available" in msg
