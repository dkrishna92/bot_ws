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

import pytest

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
    node._max_linear_speed = 1.0
    node._max_angular_speed = 3.0
    node._workspace_root = ""
    node._use_sim = True
    node._world = "speed_course_cfr"
    node._checkpoint_path = "src/bot_bringup/config/maps/checkpoint"
    node._map_save_path = "src/bot_bringup/config/maps/map"
    node._cmd_pub = _FakePublisher()
    node._latest_pose = None
    node._latest_cmd = None
    node._latest_amcl = None
    node._watch_lock = threading.Lock()
    node._watched = {}
    node._reader_blocklist = ["/oak/", "/scan"]
    # __new__ skips Node.__init__, so there's no real logger to return.
    node.get_logger = lambda: _FakeLogger()
    return node


class _FakeLogger:
    def __getattr__(self, _level):
        return lambda *args, **kwargs: None


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


def test_on_teleop_cmd_uses_slider_speeds_when_sent():
    node = _make_node(linear_speed=0.3, angular_speed=1.0)
    node.on_teleop_cmd(1.0, -1.0, linear_speed=0.8, angular_speed=2.5)
    msg = node._cmd_pub.published[-1]
    assert msg.linear.x == pytest.approx(0.8)
    assert msg.angular.z == pytest.approx(-2.5)


def test_on_teleop_cmd_caps_slider_speeds_at_max():
    node = _make_node()
    node.on_teleop_cmd(1.0, 1.0, linear_speed=50.0, angular_speed=50.0)
    msg = node._cmd_pub.published[-1]
    assert msg.linear.x == pytest.approx(1.0)
    assert msg.angular.z == pytest.approx(3.0)


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
        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw),
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
        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw),
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
        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw),
    )

    ok, _ = node.start_launch("mapping", resume=True, resume_pose=None)

    assert ok is True
    assert "resume_pose:=0.0,0.0,0.0" in captured["cmd"]


def test_start_launch_mapping_autonomous_omits_teleop(monkeypatch):
    node = _make_node()
    captured = {}
    monkeypatch.setattr(
        "bot_web_control.web_control_node.subprocess.Popen",
        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw),
    )

    ok, _ = node.start_launch("mapping", autonomous=True)

    assert ok is True
    assert "mapping.launch.py" in captured["cmd"]
    assert not any(a.startswith("teleop:=") for a in captured["cmd"])


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
    """Fake for slam_toolbox's save_map service (std_msgs/String-wrapped name)."""
    def __init__(self, available=True):
        self._available = available
        self.last_request = None

    def wait_for_service(self, timeout_sec=0.0):
        return self._available

    def call_async(self, request):
        self.last_request = request
        return _FakeFuture()


class _FakeSerializeClient:
    """Fake for slam_toolbox's serialize_map service (plain string filename)."""
    def __init__(self, available=True):
        self._available = available
        self.last_request = None

    def wait_for_service(self, timeout_sec=0.0):
        return self._available

    def call_async(self, request):
        self.last_request = request
        return _FakeFuture()


# save_checkpoint must use serialize_map (not save_map) -- see this
# module's own docstring's IMPORTANT note and web_control_node.py's: only
# serialize_map's .posegraph/.data output is something slam_toolbox's
# map_file_name startup parameter can actually resume from.

def test_save_checkpoint_uses_checkpoint_path_by_default():
    node = _make_node()
    node._checkpoint_path = "src/bot_bringup/config/maps/checkpoint"
    node._serialize_client = _FakeSerializeClient()
    node._current_map_pose = lambda timeout=2.0: None  # no TF in a unit test

    ok, msg = node.save_checkpoint(None)

    assert ok is True
    assert node._serialize_client.last_request.filename == "src/bot_bringup/config/maps/checkpoint"
    assert "checkpoint" in msg


def test_save_checkpoint_resolves_bare_name_under_maps_dir():
    node = _make_node()
    node._serialize_client = _FakeSerializeClient()
    node._current_map_pose = lambda timeout=2.0: None  # no TF in a unit test

    ok, msg = node.save_checkpoint("my_custom_checkpoint")

    assert ok is True
    assert node._serialize_client.last_request.filename == "src/bot_bringup/config/maps/my_custom_checkpoint"
    assert "my_custom_checkpoint" in msg


def test_save_checkpoint_reports_failure_when_service_unavailable():
    node = _make_node()
    node._serialize_client = _FakeSerializeClient(available=False)

    ok, msg = node.save_checkpoint(None)

    assert ok is False
    assert "not available" in msg


def test_save_checkpoint_rejects_name_with_slash():
    node = _make_node()
    node._serialize_client = _FakeSerializeClient()

    ok, msg = node.save_checkpoint("../etc/passwd")

    assert ok is False
    assert "invalid map name" in msg
    assert node._serialize_client.last_request is None


# save_final_map must use save_map (not serialize_map) -- its .pgm/.yaml
# output is what bringup.launch.py's amcl/map_server actually load.

def test_save_final_map_uses_map_save_path_by_default():
    node = _make_node()
    node._map_save_path = "src/bot_bringup/config/maps/map"
    node._save_map_client = _FakeSaveMapClient()

    ok, msg = node.save_final_map(None)

    assert ok is True
    assert node._save_map_client.last_request.name.data == "src/bot_bringup/config/maps/map"
    assert "final map" in msg


def test_save_final_map_resolves_bare_name_under_maps_dir():
    node = _make_node()
    node._save_map_client = _FakeSaveMapClient()

    ok, msg = node.save_final_map("my_custom_map")

    assert ok is True
    assert node._save_map_client.last_request.name.data == "src/bot_bringup/config/maps/my_custom_map"
    assert "my_custom_map" in msg


def test_save_final_map_rejects_name_with_slash():
    node = _make_node()
    node._save_map_client = _FakeSaveMapClient()

    ok, msg = node.save_final_map("sub/dir")

    assert ok is False
    assert "invalid map name" in msg
    assert node._save_map_client.last_request is None


def test_save_final_map_treats_blank_name_as_default():
    node = _make_node()
    node._map_save_path = "src/bot_bringup/config/maps/map"
    node._save_map_client = _FakeSaveMapClient()

    ok, _ = node.save_final_map("   ")

    assert ok is True
    assert node._save_map_client.last_request.name.data == "src/bot_bringup/config/maps/map"


def test_save_final_map_reports_failure_when_service_unavailable():
    node = _make_node()
    node._save_map_client = _FakeSaveMapClient(available=False)

    ok, msg = node.save_final_map(None)

    assert ok is False
    assert "not available" in msg


def _make_maps(tmp_path, maps):
    """maps: {name: image filename or None (image missing)}"""
    d = tmp_path / "src" / "bot_bringup" / "config" / "maps"
    d.mkdir(parents=True)
    for name, image in maps.items():
        (d / f"{name}.yaml").write_text(f"image: {image or name + '.pgm'}\nresolution: 0.05\n")
        if image is not None:
            (d / image).write_bytes(b"P5\n1 1\n255\n\x00")
    return d


def test_list_maps_only_lists_yamls_whose_image_exists_map_first(tmp_path):
    node = _make_node()
    node._workspace_root = str(tmp_path)
    _make_maps(tmp_path, {"Test": "Test.pgm", "map": "map.pgm", "broken": None, "alpha": "alpha.pgm"})
    assert node.list_maps() == ["map", "alpha", "Test"]


def test_start_launch_bringup_passes_full_path_of_chosen_map(tmp_path, monkeypatch):
    node = _make_node()
    node._workspace_root = str(tmp_path)
    d = _make_maps(tmp_path, {"map": "map.pgm", "Test": "Test.pgm"})
    captured = {}
    monkeypatch.setattr(
        "bot_web_control.web_control_node.subprocess.Popen",
        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw),
    )

    ok, _ = node.start_launch("bringup", map_name="Test")

    assert ok is True
    assert f"map:={d / 'Test.yaml'}" in captured["cmd"]


def test_start_launch_bringup_rejects_unknown_map(tmp_path, monkeypatch):
    node = _make_node()
    node._workspace_root = str(tmp_path)
    _make_maps(tmp_path, {"map": "map.pgm"})
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: pytest.fail("must not launch"))

    ok, msg = node.start_launch("bringup", map_name="../../etc/passwd")

    assert ok is False
    assert "no saved map" in msg


def _fake_share(tmp_path, monkeypatch, files):
    share = tmp_path / "share"
    (share / "config").mkdir(parents=True)
    for f in files:
        (share / "config" / f).write_text("{}\n")
    monkeypatch.setattr("ament_index_python.packages.get_package_share_directory", lambda pkg: str(share))


def test_list_nav2_params_splits_bringup_and_mapping_default_first(tmp_path, monkeypatch):
    _fake_share(tmp_path, monkeypatch, ["nav2_params_mppi.yaml", "nav2_params.yaml",
                                        "nav2_mapping_params_mppi.yaml", "nav2_mapping_params.yaml",
                                        "ekf_params.yaml"])
    node = _make_node()
    assert node.list_nav2_params() == {
        "bringup": ["nav2_params.yaml", "nav2_params_mppi.yaml"],
        "mapping": ["nav2_mapping_params.yaml", "nav2_mapping_params_mppi.yaml"],
    }


def test_autonomous_mapping_passes_chosen_nav2_params(tmp_path, monkeypatch):
    _fake_share(tmp_path, monkeypatch, ["nav2_mapping_params.yaml", "nav2_mapping_params_mppi.yaml"])
    node = _make_node()
    captured = {}
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw))

    ok, _ = node.start_launch("mapping", autonomous=True, nav2_params="nav2_mapping_params_mppi.yaml")

    assert ok is True
    assert "nav2_params_file:=nav2_mapping_params_mppi.yaml" in captured["cmd"]


def test_teleop_mapping_ignores_nav2_params(tmp_path, monkeypatch):
    _fake_share(tmp_path, monkeypatch, ["nav2_mapping_params_mppi.yaml"])
    node = _make_node()
    captured = {}
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw))

    ok, _ = node.start_launch("mapping", nav2_params="nav2_mapping_params_mppi.yaml")

    assert ok is True
    assert not any(a.startswith("nav2_params_file:=") for a in captured["cmd"])


def test_bringup_rejects_mapping_params_file(tmp_path, monkeypatch):
    _fake_share(tmp_path, monkeypatch, ["nav2_params.yaml", "nav2_mapping_params.yaml"])
    node = _make_node()
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: pytest.fail("must not launch"))

    ok, msg = node.start_launch("bringup", nav2_params="nav2_mapping_params.yaml")

    assert ok is False


def _checkpoint_node(tmp_path):
    node = _make_node()
    node._workspace_root = str(tmp_path)
    (tmp_path / "src" / "bot_bringup" / "config" / "maps").mkdir(parents=True)
    node._serialize_client = None  # never called: _call_and_wait is stubbed below
    node._call_and_wait = lambda client, request, label: (True, None)  # serialize_map "succeeds"
    return node


def test_save_checkpoint_records_the_slam_pose_for_resume(tmp_path):
    import time as _time
    node = _checkpoint_node(tmp_path)
    node._latest_pose = (1.234, -2.5, 0.75, _time.monotonic())

    ok, msg = node.save_checkpoint("lap1")

    assert ok is True and "x=1.23" in msg
    assert node.checkpoint_pose("lap1") == {"x": 1.234, "y": -2.5, "yaw": 0.75}
    assert node.checkpoint_pose(None) is None  # default checkpoint has no record


def test_save_checkpoint_without_a_recent_pose_says_so(tmp_path, monkeypatch):
    node = _checkpoint_node(tmp_path)
    monkeypatch.setattr(node, "_current_map_pose", lambda timeout=2.0: None)  # no TF either

    ok, msg = node.save_checkpoint("lap1")

    assert ok is True and "by hand" in msg
    assert node.checkpoint_pose("lap1") is None


def test_resume_uses_named_checkpoint_and_its_recorded_pose(tmp_path, monkeypatch):
    import time as _time
    node = _checkpoint_node(tmp_path)
    node._latest_pose = (1.0, 2.0, 0.5, _time.monotonic())
    node.save_checkpoint("lap1")
    captured = {}
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw))

    ok, _ = node.start_launch("mapping", resume=True, checkpoint_name="lap1")

    assert ok is True
    assert "resume_map:=src/bot_bringup/config/maps/lap1" in captured["cmd"]
    assert "resume_pose:=1.0,2.0,0.5" in captured["cmd"]


def test_explicit_resume_pose_overrides_the_recorded_one(tmp_path, monkeypatch):
    import time as _time
    node = _checkpoint_node(tmp_path)
    node._latest_pose = (1.0, 2.0, 0.5, _time.monotonic())
    node.save_checkpoint(None)
    captured = {}
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw))

    node.start_launch("mapping", resume=True, resume_pose="3,4,0")

    assert "resume_pose:=3,4,0" in captured["cmd"]


def test_slam_pose_yaw_from_quaternion():
    import math
    from geometry_msgs.msg import PoseWithCovarianceStamped
    node = _make_node()
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x, msg.pose.pose.position.y = 1.5, -0.5
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(0.6), math.cos(0.6)  # yaw 1.2

    node._on_slam_pose(msg)

    assert node._latest_pose[:3] == pytest.approx((1.5, -0.5, 1.2))


def test_status_reports_live_cmd_vel_and_newest_pose():
    import time as _time
    node = _make_node()
    now = _time.monotonic()
    node._latest_cmd = (0.3, -0.5, now)
    node._latest_pose = (1.0, 2.0, 0.1, now - 3.0)  # slam, older
    node._latest_amcl = (4.0, 5.0, 0.2, now - 0.5)  # amcl, newer
    st = node.status()
    assert st["cmd_vel"]["linear"] == 0.3 and st["cmd_vel"]["angular"] == -0.5
    assert st["pose"]["source"] == "amcl" and st["pose"]["x"] == 4.0


def test_status_live_fields_are_null_before_any_data():
    st = _make_node().status()
    assert st["cmd_vel"] is None and st["pose"] is None


def test_read_topic_unknown_topic_reports_error():
    node = _make_node()
    node.get_topic_names_and_types = lambda: [("/scan", ["sensor_msgs/msg/LaserScan"])]
    assert "no such topic" in node.read_topic("/nope")["error"]


def test_read_topic_subscribes_once_and_reports_latest_message():
    from std_msgs.msg import String as StringMsg
    node = _make_node()
    node.get_topic_names_and_types = lambda: [("/chatter", ["std_msgs/msg/String"])]
    node.get_publishers_info_by_topic = lambda name: []
    subs = []
    node.create_subscription = lambda cls, name, cb, qos: subs.append((cls, name, cb)) or object()

    first = node.read_topic("chatter")
    assert first["type"] == "std_msgs/msg/String" and first["message"] is None
    subs[0][2](StringMsg(data="hello"))
    second = node.read_topic("/chatter")

    assert len(subs) == 1  # reused, not re-subscribed
    assert second["message"] == {"data": "hello"}
    assert second["age_s"] is not None


def test_reader_blocks_oak_and_lidar_topics_but_not_lookalikes():
    node = _make_node()
    node.get_topic_names_and_types = lambda: [
        ("/oak/rgb/image_raw", ["sensor_msgs/msg/Image"]), ("/oak/points", ["sensor_msgs/msg/PointCloud2"]),
        ("/scan", ["sensor_msgs/msg/LaserScan"]), ("/scan_filtered", ["sensor_msgs/msg/LaserScan"]),
        ("/odom", ["nav_msgs/msg/Odometry"])]

    assert [t["name"] for t in node.list_topics()] == ["/odom", "/scan_filtered"]
    for blocked in ("/oak/rgb/image_raw", "oak/points", "/scan"):
        assert "blocked" in node.read_topic(blocked)["error"]


_SLAM_BASE = """slam_toolbox:
  ros__parameters:
    odom_frame: odom
    do_loop_closing: true
    minimum_time_interval: 0.5
    throttle_scans: 1
    angle_variance_penalty: 1.0
"""


def _slam_node(tmp_path, monkeypatch):
    share = tmp_path / "share"
    (share / "config").mkdir(parents=True)
    (share / "config" / "slam_toolbox_params.yaml").write_text(_SLAM_BASE)
    monkeypatch.setattr("ament_index_python.packages.get_package_share_directory", lambda pkg: str(share))
    (tmp_path / "src" / "bot_bringup" / "config").mkdir(parents=True)
    node = _make_node()
    node._workspace_root = str(tmp_path)
    return node


def test_slam_params_lists_key_params_first_and_hides_fixed_ones(tmp_path, monkeypatch):
    node = _slam_node(tmp_path, monkeypatch)
    names = [p["name"] for p in node.slam_params()["params"]]
    assert names[:3] == ["minimum_time_interval", "angle_variance_penalty", "do_loop_closing"]
    assert "odom_frame" not in names and "throttle_scans" in names


def test_set_slam_params_saves_only_changes_and_mapping_loads_them(tmp_path, monkeypatch):
    import yaml
    node = _slam_node(tmp_path, monkeypatch)

    ok, msg = node.set_slam_params({"minimum_time_interval": "0.1", "do_loop_closing": "false",
                                    "angle_variance_penalty": "1.0", "throttle_scans": "1"})

    assert ok is True, msg
    saved = yaml.safe_load((tmp_path / "src/bot_bringup/config/slam_toolbox_overrides.yaml").read_text())
    assert saved == {"slam_toolbox": {"ros__parameters": {"minimum_time_interval": 0.1, "do_loop_closing": False}}}
    params = {p["name"]: p for p in node.slam_params()["params"]}
    assert params["minimum_time_interval"]["value"] == 0.1 and params["minimum_time_interval"]["overridden"]

    captured = {}
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw))
    node.start_launch("mapping")
    assert any(a.startswith("slam_params_overrides:=") and a.endswith("slam_toolbox_overrides.yaml")
               for a in captured["cmd"])


def test_set_slam_params_rejects_bad_values_and_fixed_params(tmp_path, monkeypatch):
    node = _slam_node(tmp_path, monkeypatch)
    assert node.set_slam_params({"throttle_scans": "0.5"})[0] is False
    assert node.set_slam_params({"odom_frame": "x"})[0] is False
    assert node.set_slam_params({"not_a_param": 1})[0] is False


def test_reset_removes_overrides_and_mapping_launches_without_them(tmp_path, monkeypatch):
    node = _slam_node(tmp_path, monkeypatch)
    node.set_slam_params({"minimum_time_interval": 0.2})
    ok, _ = node.set_slam_params({})
    assert ok is True
    assert not (tmp_path / "src/bot_bringup/config/slam_toolbox_overrides.yaml").exists()
    captured = {}
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw))
    node.start_launch("mapping")
    assert not any(a.startswith("slam_params_overrides:=") for a in captured["cmd"])


def _route_node(tmp_path):
    import time as _time
    node = _make_node()
    node._workspace_root = str(tmp_path)
    (tmp_path / "src" / "bot_bringup" / "config" / "maps").mkdir(parents=True)
    node._latest_pose = (1.0, 2.0, 0.0, _time.monotonic())
    return node


def test_route_checkpoints_add_undo_clear(tmp_path):
    import time as _time
    node = _route_node(tmp_path)
    assert node.add_route_checkpoint("speed")[0] is True
    node._latest_pose = (4.5, -1.25, 0.0, _time.monotonic())
    ok, msg = node.add_route_checkpoint("speed")
    assert ok and "checkpoint 2" in msg
    info = node.route_info("speed")
    assert info["file"] == "speed.route.yaml" and info["checkpoints"] == [[1.0, 2.0], [4.5, -1.25]]

    assert node.undo_route_checkpoint("speed")[0] is True
    assert node.route_info("speed")["checkpoints"] == [[1.0, 2.0]]
    assert node.clear_route("speed")[0] is True
    assert node.route_info("speed")["checkpoints"] == []


def test_route_checkpoint_needs_a_current_pose(tmp_path, monkeypatch):
    node = _route_node(tmp_path)
    node._latest_pose = None
    monkeypatch.setattr(node, "_current_map_pose", lambda timeout=2.0: None)  # no TF either
    ok, msg = node.add_route_checkpoint("speed")
    assert ok is False and "no current map pose" in msg


def test_stale_topic_pose_falls_back_to_tf(tmp_path, monkeypatch):
    node = _route_node(tmp_path)
    node._latest_pose = (9.0, 9.0, 0.0, 0.0)  # ancient
    monkeypatch.setattr(node, "_current_map_pose",
                        lambda timeout=2.0: {"x": 3.0, "y": 4.0, "yaw": 0.5, "source": "tf", "age_s": 0.0})
    assert node.add_route_checkpoint("speed")[0] is True
    assert node.route_info("speed")["checkpoints"] == [[3.0, 4.0]]


def test_set_start_pose_writes_the_maps_start_file(tmp_path):
    import yaml
    node = _route_node(tmp_path)
    ok, msg = node.set_start_pose("speed")
    assert ok is True, msg
    saved = yaml.safe_load((tmp_path / "src/bot_bringup/config/maps/speed.start.yaml").read_text())
    assert saved == {"x": 1.0, "y": 2.0, "yaw": 0.0}
    assert node.route_info("speed")["start_pose"] == saved


def test_route_file_is_readable_by_lap_navigator(tmp_path):
    from bot_navigation.lap_navigator_node import load_route_file
    node = _route_node(tmp_path)
    node.add_route_checkpoint("speed")
    points, laps = load_route_file(str(tmp_path / "src/bot_bringup/config/maps/speed.route.yaml"))
    assert points == [(1.0, 2.0)] and laps == 0


def test_race_mode_launches_mapping_with_route_and_speed_caps(tmp_path, monkeypatch):
    node = _route_node(tmp_path)
    node.add_route_checkpoint("speed")
    captured = {}
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw))

    ok, msg = node.start_launch("mapping", race=True, map_name="speed", max_speed=0.5, max_turn="1.0")

    assert ok is True, msg
    cmd = captured["cmd"]
    assert "mapping.launch.py" in cmd and "race:=true" in cmd and "teleop:=true" not in cmd
    assert f"route:={tmp_path / 'src/bot_bringup/config/maps/speed.route.yaml'}" in cmd
    assert "max_speed:=0.5" in cmd and "max_turn:=1.0" in cmd
    assert node._proc_label == "race (live map)"


def test_race_mode_refuses_without_a_recorded_route(tmp_path, monkeypatch):
    node = _route_node(tmp_path)
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: pytest.fail("must not launch"))
    ok, msg = node.start_launch("mapping", race=True, map_name="speed")
    assert ok is False and "no recorded route" in msg


def test_speed_caps_not_passed_to_teleop_mapping(tmp_path, monkeypatch):
    node = _route_node(tmp_path)
    captured = {}
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: _FakePopen(captured.setdefault("cmd", cmd), **kw))
    node.start_launch("mapping", max_speed=0.5)
    assert not any(a.startswith("max_speed:=") for a in captured["cmd"])


def test_list_checkpoints_newest_first(tmp_path):
    import os
    node = _make_node()
    node._workspace_root = str(tmp_path)
    d = tmp_path / "src" / "bot_bringup" / "config" / "maps"
    d.mkdir(parents=True)
    for i, n in enumerate(("speed_1", "speed_2", "speed_3")):
        f = d / f"{n}.posegraph"
        f.write_bytes(b"x")
        os.utime(f, (1000 + i, 1000 + i))
    (d / "map.yaml").write_text("image: map.pgm\n")
    assert node.list_checkpoints() == ["speed_3", "speed_2", "speed_1"]


def _ckpt_node(tmp_path, names):
    import os
    node = _make_node()
    node._workspace_root = str(tmp_path)
    d = tmp_path / "src" / "bot_bringup" / "config" / "maps"
    d.mkdir(parents=True, exist_ok=True)
    for i, n in enumerate(names):
        f = d / f"{n}.posegraph"
        f.write_bytes(b"x")
        os.utime(f, (1000 + i, 1000 + i))
    return node


def test_checkpoint_sequence_numbers_per_map_name(tmp_path):
    node = _ckpt_node(tmp_path, ["speed_1", "speed_2", "speed_10", "obstacle_1", "speedway_4", "checkpoint"])
    assert node.checkpoint_sequence("speed") == {"next": "speed_11", "latest": "speed_10"}
    assert node.checkpoint_sequence("obstacle") == {"next": "obstacle_2", "latest": "obstacle_1"}
    assert node.checkpoint_sequence("new") == {"next": "new_1", "latest": None}
    assert node.checkpoint_sequence("") == {"next": "map_1", "latest": None}


def test_pick_checkpoint_explicit_name_wins(tmp_path):
    node = _ckpt_node(tmp_path, ["speed_1", "speed_2"])
    assert node.pick_checkpoint("speed_1", "speed", saving=False) == "speed_1"
    assert node.pick_checkpoint("", "speed", saving=True) == "speed_3"
    assert node.pick_checkpoint(None, "speed", saving=False) == "speed_2"
    assert node.pick_checkpoint(None, None, saving=True) is None  # legacy default checkpoint


def test_save_checkpoint_at_rest_records_the_tf_pose(tmp_path, monkeypatch):
    """/pose goes stale once the robot stops; the save must still record where it is."""
    node = _checkpoint_node(tmp_path)
    node._latest_pose = (9.0, 9.0, 0.0, 0.0)  # ancient
    monkeypatch.setattr(node, "_current_map_pose",
                        lambda timeout=2.0: {"x": 3.0, "y": -1.5, "yaw": 0.25, "source": "tf", "age_s": 0.0})
    ok, msg = node.save_checkpoint("desk_1")
    assert ok is True and "x=3.00" in msg
    assert node.checkpoint_pose("desk_1") == {"x": 3.0, "y": -1.5, "yaw": 0.25}


def test_no_checkpoint_message_names_what_is_saved(tmp_path):
    node = _ckpt_node(tmp_path, ["desk_1"])
    msg = node.no_checkpoint_message("")
    assert "map_1" in msg and "desk_1" in msg and "'desk'" in msg


def test_starting_a_launch_clears_the_previous_map(tmp_path, monkeypatch):
    node = _make_node()
    node._latest_map = _make_grid([0], width=1, height=1)
    monkeypatch.setattr("bot_web_control.web_control_node.subprocess.Popen",
                        lambda cmd, **kw: _FakePopen(cmd, **kw))
    node.start_launch("mapping")
    assert node.status()["map_available"] is False
