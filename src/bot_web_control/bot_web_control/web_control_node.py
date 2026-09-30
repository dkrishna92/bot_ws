"""Localhost web control dashboard for the robot.

Runs a small HTTP server (stdlib http.server -- no new pip/apt dependency)
alongside an rclpy node, serving a single-page dashboard with:
  - Start: launches bringup.launch.py (race run) as a `ros2 launch` subprocess,
    racing on the saved map picked in the dashboard's map list (map:=).
  - Stop: terminates whichever launch (bringup or mapping) is currently
    tracked, then (if workspace_root is set) best-effort runs
    scripts/clean_sim.sh -- or scripts/clean_robot.sh when use_sim is false
    -- for a thorough sweep that catches strays a process-group kill misses.
  - Start New Mapping Run: launches mapping.launch.py with teleop:=true and
    no resume args (this tool's own teleop feature below is what you'd
    drive it with) -- starts from a blank map.
  - Resume Mapping Run: same, but forwards resume_map/resume_pose to
    mapping.launch.py so slam_toolbox continues from a previously-saved
    checkpoint instead of starting empty (see that file's docstring for
    the full mechanism, including why the initial pose also has to match
    wherever the sim robot actually spawns).
  - Start Autonomous Mapping Run: mapping.launch.py WITHOUT teleop:=true,
    so Nav2 + bot_explore's frontier_explore_node drive the robot around
    unexplored space by themselves (blank map, no resume). Heavier than
    teleop mapping -- runs the full Nav2 stack.
  - Nav2 params pickers: which installed nav2_params*.yaml (bringup) /
    nav2_mapping_params*.yaml (autonomous mapping) variant to launch with
    (nav2_params_file:=), e.g. the MPPI controller.
  - Save Checkpoint: calls slam_toolbox's /slam_toolbox/serialize_map
    service (NOT save_map -- see the note below) to checkpoint the
    in-progress map without ending the mapping run, so Resume Mapping Run
    above can genuinely continue from it later.
  - Save Final Map: calls slam_toolbox's /slam_toolbox/save_map service to
    export the current map as the .pgm/.yaml pair bringup.launch.py's
    amcl/map_server actually consumes for racing. Use this once mapping is
    complete -- its output is a flattened occupancy-grid snapshot, not
    something Resume Mapping Run can continue from (see the note below).
  - IMPORTANT, found 2026-09-27: save_map and serialize_map are NOT
    interchangeable, despite both being colloquially "saving the map".
    save_map exports a flattened nav_msgs/OccupancyGrid image (.pgm/.yaml)
    -- what map_server/amcl need, but slam_toolbox's own map_file_name
    startup parameter (what Resume Mapping Run relies on, per
    mapper_params_localization.yaml's stock example) expects a serialized
    pose graph (.posegraph/.data) instead, which only serialize_map
    produces. Checkpointing via save_map alone (an earlier version of
    this tool, and of mapping.launch.py's own docstring, did exactly
    this) makes Resume Mapping Run silently start a fresh, empty map
    instead of actually continuing -- slam_toolbox never finds the
    serialized files it's looking for at that path, so everything the
    robot doesn't happen to re-scan during the resumed drive reverts to
    unknown space. This surfaced as a real bug: a resumed run "fixed" an
    AMCL out-of-bounds issue (the map's bounds did grow, since that part
    only needed fresh scans near the edge) while silently breaking Nav2
    planning elsewhere on the course (large unknown/phantom-obstacle
    patches where the old, undiscarded-looking but actually-gone map data
    used to be).
  - Set Initial Pose: x/y/yaw fields the dashboard sends as resume_pose --
    this must be the robot's actual pose within the checkpoint map. Save
    Checkpoint records slam_toolbox's latest /pose next to the checkpoint
    (<checkpoint>.pose.json) and the page fills these fields from it, so
    as long as the robot is resumed from where it was when saved (or put
    back there), nothing needs typing. Resume uses the checkpoint named in
    the map-name field (default checkpoint if empty).
  - Map display: renders the latest /map (nav_msgs/OccupancyGrid) as a PNG,
    refreshed periodically by the page -- hand-rolled PNG encoder (stdlib
    zlib only) rather than adding a Pillow dependency for one feature.
  - Teleop: gazebo/teleop_twist_keyboard-style WASD (w/s = forward/back,
    a/d = rotate left/right, diagonals combine), plus on-screen buttons for
    non-keyboard use, at adjustable linear/angular speed. Publishes
    geometry_msgs/Twist on /cmd_vel. A 0.5s command-staleness watchdog
    zeroes the Twist if the browser stops sending (tab closed, network
    drop, etc.) -- same "don't trust silence" principle as motor_node's own
    cmd_vel timeout, just enforced at this layer instead.

This is a dev/ops convenience tool, not part of the robot's own runtime
safety architecture -- see CLAUDE.md's Safety architecture section for the
actual e-stop (a dedicated Arduino Nano, independent of this ROS 2 stack
entirely). Do not rely on this page's Stop button as an e-stop.

Usage:
    ros2 launch bot_web_control web_control.launch.py
    # then open http://localhost:8080 (or the machine's LAN IP, since the
    # server binds 0.0.0.0 by default -- handy for driving from a phone/
    # tablet on the same network, still reachable at localhost too)
"""
from __future__ import annotations

import collections
import json
import math
import os
import re
import signal
import struct
import subprocess
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
import rclpy.time
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from slam_toolbox.srv import SaveMap, SerializePoseGraph

TELEOP_STALE_S = 0.5  # zero cmd_vel if the browser stops sending for this long
SAVE_MAP_TIMEOUT_S = 10.0
# Workspace-relative -- matches mapping.launch.py's own maps directory.
# Custom map names typed into the dashboard resolve to a path here (see
# WebControlNode._resolve_map_name), so saving under a new name never
# writes outside this directory.
MAPS_DIR = "src/bot_bringup/config/maps"
# Topic reader: at most this many topics subscribed at once, each dropped
# this long after the page stops polling it.
# SLAM settings: dashboard edits to slam_toolbox's params, saved here
# (workspace-relative) and loaded by mapping.launch.py on top of
# config/slam_toolbox_params.yaml (slam_params_overrides:=).
SLAM_OVERRIDES = "src/bot_bringup/config/slam_toolbox_overrides.yaml"
# Not editable: wiring that has to match the rest of the stack.
SLAM_FIXED_PARAMS = {"odom_frame", "map_frame", "base_frame", "scan_topic", "mode",
                     "solver_plugin", "stack_size_to_use", "use_map_saver"}
# Shown first, with hints: the ones that matter when the map's heading goes
# wrong (lanes drawn rotated / in the wrong direction).
SLAM_KEY_PARAMS = {
    "minimum_time_interval": "Seconds between processed scans. Lower = more scans while turning (more CPU).",
    "minimum_travel_heading": "Radians of turn that triggers processing a scan.",
    "minimum_travel_distance": "Metres of travel that triggers processing a scan.",
    "coarse_search_angle_offset": "Radians the scan matcher searches around the odometry heading (0.349 = +-20 deg).",
    "angle_variance_penalty": "Higher = trust odometry (IMU) heading more, scan-matcher rotations less.",
    "distance_variance_penalty": "Higher = trust odometry distance more.",
    "correlation_search_space_dimension": "Metres the matcher searches around the odometry position.",
    "do_loop_closing": "Repeated identical lanes can cause false loop closures -- try false to test.",
    "loop_match_minimum_response_fine": "Higher = stricter loop closures (fewer false ones).",
    "loop_search_maximum_distance": "Metres to look for loop closures.",
    "max_laser_range": "Metres of lidar used for the map.",
}
MAX_WATCHED_TOPICS = 3
WATCH_EXPIRY_S = 10.0


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)


def encode_grayscale_png(width: int, height: int, pixels: bytes) -> bytes:
    """Minimal 8-bit grayscale PNG encoder (stdlib zlib only, no Pillow)."""
    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    raw = bytearray()
    for y in range(height):
        raw.append(0)  # per-row filter byte: 0 = None
        raw.extend(pixels[y * width:(y + 1) * width])
    idat = zlib.compress(bytes(raw), 6)
    return sig + _png_chunk(b"IHDR", ihdr) + _png_chunk(b"IDAT", idat) + _png_chunk(b"IEND", b"")


class WebControlNode(Node):
    def __init__(self):
        super().__init__("web_control_node")

        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("default_linear_speed", 0.3)
        self.declare_parameter("default_angular_speed", 1.0)
        # Hard caps on the speeds the dashboard's sliders can request --
        # match the sliders' own max, but enforced here so a hand-crafted
        # request can't exceed them either.
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 3.0)
        self.declare_parameter("world", "speed_course_cfr")
        # Bool, not str -- launch_ros infers a bool from a LaunchConfiguration
        # whose resolved text is literally "true"/"false" when building the
        # params file it hands this node, regardless of what type this
        # declare_parameter call asks for. Declaring str here (as an earlier
        # revision did) raised InvalidParameterTypeException at startup. See
        # start_launch() below for where this gets turned back into the
        # "true"/"false" text a `ros2 launch ... use_sim:=` arg needs.
        self.declare_parameter("use_sim", True)
        # Set to the workspace root (e.g. via the launch file) to enable the
        # scripts/clean_sim.sh sweep on Stop. Left blank, Stop still kills
        # the tracked launch's whole process group, just without that extra
        # safety net for orphaned processes.
        self.declare_parameter("workspace_root", "")
        # Path prefix (no extension), workspace-relative -- matches
        # mapping.launch.py's own resume_map convention. This is where
        # Save Checkpoint writes and Resume Mapping Run reads from; it is
        # NOT what bringup.launch.py races against (see map_save_path).
        self.declare_parameter("checkpoint_path", "src/bot_bringup/config/maps/checkpoint")
        # Path prefix (no extension), workspace-relative -- matches
        # mapping.launch.py's own map_save_path convention and what
        # bringup.launch.py's amcl/map_server actually loads. Save Final
        # Map writes here.
        self.declare_parameter("map_save_path", "src/bot_bringup/config/maps/map")
        # slam_toolbox's robot pose in the map frame (PoseWithCovarianceStamped,
        # published per processed scan). Save Checkpoint records the latest
        # one next to the checkpoint so Resume can reuse it -- see
        # save_checkpoint.
        self.declare_parameter("slam_pose_topic", "/pose")
        # Topic reader can't open these (exact names, or prefixes ending in
        # '/'): big high-rate sensor streams that Python would have to decode
        # in full at camera/lidar rate on the Pi -- the OAK-D's image, depth,
        # point cloud and IMU topics and the RPLIDAR's scan.
        self.declare_parameter("topic_reader_blocklist", ["/oak/", "/scan"])

        self._linear_speed = self.get_parameter("default_linear_speed").value
        self._angular_speed = self.get_parameter("default_angular_speed").value
        self._max_linear_speed = self.get_parameter("max_linear_speed").value
        self._max_angular_speed = self.get_parameter("max_angular_speed").value
        self._world = self.get_parameter("world").value
        self._use_sim = self.get_parameter("use_sim").value
        self._workspace_root = self.get_parameter("workspace_root").value
        self._checkpoint_path = self.get_parameter("checkpoint_path").value
        self._map_save_path = self.get_parameter("map_save_path").value

        self._lock = threading.Lock()
        self._latest_map: OccupancyGrid | None = None
        self._last_teleop_time = 0.0
        self._zeroed_since_stale = True
        self._proc: subprocess.Popen | None = None
        self._proc_label: str | None = None  # "bringup" | "mapping"

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._save_map_client = self.create_client(SaveMap, "/slam_toolbox/save_map")
        self._serialize_client = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")
        # Match map_server/slam_toolbox's latched /map publisher so the
        # dashboard still gets the current map when it connects after the
        # (race-time) map_server has already published its single latched map.
        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "map", self._on_map, map_qos)
        self._latest_pose: tuple[float, float, float, float] | None = None  # x, y, yaw, monotonic time
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("slam_pose_topic").value,
                                 self._on_slam_pose, 10)
        # Live readouts for the page: what the motors are being told (from
        # this page's teleop or from Nav2), and AMCL's map pose while racing
        # (slam_toolbox's /pose above covers mapping).
        self._latest_cmd: tuple[float, float, float] | None = None  # linear, angular, monotonic time
        self._latest_amcl: tuple[float, float, float, float] | None = None
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._on_amcl_pose, 10)
        # Topic reader: topics the page is viewing, subscribed on demand and
        # dropped WATCH_EXPIRY_S after the page stops polling them.
        self._watch_lock = threading.Lock()
        self._watched: dict[str, dict] = {}
        self._reader_blocklist = list(self.get_parameter("topic_reader_blocklist").value)
        self.create_timer(2.0, self._expire_watched)
        self.create_timer(0.1, self._teleop_watchdog_tick)
        # Rosbag of the running launch (scripts/record_run.sh), see start_recording
        self._rec_proc: subprocess.Popen | None = None
        self._rec_path: str | None = None
        self.create_timer(2.0, self._check_recording)

        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                node.get_logger().debug("%s - %s" % (self.address_string(), fmt % args))

            def _send_json(self, obj, status=200):
                body = json.dumps(obj).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self):
                length = int(self.headers.get("Content-Length", 0))
                if length == 0:
                    return {}
                return json.loads(self.rfile.read(length).decode("utf-8"))

            def do_GET(self):
                if self.path == "/" or self.path == "/index.html":
                    self._serve_static("index.html", "text/html")
                elif self.path.startswith("/api/map.png"):
                    png = node.render_map_png()
                    if png is None:
                        self.send_response(503)
                        self.end_headers()
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/png")
                    self.send_header("Content-Length", str(len(png)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(png)
                elif self.path.startswith("/api/route"):
                    from urllib.parse import parse_qs, urlparse
                    name = (parse_qs(urlparse(self.path).query).get("name") or [""])[0]
                    self._send_json(node.route_info(name))
                elif self.path.startswith("/api/slam_params"):
                    self._send_json(node.slam_params())
                elif self.path.startswith("/api/topics"):
                    self._send_json({"topics": node.list_topics()})
                elif self.path.startswith("/api/topic?") or self.path == "/api/topic":
                    from urllib.parse import parse_qs, urlparse
                    name = (parse_qs(urlparse(self.path).query).get("name") or [""])[0]
                    self._send_json(node.read_topic(name) if name.strip("/ ") else {"error": "no topic name"})
                elif self.path.startswith("/api/checkpoint_pose"):
                    from urllib.parse import parse_qs, urlparse
                    q = parse_qs(urlparse(self.path).query)
                    base = (q.get("base") or [None])[0]
                    name = node.pick_checkpoint((q.get("name") or [""])[0], base, saving=False)
                    self._send_json({"name": name, "pose": node.checkpoint_pose(name) if (name or base is None) else None})
                elif self.path.startswith("/api/nav2_params"):
                    self._send_json(node.list_nav2_params())
                elif self.path.startswith("/api/checkpoints"):
                    from urllib.parse import parse_qs, urlparse
                    base = (parse_qs(urlparse(self.path).query).get("base") or [""])[0]
                    self._send_json({"checkpoints": node.list_checkpoints(), **node.checkpoint_sequence(base)})
                elif self.path.startswith("/api/maps"):
                    self._send_json({"maps": node.list_maps()})
                elif self.path.startswith("/api/status"):
                    self._send_json(node.status())
                else:
                    self.send_response(404)
                    self.end_headers()

            def do_POST(self):
                if self.path == "/api/cmd_vel":
                    body = self._read_json()
                    lin_speed = body.get("linear_speed")
                    ang_speed = body.get("angular_speed")
                    node.on_teleop_cmd(
                        float(body.get("linear", 0.0)),
                        float(body.get("angular", 0.0)),
                        float(lin_speed) if lin_speed is not None else None,
                        float(ang_speed) if ang_speed is not None else None,
                    )
                    self._send_json({"ok": True})
                elif self.path == "/api/launch/bringup":
                    body = self._read_json()
                    ok, msg = node.start_launch("bringup", map_name=body.get("map"),
                                                nav2_params=body.get("nav2_params"),
                                                max_speed=body.get("max_speed"), max_turn=body.get("max_turn"),
                                                record=bool(body.get("record", False)),
                                                record_camera=bool(body.get("record_camera", False)))
                    self._send_json({"ok": ok, "message": msg})
                elif self.path == "/api/launch/mapping":
                    body = self._read_json()
                    resume = bool(body.get("resume", False))
                    ckpt = node.pick_checkpoint(body.get("name"), body.get("base"), saving=False)
                    if resume and ckpt is None and body.get("base") is not None:
                        self._send_json({"ok": False, "message": node.no_checkpoint_message(body.get("base"))})
                        return
                    ok, msg = node.start_launch(
                        "mapping",
                        resume=resume,
                        resume_pose=body.get("pose"),
                        checkpoint_name=ckpt,
                        autonomous=bool(body.get("autonomous", False)),
                        nav2_params=body.get("nav2_params"),
                        race=bool(body.get("race", False)),
                        map_name=body.get("route_name"),
                        max_speed=body.get("max_speed"),
                        max_turn=body.get("max_turn"),
                        record=bool(body.get("record", False)),
                        record_camera=bool(body.get("record_camera", False)),
                    )
                    self._send_json({"ok": ok, "message": msg})
                elif self.path in ("/api/route/add", "/api/route/undo", "/api/route/clear", "/api/route/start"):
                    name = self._read_json().get("name")
                    action = {"add": node.add_route_checkpoint, "undo": node.undo_route_checkpoint,
                              "clear": node.clear_route, "start": node.set_start_pose}[self.path.rsplit("/", 1)[1]]
                    ok, msg = action(name)
                    self._send_json({"ok": ok, "message": msg, **node.route_info(name)})
                elif self.path == "/api/slam_params":
                    body = self._read_json()
                    ok, msg = node.set_slam_params({} if body.get("reset") else body.get("values") or {})
                    self._send_json({"ok": ok, "message": msg})
                elif self.path == "/api/save_checkpoint":
                    body = self._read_json()
                    name = node.pick_checkpoint(body.get("name"), body.get("base"), saving=True)
                    ok, msg = node.save_checkpoint(name)
                    self._send_json({"ok": ok, "message": msg, "name": name})
                elif self.path == "/api/save_final_map":
                    body = self._read_json()
                    ok, msg = node.save_final_map(body.get("name"))
                    self._send_json({"ok": ok, "message": msg})
                elif self.path == "/api/stop":
                    ok, msg = node.stop_launch()
                    self._send_json({"ok": ok, "message": msg})
                elif self.path == "/api/manual_start":
                    ok, msg = node.manual_start()
                    self._send_json({"ok": ok, "message": msg})
                else:
                    self.send_response(404)
                    self.end_headers()

            def _serve_static(self, name, content_type):
                static_dir = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "static"
                )
                # Falls back to a share-dir install location (this file runs
                # from install/.../site-packages, not the source tree, once
                # colcon-installed -- static/ is installed as a sibling
                # share/bot_web_control/static/ dir, see setup.py).
                candidates = [
                    os.path.join(static_dir, name),
                    os.path.join(
                        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                        "share", "bot_web_control", "static", name,
                    ),
                ]
                for path in candidates:
                    if os.path.isfile(path):
                        with open(path, "rb") as f:
                            data = f.read()
                        self.send_response(200)
                        self.send_header("Content-Type", content_type)
                        self.send_header("Content-Length", str(len(data)))
                        self.end_headers()
                        self.wfile.write(data)
                        return
                self.send_response(404)
                self.end_headers()

        host = self.get_parameter("http_host").value
        port = self.get_parameter("http_port").value
        self._httpd = ThreadingHTTPServer((host, port), Handler)
        self._http_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()
        self.get_logger().info(f"web control dashboard listening on http://{host}:{port}")

    # -- ROS callbacks -----------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._latest_map = msg

    def _teleop_watchdog_tick(self) -> None:
        with self._lock:
            stale = (time.time() - self._last_teleop_time) > TELEOP_STALE_S
            already_zeroed = self._zeroed_since_stale
            if stale:
                self._zeroed_since_stale = True
        if stale and not already_zeroed:
            self._cmd_pub.publish(Twist())

    # -- HTTP-facing operations ---------------------------------------------

    def on_teleop_cmd(self, linear_frac: float, angular_frac: float,
                      linear_speed: float | None = None,
                      angular_speed: float | None = None) -> None:
        """linear_frac/angular_frac are in [-1, 1], scaled by the speeds the
        dashboard's sliders send (or the startup defaults if a request
        omits them), each capped at max_linear_speed/max_angular_speed."""
        lin = self._linear_speed if linear_speed is None else linear_speed
        ang = self._angular_speed if angular_speed is None else angular_speed
        lin = max(0.0, min(self._max_linear_speed, lin))
        ang = max(0.0, min(self._max_angular_speed, ang))
        msg = Twist()
        msg.linear.x = max(-1.0, min(1.0, linear_frac)) * lin
        msg.angular.z = max(-1.0, min(1.0, angular_frac)) * ang
        self._cmd_pub.publish(msg)
        with self._lock:
            self._last_teleop_time = time.time()
            self._zeroed_since_stale = False

    def render_map_png(self) -> bytes | None:
        with self._lock:
            grid = self._latest_map
        if grid is None:
            return None
        w, h = grid.info.width, grid.info.height
        if w == 0 or h == 0:
            return None
        # nav_msgs/OccupancyGrid convention: -1 unknown, 0 free, 100 occupied.
        arr = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
        pixels = np.where(arr < 0, 205, (254.0 - (arr / 100.0) * 254.0).astype(np.int16))
        # OccupancyGrid row 0 is the min-y row; flip so row 0 renders at the
        # top, matching normal image/map_server PGM display convention.
        flipped = np.flipud(pixels).astype(np.uint8)
        return encode_grayscale_png(w, h, flipped.tobytes())

    def _call_and_wait(self, client, request, service_label: str):
        if not client.wait_for_service(timeout_sec=2.0):
            return False, f"{service_label} service not available -- is a mapping run active?"
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda f: done.set())
        # The main executor thread (already spinning independently of this
        # HTTP request thread) is what actually processes the response and
        # fires the done callback above -- safe to just wait on it here
        # rather than spinning ourselves (would risk the "executor already
        # spinning" collision this project hit before with frontier_explore_node).
        if not done.wait(timeout=SAVE_MAP_TIMEOUT_S):
            return False, f"{service_label} call timed out"
        return True, None

    def _resolve_map_name(self, name: str | None, default_path: str):
        """Turn a bare name typed in the dashboard (e.g. "obstacle_v2") into
        a full workspace-relative path prefix alongside the other saved
        maps, so callers don't have to know/type
        src/bot_bringup/config/maps/ each time -- and don't get to escape
        that directory either. Empty/missing name keeps the old
        always-overwrite default. Returns (path_or_None, error_or_None).
        """
        if not name:
            return default_path, None
        name = name.strip()
        if not name:
            return default_path, None
        if "/" in name or "\\" in name or name in (".", ".."):
            return None, f"invalid map name {name!r} -- use a bare filename, no slashes"
        return f"{MAPS_DIR}/{name}", None

    def _maps_dir(self) -> str:
        return os.path.join(self._workspace_root, MAPS_DIR) if self._workspace_root else MAPS_DIR

    def list_checkpoints(self) -> list[str]:
        """Names of the saved (resumable) checkpoints: <name>.posegraph in the
        maps directory, newest first."""
        maps_dir = self._maps_dir()
        try:
            files = [f for f in os.listdir(maps_dir) if f.endswith(".posegraph")]
        except OSError:
            return []
        files.sort(key=lambda f: os.path.getmtime(os.path.join(maps_dir, f)), reverse=True)
        return [f[:-len(".posegraph")] for f in files]

    def checkpoint_sequence(self, base: str | None) -> dict:
        """Auto-numbered checkpoints for a map name: base 'speed' ->
        speed_1, speed_2, ... Returns {'next': name to save, 'latest': newest
        existing name or None}."""
        base = (base or "").strip() or "map"
        if "/" in base or "\\" in base or base in (".", ".."):
            return {"next": None, "latest": None}
        pattern = re.compile(rf"^{re.escape(base)}_(\d+)$")
        numbers = [int(m.group(1)) for n in self.list_checkpoints() if (m := pattern.match(n))]
        return {"next": f"{base}_{max(numbers, default=0) + 1}",
                "latest": f"{base}_{max(numbers)}" if numbers else None}

    def no_checkpoint_message(self, base: str | None) -> str:
        """Resume found no <base>_N checkpoint: say what it looked for and
        what IS saved (a blank Map name means 'map', so a checkpoint saved
        as desk_1 is invisible until Map name is 'desk')."""
        base = (base or "").strip() or "map"
        saved = self.list_checkpoints()
        if not saved:
            return f"no checkpoints saved at all -- nothing to resume (looked for {base}_1, {base}_2, ...)"
        return (f"no checkpoint named {base}_1, {base}_2, ... (Map name is '{base}'). Saved, newest first: "
                f"{', '.join(saved[:6])}. Set Map name to the part before _N (e.g. '{saved[0].rsplit('_', 1)[0]}') "
                f"or type the full name in Checkpoint name.")

    def pick_checkpoint(self, name: str | None, base: str | None, saving: bool) -> str | None:
        """An explicit checkpoint name wins; otherwise, when a map name is
        given, the next auto-numbered one (saving) or the latest (resuming).
        None when neither applies (callers then use the default checkpoint)."""
        if name and name.strip():
            return name.strip()
        if base is None:
            return None
        seq = self.checkpoint_sequence(base)
        return seq["next"] if saving else seq["latest"]

    def list_maps(self) -> list[str]:
        """Names of the saved race maps: every <name>.yaml in the maps
        directory whose 'image:' file actually exists next to it. 'map'
        (bringup.launch.py's default) first, then alphabetical."""
        maps_dir = self._maps_dir()
        try:
            files = os.listdir(maps_dir)
        except OSError:
            return []
        names = []
        for f in files:
            if not f.endswith(".yaml"):
                continue
            try:
                with open(os.path.join(maps_dir, f)) as fh:
                    image = next((line.split(":", 1)[1].strip() for line in fh
                                  if line.startswith("image:")), "")
            except OSError:
                continue
            if image and os.path.isfile(os.path.join(maps_dir, image)):
                names.append(f[:-len(".yaml")])
        return sorted(names, key=lambda n: (n != "map", n.lower()))

    def list_nav2_params(self) -> dict[str, list[str]]:
        """Nav2 params files installed with bot_bringup -- what its launches
        can actually load (nav2_params_file:= is resolved in its share dir):
        'bringup' = nav2_params*.yaml, 'mapping' = nav2_mapping_params*.yaml,
        each with the launch's own default first."""
        try:
            from ament_index_python.packages import get_package_share_directory
            files = os.listdir(os.path.join(get_package_share_directory("bot_bringup"), "config"))
        except (LookupError, OSError):
            return {"bringup": [], "mapping": []}
        out = {}
        for target, prefix in (("bringup", "nav2_params"), ("mapping", "nav2_mapping_params")):
            names = [f for f in files if f.startswith(prefix) and f.endswith(".yaml")]
            out[target] = sorted(names, key=lambda f: (f != f"{prefix}.yaml", f))
        return out

    @staticmethod
    def _xy_yaw(msg: PoseWithCovarianceStamped) -> tuple[float, float, float]:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_slam_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_pose = (*self._xy_yaw(msg), time.monotonic())

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_amcl = (*self._xy_yaw(msg), time.monotonic())

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._latest_cmd = (msg.linear.x, msg.angular.z, time.monotonic())

    # ---- SLAM settings --------------------------------------------------

    def _slam_base_params(self) -> dict:
        """slam_toolbox's params as the mapping launch loads them (installed
        config/slam_toolbox_params.yaml)."""
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("bot_bringup"), "config", "slam_toolbox_params.yaml")
        with open(path) as f:
            return (yaml.safe_load(f) or {})["slam_toolbox"]["ros__parameters"]

    def _slam_overrides(self) -> dict:
        try:
            with open(self._ws_path(SLAM_OVERRIDES)) as f:
                return ((yaml.safe_load(f) or {}).get("slam_toolbox") or {}).get("ros__parameters") or {}
        except (OSError, yaml.YAMLError, AttributeError):
            return {}

    def slam_params(self) -> dict:
        try:
            base = self._slam_base_params()
        except (LookupError, OSError, KeyError, TypeError, yaml.YAMLError) as exc:
            return {"error": f"can't read slam_toolbox_params.yaml: {exc}", "params": []}
        overrides = self._slam_overrides()
        names = [n for n in SLAM_KEY_PARAMS if n in base] + sorted(
            n for n in base if n not in SLAM_KEY_PARAMS and n not in SLAM_FIXED_PARAMS)
        return {"params": [{
            "name": n, "default": base[n], "value": overrides.get(n, base[n]),
            "overridden": n in overrides, "key": n in SLAM_KEY_PARAMS, "hint": SLAM_KEY_PARAMS.get(n, ""),
        } for n in names]}

    def set_slam_params(self, values: dict) -> tuple[bool, str]:
        """Save the given {name: value} (values equal to the default are
        dropped); an empty result deletes the overrides file. Takes effect
        at the next mapping start."""
        try:
            base = self._slam_base_params()
        except (LookupError, OSError, KeyError, TypeError, yaml.YAMLError) as exc:
            return False, f"can't read slam_toolbox_params.yaml: {exc}"
        overrides = {}
        for name, raw in (values or {}).items():
            if name not in base or name in SLAM_FIXED_PARAMS:
                return False, f"{name!r} isn't an editable slam_toolbox parameter"
            default = base[name]
            try:
                if isinstance(default, bool):
                    value = raw if isinstance(raw, bool) else str(raw).strip().lower() in ("true", "1", "yes")
                elif isinstance(default, int):
                    value = int(raw)
                elif isinstance(default, float):
                    value = float(raw)
                else:
                    value = str(raw)
            except (TypeError, ValueError):
                return False, f"{name}: {raw!r} isn't a valid {type(default).__name__}"
            if value != default:
                overrides[name] = value
        path = self._ws_path(SLAM_OVERRIDES)
        try:
            if not overrides:
                if os.path.exists(path):
                    os.remove(path)
                return True, "SLAM settings back to defaults (applies at the next mapping start)"
            with open(path, "w") as f:
                f.write("# Written by the web dashboard's SLAM settings; loaded on top of\n"
                        "# slam_toolbox_params.yaml by mapping.launch.py (slam_params_overrides:=).\n")
                yaml.safe_dump({"slam_toolbox": {"ros__parameters": overrides}}, f, default_flow_style=False)
        except OSError as exc:
            return False, f"couldn't save SLAM settings: {exc}"
        changed = ", ".join(f"{k}={v}" for k, v in overrides.items())
        return True, f"saved ({changed}) -- applies at the next mapping start"

    # ---- topic reader -------------------------------------------------

    def _reader_blocked(self, name: str) -> bool:
        return any(name.startswith(b) if b.endswith("/") else name == b for b in self._reader_blocklist)

    def list_topics(self) -> list[dict]:
        """Topics the reader can open (blocked ones left out)."""
        return [{"name": name, "type": types[0] if types else ""}
                for name, types in sorted(self.get_topic_names_and_types())
                if not self._reader_blocked(name)]

    def read_topic(self, name: str) -> dict:
        """Latest message on any topic, subscribing on first request with QoS
        matched to its publishers (so best-effort sensor topics and latched
        topics both work). Big arrays are truncated for display."""
        name = "/" + name.strip().lstrip("/")
        if self._reader_blocked(name):
            return {"name": name, "error": "blocked in the dashboard (too heavy to decode on the Pi) -- "
                                           "see the topic_reader_blocklist parameter"}
        now = time.monotonic()
        with self._watch_lock:
            w = self._watched.get(name)
            if w is None:
                types = dict(self.get_topic_names_and_types()).get(name)
                if not types:
                    return {"name": name, "error": "no such topic (nothing publishing or subscribing to it)"}
                try:
                    from rosidl_runtime_py.utilities import get_message
                    msg_class = get_message(types[0])
                except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                    return {"name": name, "error": f"can't load message type {types[0]}: {exc}"}
                if len(self._watched) >= MAX_WATCHED_TOPICS:
                    oldest = min(self._watched, key=lambda t: self._watched[t]["last_poll"])
                    self.destroy_subscription(self._watched.pop(oldest)["sub"])
                w = {"type": types[0], "msg": None, "stamps": collections.deque(maxlen=100), "last_poll": now}
                w["sub"] = self.create_subscription(
                    msg_class, name, lambda m, w=w: self._on_watched(w, m), self._matching_qos(name))
                self._watched[name] = w
            w["last_poll"] = now
            msg, stamps, msg_type = w["msg"], list(w["stamps"]), w["type"]
        recent = [t for t in stamps if now - t <= 5.0]
        rate = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) >= 2 and recent[-1] > recent[0] else None
        out = {"name": name, "type": msg_type, "rate_hz": round(rate, 1) if rate else None,
               "age_s": round(now - stamps[-1], 1) if stamps else None, "message": None}
        if msg is not None:
            from rosidl_runtime_py import message_to_ordereddict
            # default=str: anything json can't encode (bytes etc.) shows as text
            out["message"] = json.loads(json.dumps(message_to_ordereddict(msg, truncate_length=16), default=str))
        return out

    def _matching_qos(self, name: str) -> QoSProfile:
        infos = self.get_publishers_info_by_topic(name)
        best_effort = not infos or any(i.qos_profile.reliability == QoSReliabilityPolicy.BEST_EFFORT for i in infos)
        latched = bool(infos) and all(i.qos_profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL for i in infos)
        return QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT if best_effort else QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL if latched else QoSDurabilityPolicy.VOLATILE,
        )

    @staticmethod
    def _on_watched(w: dict, msg) -> None:
        w["msg"] = msg
        w["stamps"].append(time.monotonic())

    def _expire_watched(self) -> None:
        now = time.monotonic()
        with self._watch_lock:
            for name in [t for t, w in self._watched.items() if now - w["last_poll"] > WATCH_EXPIRY_S]:
                self.destroy_subscription(self._watched.pop(name)["sub"])

    def _ws_path(self, path: str) -> str:
        """Workspace-relative path -> absolute (when workspace_root is set)."""
        return os.path.join(self._workspace_root, path) if self._workspace_root else path

    def checkpoint_pose(self, name: str | None) -> dict | None:
        """The robot's map pose recorded when checkpoint `name` (default
        checkpoint if empty) was saved, or None if there's no record."""
        path, err = self._resolve_map_name(name, self._checkpoint_path)
        if err:
            return None
        try:
            with open(self._ws_path(path) + ".pose.json") as f:
                pose = json.load(f)
            return {k: float(pose[k]) for k in ("x", "y", "yaw")}
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def save_checkpoint(self, name: str | None):
        """Checkpoint the in-progress map via slam_toolbox's serialize_map
        service -- does not stop mapping. Unlike save_map (a flattened
        occupancy-grid snapshot), this writes the actual pose graph
        (.posegraph/.data), which is what slam_toolbox's map_file_name
        startup parameter needs to genuinely continue from later (see this
        file's module docstring's IMPORTANT note for why save_map alone
        can't do this, found 2026-09-27). name is an optional bare filename
        (e.g. "obstacle_v2") to save alongside the default checkpoint
        instead of overwriting it -- see _resolve_map_name.
        """
        path, err = self._resolve_map_name(name, self._checkpoint_path)
        if err:
            return False, err
        request = SerializePoseGraph.Request()
        request.filename = path
        ok, err = self._call_and_wait(self._serialize_client, request, "slam_toolbox serialize_map")
        if not ok:
            return False, err
        # Record where the robot is in this map right now: resuming needs
        # the robot's pose in the checkpoint's map frame (map_start_pose),
        # which isn't readable back out of the serialized pose graph. Leave
        # the robot here (or put it back) and Resume uses this pose.
        # slam_toolbox's /pose only updates while the robot moves, so a save
        # after stopping needs the TF fallback (found 2026-09-30: every save
        # made at rest recorded nothing and resumed at 0,0,0).
        pose = self._current_map_pose()
        if pose is None:
            return True, (f"saved checkpoint '{path}' -- but no current slam pose to record, "
                          "so enter the resume pose by hand")
        try:
            with open(self._ws_path(path) + ".pose.json", "w") as f:
                json.dump({k: round(pose[k], 3) for k in ("x", "y", "yaw")}, f)
        except OSError as exc:
            return True, f"saved checkpoint '{path}' -- but couldn't record the pose ({exc})"
        return True, (f"saved checkpoint '{path}' at pose x={pose['x']:.2f} y={pose['y']:.2f} "
                      f"yaw={pose['yaw']:.2f} -- Resume will start from there")

    def save_final_map(self, name: str | None):
        """Export the current map via slam_toolbox's save_map service --
        the flattened .pgm/.yaml pair bringup.launch.py's amcl/map_server
        actually load for racing. Not resumable by a later mapping run --
        use save_checkpoint for that instead. name is an optional bare
        filename (e.g. "obstacle_v2") to save alongside the default map
        instead of overwriting it -- see _resolve_map_name. Any saved map
        can then be picked for bringup (start_launch's map_name).
        """
        path, err = self._resolve_map_name(name, self._map_save_path)
        if err:
            return False, err
        request = SaveMap.Request()
        request.name = String(data=path)
        ok, err = self._call_and_wait(self._save_map_client, request, "slam_toolbox save_map")
        if not ok:
            return False, err
        return True, f"saved final map as '{path}' (ready for bringup.launch.py)"

    # ---- race route ----------------------------------------------------

    def _route_path(self, name: str | None) -> tuple[str | None, str | None]:
        """config/maps/<name>.route.yaml (name = a saved map's name, default
        'map'): a route belongs to the map it was recorded on."""
        base, err = self._resolve_map_name(name, self._map_save_path)
        return (None, err) if err else (self._ws_path(base) + ".route.yaml", None)

    def _current_map_pose(self, timeout: float = 2.0) -> dict | None:
        """The robot's map pose right now: the live /pose or /amcl_pose if
        fresh, else a one-off TF lookup of map -> base_link. Those topics
        only publish while the robot moves, so after stopping at a corner
        they go stale; the temporary TF listener costs nothing the rest of
        the time."""
        pose = self.status()["pose"]
        if pose is not None and pose["age_s"] <= 3.0:
            return pose
        try:
            from tf2_ros import Buffer, TransformListener
        except ImportError:
            return None
        buf = Buffer()
        listener = TransformListener(buf, self, spin_thread=False)
        try:
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                try:
                    t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
                except Exception:  # noqa: BLE001 -- not available yet
                    time.sleep(0.05)
                    continue
                q = t.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return {"x": round(t.transform.translation.x, 3), "y": round(t.transform.translation.y, 3),
                        "yaw": round(yaw, 3), "source": "tf", "age_s": 0.0}
            return None
        finally:
            listener.unregister()

    def _start_pose_path(self, name: str | None) -> tuple[str | None, str | None]:
        base, err = self._resolve_map_name(name, self._map_save_path)
        return (None, err) if err else (self._ws_path(base) + ".start.yaml", None)

    def set_start_pose(self, name: str | None) -> tuple[bool, str]:
        """Record the robot's current map pose as this map's race start
        (<name>.start.yaml) -- bringup seeds AMCL from it."""
        path, err = self._start_pose_path(name)
        if err:
            return False, err
        pose = self._current_map_pose()
        if pose is None:
            return False, "no current map pose (needs slam_toolbox or AMCL running) -- nothing saved"
        try:
            with open(path, "w") as f:
                f.write("# Race start pose in this map (web dashboard 'Set race start pose here');\n"
                        "# bringup.launch.py seeds AMCL from it. Place the robot on the marked start spot.\n")
                yaml.safe_dump({"x": pose["x"], "y": pose["y"], "yaw": pose["yaw"]}, f)
        except OSError as exc:
            return False, f"couldn't save the start pose: {exc}"
        return True, (f"race start pose saved: x={pose['x']:.2f} y={pose['y']:.2f} yaw={pose['yaw']:.2f} "
                      f"[{pose['source']}]")

    def route_info(self, name: str | None) -> dict:
        path, err = self._route_path(name)
        if err:
            return {"error": err, "checkpoints": []}
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError):
            data = {}
        start = None
        start_path, _ = self._start_pose_path(name)
        try:
            with open(start_path) as f:
                start = yaml.safe_load(f)
        except (OSError, yaml.YAMLError, TypeError):
            pass
        return {"file": os.path.basename(path), "checkpoints": data.get("checkpoints") or [], "start_pose": start}

    def _write_route(self, path: str, checkpoints: list) -> None:
        with open(path, "w") as f:
            f.write("# Race route recorded with the web dashboard ('Add checkpoint here'):\n"
                    "# [x, y] in this map's frame, one lap, in driving order. lap_navigator\n"
                    "# repeats it for the course's laps and closes each lap back to the first.\n")
            yaml.safe_dump({"checkpoints": checkpoints}, f, default_flow_style=None)

    def add_route_checkpoint(self, name: str | None) -> tuple[bool, str]:
        """Append the robot's current map pose (slam or AMCL, whichever is
        newest and fresh) to the route."""
        path, err = self._route_path(name)
        if err:
            return False, err
        pose = self._current_map_pose()
        if pose is None:
            return False, "no current map pose (needs slam_toolbox or AMCL running) -- nothing added"
        points = self.route_info(name)["checkpoints"]
        points.append([pose["x"], pose["y"]])
        try:
            self._write_route(path, points)
        except OSError as exc:
            return False, f"couldn't save the route: {exc}"
        return True, f"checkpoint {len(points)} at ({pose['x']:.2f}, {pose['y']:.2f}) [{pose['source']}]"

    def undo_route_checkpoint(self, name: str | None) -> tuple[bool, str]:
        path, err = self._route_path(name)
        if err:
            return False, err
        points = self.route_info(name)["checkpoints"]
        if not points:
            return False, "route is already empty"
        points.pop()
        self._write_route(path, points)
        return True, f"removed the last checkpoint ({len(points)} left)"

    def clear_route(self, name: str | None) -> tuple[bool, str]:
        path, err = self._route_path(name)
        if err:
            return False, err
        if os.path.exists(path):
            os.remove(path)
        return True, "route cleared"

    def start_launch(self, target: str, resume: bool = False, resume_pose: str | None = None,
                     autonomous: bool = False, map_name: str | None = None,
                     nav2_params: str | None = None, checkpoint_name: str | None = None,
                     race: bool = False, max_speed: float | None = None, max_turn: float | None = None,
                     record: bool = False, record_camera: bool = False):
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return False, f"'{self._proc_label}' is already running -- stop it first"
        self._drop_start_publisher()
        launch_file = "bringup.launch.py" if target == "bringup" else "mapping.launch.py"
        cmd = ["ros2", "launch", "bot_bringup", launch_file,
               f"use_sim:={'true' if self._use_sim else 'false'}", f"world:={self._world}"]
        uses_nav2 = target == "bringup" or autonomous or race
        # Speed caps for slow first laps (launch-side: capped copy of the Nav2 params)
        for arg, value in (("max_speed", max_speed), ("max_turn", max_turn)):
            if value and uses_nav2:
                try:
                    if float(value) > 0:
                        cmd.append(f"{arg}:={float(value)}")
                except (TypeError, ValueError):
                    return False, f"{arg} must be a number, got {value!r}"
        if race:
            # Plan B: race on the live slam map along a recorded route
            route_path, err = self._route_path(map_name)
            if err:
                return False, err
            if not os.path.isfile(route_path):
                return False, f"no recorded route {os.path.basename(route_path)} -- record one first"
            cmd += ["race:=true", f"route:={os.path.abspath(route_path)}"]
        if target == "bringup" and map_name:
            # Only maps actually listed (no arbitrary paths from the browser).
            # Passed as a full path: maps saved since the last colcon build
            # aren't in the installed share dir that a bare name resolves to.
            if map_name not in self.list_maps():
                return False, f"no saved map named {map_name!r}"
            cmd.append(f"map:={os.path.abspath(os.path.join(self._maps_dir(), map_name + '.yaml'))}")
        # Nav2 variant (e.g. the MPPI controller). Teleop mapping doesn't
        # run Nav2 at all, so it's only passed where Nav2 actually starts.
        if nav2_params and uses_nav2:
            if nav2_params not in self.list_nav2_params()[target]:
                return False, f"no installed {target} Nav2 params file {nav2_params!r}"
            cmd.append(f"nav2_params_file:={nav2_params}")
        if target == "mapping":
            if self._slam_overrides():
                cmd.append(f"slam_params_overrides:={os.path.abspath(self._ws_path(SLAM_OVERRIDES))}")
            # Default: this tool's own teleop feature is what you'd drive it
            # with (mapping.launch.py skips Nav2 and frontier_explore_node).
            # Autonomous leaves teleop at its false default, so Nav2 +
            # frontier_explore_node drive the robot on their own.
            if not autonomous and not race:
                cmd.append("teleop:=true")
            if resume:
                # Which checkpoint: checkpoint_name (same bare-name rules as
                # saving), default checkpoint if empty.
                ckpt, err = self._resolve_map_name(checkpoint_name, self._checkpoint_path)
                if err:
                    return False, err
                # resume_pose is forwarded as-is (mapping.launch.py parses
                # "x,y,yaw" itself) -- must be the robot's actual pose in the
                # checkpoint's map. Explicit pose wins; otherwise the pose
                # recorded when the checkpoint was saved.
                pose = resume_pose
                if not pose:
                    saved = self.checkpoint_pose(checkpoint_name)
                    pose = f"{saved['x']},{saved['y']},{saved['yaw']}" if saved else "0.0,0.0,0.0"
                cmd.append(f"resume_map:={ckpt}")
                cmd.append(f"resume_pose:={pose}")
        try:
            # New session/process group so Stop can kill every descendant
            # `ros2 launch` spawns, not just the immediate `ros2` process.
            proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        except OSError as exc:
            return False, f"failed to launch: {exc}"
        with self._lock:
            self._proc = proc
            self._proc_label = "race (live map)" if race else target
        # A new launch means new map frames: drop the previous run's readouts,
        # and its map -- otherwise a Resume keeps showing the bad section it
        # just discarded until slam_toolbox publishes the checkpoint's map.
        self._latest_pose = self._latest_amcl = self._latest_cmd = None
        with self._lock:
            self._latest_map = None
        self.get_logger().info(f"started {launch_file} (pid {proc.pid}): {' '.join(cmd[4:])}")
        msg = f"started {launch_file}" + (" in race mode" if race else "")
        if record:
            kind = ("race" if race else "bringup" if target == "bringup" else
                    "resume" if resume else "auto" if autonomous else "mapping")
            msg += "; " + self.start_recording(f"{(map_name or 'map').strip() or 'map'}_{kind}", record_camera)[1]
        return True, msg

    # ---- manual start ----------------------------------------------------

    MANUAL_START_HOLD_S = 3.0

    def manual_start(self) -> tuple[bool, str]:
        """Publish start_signal=True like start_trigger_node does on the
        green light (latched: lap_navigator subscribes transient_local).
        Rules: a manual start costs a 5 s penalty. The publisher is dropped
        after MANUAL_START_HOLD_S -- a latched True left behind would start
        the NEXT race launch the moment its lap_navigator subscribed."""
        with self._lock:
            running = self._proc_label if (self._proc is not None and self._proc.poll() is None) else None
        if running not in ("bringup", "race (live map)"):
            return False, "manual start only works while a race launch (Plan A or Plan B) is running"
        self._drop_start_publisher()
        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        pub = self.create_publisher(Bool, "start_signal", qos)
        pub.publish(Bool(data=True))
        timer = threading.Timer(self.MANUAL_START_HOLD_S, self._drop_start_publisher)
        timer.daemon = True
        with self._lock:
            self._start_pub, self._start_timer = pub, timer
        timer.start()
        self.get_logger().warn("MANUAL START sent on start_signal (5 s penalty in a real race)")
        return True, "start signal sent -- lap_navigator should start the route now"

    def _drop_start_publisher(self) -> None:
        with self._lock:
            pub, timer = getattr(self, "_start_pub", None), getattr(self, "_start_timer", None)
            self._start_pub = self._start_timer = None
        if timer is not None:
            timer.cancel()
        if pub is not None:
            self.destroy_publisher(pub)

    # ---- rosbag recording ------------------------------------------------

    def start_recording(self, name: str, camera: bool = False) -> tuple[bool, str]:
        """Record the running launch to runs/<name>_<date-time>/ via
        scripts/record_run.sh (one topic list for the dashboard and by-hand
        use). Stopped by Stop, or automatically once the launch exits."""
        self.stop_recording()
        script = os.path.join(self._workspace_root, "scripts", "record_run.sh") if self._workspace_root else ""
        if not os.path.isfile(script):
            return False, "not recording (no workspace_root/scripts/record_run.sh)"
        if not re.fullmatch(r"[\w.-]+", name):
            name = "run"
        runs = os.path.join(self._workspace_root, "runs")
        os.makedirs(runs, exist_ok=True)
        out = os.path.join(runs, f"{name}_{time.strftime('%Y%m%d_%H%M%S')}")
        try:
            with open(out + ".log", "w") as log:
                proc = subprocess.Popen([script, name, "--out", out] + ([] if camera else ["--no-camera"]),
                                        stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        except OSError as exc:
            return False, f"not recording ({exc})"
        with self._lock:
            self._rec_proc, self._rec_path = proc, out
        rel = os.path.relpath(out, self._workspace_root)
        self.get_logger().info(f"recording rosbag to {rel}")
        return True, f"recording {rel}"

    def stop_recording(self) -> str | None:
        """SIGTERM the recorder and wait for it to finish writing the bag
        (rosbag2 closes the bag cleanly on SIGTERM, and it still works if
        SIGINT is ignored; a SIGKILLed mcap bag needs `ros2 bag reindex`).
        Returns the bag's path, or None if nothing was recording."""
        with self._lock:
            proc, path = self._rec_proc, self._rec_path
            self._rec_proc = self._rec_path = None
        if proc is None:
            return None
        if proc.poll() is None:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGTERM)
                try:
                    proc.wait(timeout=15.0)
                except subprocess.TimeoutExpired:
                    os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.get_logger().info(f"rosbag closed: {path}")
        return path

    def _check_recording(self) -> None:
        """A launch that died on its own (crash, Ctrl-C) ends its recording too."""
        with self._lock:
            recording = self._rec_proc is not None
            launch_alive = self._proc is not None and self._proc.poll() is None
        if recording and not launch_alive:
            threading.Thread(target=self.stop_recording, daemon=True).start()

    def stop_launch(self):
        # The recorder first, so the cleanup sweep below can't cut it off
        # mid-write.
        bag = self.stop_recording()
        with self._lock:
            proc, label = self._proc, self._proc_label
            self._proc = None
            self._proc_label = None
        stopped_any = False
        if proc is not None and proc.poll() is None:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGINT)
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            stopped_any = True
        # Best-effort thorough sweep -- catches anything the process-group
        # kill above missed. On the robot, clean_robot.sh also frees the
        # sensors and drives the motor pins low. --keep-dashboard stops the
        # scripts from matching (and killing) this dashboard itself.
        if self._workspace_root:
            script_name = "clean_sim.sh" if self._use_sim else "clean_robot.sh"
            script = os.path.join(self._workspace_root, "scripts", script_name)
            if os.path.isfile(script):
                subprocess.run([script, "--keep-dashboard"], cwd=self._workspace_root,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # Zero cmd_vel immediately rather than waiting for the watchdog tick.
        self._cmd_pub.publish(Twist())
        with self._lock:
            self._zeroed_since_stale = True
        msg = f"stopped '{label}'" if stopped_any else "nothing was running"
        if bag:
            msg += f"; rosbag saved to {os.path.relpath(bag, self._workspace_root or '.')}"
        return True, msg

    def status(self):
        with self._lock:
            running = self._proc_label if (self._proc is not None and self._proc.poll() is None) else None
            map_available = self._latest_map is not None
            rec = self._rec_path
        now = time.monotonic()
        cmd = self._latest_cmd
        # Map pose: whichever of slam_toolbox (mapping) / AMCL (racing) is newest
        poses = [(p, src) for p, src in ((self._latest_pose, "slam"), (self._latest_amcl, "amcl")) if p]
        pose, src = max(poses, key=lambda ps: ps[0][3]) if poses else (None, None)
        return {
            "running": running,
            "map_available": map_available,
            "recording": os.path.relpath(rec, self._workspace_root or ".") if rec else None,
            "linear_speed": self._linear_speed,
            "angular_speed": self._angular_speed,
            "cmd_vel": None if cmd is None else {
                "linear": round(cmd[0], 3), "angular": round(cmd[1], 3), "age_s": round(now - cmd[2], 1)},
            "pose": None if pose is None else {
                "x": round(pose[0], 3), "y": round(pose[1], 3), "yaw": round(pose[2], 3),
                "source": src, "age_s": round(now - pose[3], 1)},
        }

    def shutdown(self) -> None:
        self.stop_launch()
        self._httpd.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
