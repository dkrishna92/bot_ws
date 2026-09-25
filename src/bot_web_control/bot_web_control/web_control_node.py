"""Localhost web control dashboard for the robot.

Runs a small HTTP server (stdlib http.server -- no new pip/apt dependency)
alongside an rclpy node, serving a single-page dashboard with:
  - Start: launches bringup.launch.py (race run) as a `ros2 launch` subprocess.
  - Stop: terminates whichever launch (bringup or mapping) is currently
    tracked, then best-effort runs scripts/clean_sim.sh for a thorough sweep
    (see that script -- catches strays a plain process-group kill can miss).
  - Mapping Run: launches mapping.launch.py with teleop:=true (this tool's
    own teleop feature below is what you'd drive it with). Optionally
    resumes from a previously-saved checkpoint map at a given initial pose
    (see Save Map / Set Initial Pose below) instead of starting empty --
    forwards straight through to mapping.launch.py's own resume_map/
    resume_pose args (see that file's docstring for the full mechanism,
    including why the initial pose also has to match wherever the sim
    robot actually spawns).
  - Save Map: calls slam_toolbox's /slam_toolbox/save_map service (same
    service frontier_explore_node itself uses) to checkpoint the
    in-progress map without ending the mapping run, so a later Mapping Run
    can resume from it instead of re-driving the whole course again.
  - Set Initial Pose: x/y/yaw fields the dashboard sends as resume_pose --
    this must be the robot's actual pose within the checkpoint map when it
    was saved (e.g. from `ros2 run tf2_ros tf2_echo map base_link` at save
    time), not an arbitrary value.
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

import json
import os
import signal
import struct
import subprocess
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from slam_toolbox.srv import SaveMap

TELEOP_STALE_S = 0.5  # zero cmd_vel if the browser stops sending for this long
SAVE_MAP_TIMEOUT_S = 10.0


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
        # mapping.launch.py's own map_save_path/resume_map convention.
        self.declare_parameter("checkpoint_path", "src/bot_bringup/config/maps/checkpoint")

        self._linear_speed = self.get_parameter("default_linear_speed").value
        self._angular_speed = self.get_parameter("default_angular_speed").value
        self._world = self.get_parameter("world").value
        self._use_sim = self.get_parameter("use_sim").value
        self._workspace_root = self.get_parameter("workspace_root").value
        self._checkpoint_path = self.get_parameter("checkpoint_path").value

        self._lock = threading.Lock()
        self._latest_map: OccupancyGrid | None = None
        self._last_teleop_time = 0.0
        self._zeroed_since_stale = True
        self._proc: subprocess.Popen | None = None
        self._proc_label: str | None = None  # "bringup" | "mapping"

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._save_map_client = self.create_client(SaveMap, "/slam_toolbox/save_map")
        self.create_subscription(OccupancyGrid, "map", self._on_map, 1)
        self.create_timer(0.1, self._teleop_watchdog_tick)

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
                elif self.path.startswith("/api/status"):
                    self._send_json(node.status())
                else:
                    self.send_response(404)
                    self.end_headers()

            def do_POST(self):
                if self.path == "/api/cmd_vel":
                    body = self._read_json()
                    node.on_teleop_cmd(float(body.get("linear", 0.0)), float(body.get("angular", 0.0)))
                    self._send_json({"ok": True})
                elif self.path == "/api/launch/bringup":
                    ok, msg = node.start_launch("bringup")
                    self._send_json({"ok": ok, "message": msg})
                elif self.path == "/api/launch/mapping":
                    body = self._read_json()
                    ok, msg = node.start_launch(
                        "mapping",
                        resume=bool(body.get("resume", False)),
                        resume_pose=body.get("pose"),
                    )
                    self._send_json({"ok": ok, "message": msg})
                elif self.path == "/api/save_map":
                    body = self._read_json()
                    ok, msg = node.save_map(body.get("name"))
                    self._send_json({"ok": ok, "message": msg})
                elif self.path == "/api/stop":
                    ok, msg = node.stop_launch()
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

    def on_teleop_cmd(self, linear_frac: float, angular_frac: float) -> None:
        """linear_frac/angular_frac are in [-1, 1]; scaled by the configured speeds."""
        msg = Twist()
        msg.linear.x = max(-1.0, min(1.0, linear_frac)) * self._linear_speed
        msg.angular.z = max(-1.0, min(1.0, angular_frac)) * self._angular_speed
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
        data = grid.data
        pixels = bytearray(w * h)
        for i, v in enumerate(data):
            # nav_msgs/OccupancyGrid convention: -1 unknown, 0 free, 100 occupied.
            pixels[i] = 205 if v < 0 else int(254 - (v / 100.0) * 254)
        # OccupancyGrid row 0 is the min-y row; flip so row 0 renders at the
        # top, matching normal image/map_server PGM display convention.
        flipped = bytearray(w * h)
        for row in range(h):
            dst = h - 1 - row
            flipped[dst * w:(dst + 1) * w] = pixels[row * w:(row + 1) * w]
        return encode_grayscale_png(w, h, bytes(flipped))

    def save_map(self, name: str | None):
        """Checkpoint the in-progress map via slam_toolbox's own save_map
        service (same one frontier_explore_node uses) -- does not stop
        mapping, just serializes the current pose graph/map to disk so a
        later Mapping Run can resume from it via start_launch(resume=True).
        """
        name = name or self._checkpoint_path
        if not self._save_map_client.wait_for_service(timeout_sec=2.0):
            return False, "slam_toolbox save_map service not available -- is a mapping run active?"
        request = SaveMap.Request()
        request.name = String(data=name)
        future = self._save_map_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda f: done.set())
        # The main executor thread (already spinning independently of this
        # HTTP request thread) is what actually processes the response and
        # fires the done callback above -- safe to just wait on it here
        # rather than spinning ourselves (would risk the "executor already
        # spinning" collision this project hit before with frontier_explore_node).
        if not done.wait(timeout=SAVE_MAP_TIMEOUT_S):
            return False, "save_map call timed out"
        return True, f"saved map as '{name}'"

    def start_launch(self, target: str, resume: bool = False, resume_pose: str | None = None):
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return False, f"'{self._proc_label}' is already running -- stop it first"
        launch_file = "bringup.launch.py" if target == "bringup" else "mapping.launch.py"
        cmd = ["ros2", "launch", "bot_bringup", launch_file,
               f"use_sim:={'true' if self._use_sim else 'false'}", f"world:={self._world}"]
        if target == "mapping":
            # This tool's own teleop feature is what you'd drive it with.
            cmd.append("teleop:=true")
            if resume:
                # resume_pose is forwarded as-is (mapping.launch.py parses
                # "x,y,yaw" itself) -- must be the robot's actual pose in
                # the checkpoint map when it was saved, not arbitrary; see
                # this file's module docstring.
                pose = resume_pose or "0.0,0.0,0.0"
                cmd.append(f"resume_map:={self._checkpoint_path}")
                cmd.append(f"resume_pose:={pose}")
        try:
            # New session/process group so Stop can kill every descendant
            # `ros2 launch` spawns, not just the immediate `ros2` process.
            proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        except OSError as exc:
            return False, f"failed to launch: {exc}"
        with self._lock:
            self._proc = proc
            self._proc_label = target
        self.get_logger().info(f"started {launch_file} (pid {proc.pid})")
        return True, f"started {launch_file}"

    def stop_launch(self):
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
        # kill above missed (see scripts/clean_sim.sh's own docstring).
        if self._workspace_root:
            script = os.path.join(self._workspace_root, "scripts", "clean_sim.sh")
            if os.path.isfile(script):
                subprocess.run([script], cwd=self._workspace_root,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # Zero cmd_vel immediately rather than waiting for the watchdog tick.
        self._cmd_pub.publish(Twist())
        with self._lock:
            self._zeroed_since_stale = True
        msg = f"stopped '{label}'" if stopped_any else "nothing was running"
        return True, msg

    def status(self):
        with self._lock:
            running = self._proc_label if (self._proc is not None and self._proc.poll() is None) else None
            map_available = self._latest_map is not None
        return {
            "running": running,
            "map_available": map_available,
            "linear_speed": self._linear_speed,
            "angular_speed": self._angular_speed,
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
