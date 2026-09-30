#!/usr/bin/env python3
"""Test the vision start trigger against the REAL OAK-D and the real start signal.

Runs bot_perception's start_trigger_node detection code unchanged (same ROI,
HSV ranges, threshold and confirm_frames -- any of them overridable with
--ros-args -p ...), but instead of latching once it:
  - prints, twice a second: camera frame rate, red % and green % in the ROI,
    the ROI's median H/S/V, and the verdict (RED / GREEN n-of-N / neither)
  - saves annotated snapshots (ROI box + numbers) so you can see what the
    camera sees on the headless Pi: first frame, every --snapshot-every
    seconds, and every trigger
  - logs each trigger with the delay since green was first seen, then
    re-arms once the signal reads red again, so you can flip it repeatedly
The trigger decision is start_trigger_node's own _update(): ignore the
first warmup_s of frames, arm after red_confirm_frames of red, fire after
confirm_frames of green.
It NEVER publishes start_signal -- running it next to bringup can't start
the robot -- and runs as node 'start_signal_test', not start_trigger_node.

The ROI/HSV defaults were tuned in the SIM only (see start_trigger_node.py's
TODO): expect to adjust them here, then copy the working values into
bringup.launch.py's hardware start_trigger_node parameters.

Usage (on the Pi, robot at the real start position, signal in view):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    scripts/test_start_signal.py --start-camera            # camera not running yet
    scripts/test_start_signal.py                           # bringup already running the camera
    scripts/test_start_signal.py --start-camera --ros-args -p roi:="[0.4, 0.2, 0.6, 0.5]" \\
        -p green_hsv_lower:="[35, 80, 60]" -p detect_threshold:=0.10
Snapshots go to ~/start_signal_test/ (copy them off with scp to look).
"""
import argparse
import os
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy

from bot_perception.start_trigger_node import StartTriggerNode


class StartSignalTest(StartTriggerNode):
    """start_trigger_node's detector, instrumented and non-latching."""

    def setup_test(self, out_dir, snapshot_every, once):
        self._out_dir = out_dir
        self._snapshot_every = snapshot_every
        self._once = once
        self._t0 = time.monotonic()
        self._frames = 0
        self._last_report = 0.0
        self._last_snapshot = -1e9
        self._green_since = None
        self.triggers = 0
        self.done = False
        os.makedirs(out_dir, exist_ok=True)
        self.get_logger().info(
            f"watching {self._image_topic}, ROI {list(self._roi)}, threshold {self._threshold}, "
            f"confirm_frames {self._confirm_frames}, warmup {self._warmup_s}s, "
            f"red_confirm_frames {self._red_confirm_frames} -- snapshots in {out_dir}")

    def _snapshot(self, frame, name, green, red, verdict):
        img = frame.copy()
        h, w = img.shape[:2]
        x1, y1, x2, y2 = self._roi
        cv2.rectangle(img, (int(x1 * w), int(y1 * h)), (int(x2 * w), int(y2 * h)), (255, 255, 0), 2)
        cv2.putText(img, f"red {red * 100:.0f}%  green {green * 100:.0f}%  {verdict}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        path = os.path.join(self._out_dir, name)
        cv2.imwrite(path, img)
        return path

    def _on_image(self, msg):
        now = time.monotonic()
        t = now - self._t0
        self._frames += 1
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = self._roi_hsv(frame)
        green = self._color_fraction(hsv, self._green_lower, self._green_upper)
        red = self._red_fraction(hsv)

        is_green = green > self._threshold and red <= self._threshold
        if is_green and self._green_since is None:
            self._green_since = now
        elif not is_green:
            self._green_since = None

        was_armed = self._armed
        fired = self._update(green, red, now)  # exactly start_trigger_node's rule
        if self._armed and not was_armed:
            self.get_logger().info(f"{t:7.2f}s  signal reads RED -- armed")

        if now - self._first_frame_time < self._warmup_s:
            verdict = "warming up"
        elif not self._armed:
            verdict = f"waiting for RED {self._red_streak}/{self._red_confirm_frames}"
        elif is_green:
            verdict = f"GREEN {min(self._green_streak, self._confirm_frames)}/{self._confirm_frames}"
        else:
            verdict = "armed"

        if self._frames == 1:
            self.get_logger().info("first frame saved: " + self._snapshot(frame, "first.jpg", green, red, verdict))

        if fired:
            self.triggers += 1
            delay = (now - self._green_since) * 1000 if self._green_since else float("nan")
            path = self._snapshot(frame, f"trigger_{self.triggers}.jpg", green, red, "TRIGGER")
            self.get_logger().info(
                f"{t:7.2f}s  *** TRIGGER #{self.triggers} *** {delay:.0f} ms after green first seen "
                f"(green {green * 100:.0f}%, red {red * 100:.0f}%) -> {path}")
            # Re-arm like a fresh run, minus the warmup: needs red again first
            self._armed = False
            self._red_streak = self._green_streak = 0
            if self._once:
                self.done = True

        if now - self._last_snapshot >= self._snapshot_every:
            self._last_snapshot = now
            self._snapshot(frame, "latest.jpg", green, red, verdict)

        if now - self._last_report >= 0.5:
            fps = self._frames / t if t > 0 else 0.0
            med_h, med_s, med_v = (int(v) for v in np.median(hsv.reshape(-1, 3), axis=0))
            self.get_logger().info(
                f"{t:7.2f}s  {fps:4.1f} fps  red {red * 100:5.1f}%  green {green * 100:5.1f}%  "
                f"ROI median HSV ({med_h:3d},{med_s:3d},{med_v:3d})  {verdict}")
            self._last_report = now


def start_camera():
    """The OAK-D the same way hardware.launch.py starts it."""
    from ament_index_python.packages import get_package_share_directory
    params = os.path.join(get_package_share_directory("bot_bringup"), "config", "oak_params.yaml")
    cmd = ["ros2", "launch", "depthai_ros_driver", "camera.launch.py", "name:=oak",
           "camera_model:=OAK-D-S2", f"params_file:={params}", "rectify_rgb:=false"]
    print("starting camera:", " ".join(cmd), flush=True)
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--start-camera", action="store_true",
                    help="launch the OAK-D driver too (use when bringup isn't running)")
    ap.add_argument("--out-dir", default=os.path.expanduser("~/start_signal_test"),
                    help="where snapshots go (default ~/start_signal_test)")
    ap.add_argument("--snapshot-every", type=float, default=5.0,
                    help="seconds between latest.jpg updates (default 5)")
    ap.add_argument("--once", action="store_true", help="exit after the first trigger")
    ap.add_argument("--timeout", type=float, default=0.0,
                    help="exit after this many seconds (default: run until Ctrl-C)")
    args, ros_args = ap.parse_known_args()

    # Run start_trigger_node's code under its own node name, so it doesn't
    # collide with a running start_trigger_node (or its parameters).
    if "--ros-args" not in ros_args:
        ros_args.append("--ros-args")
    ros_args += ["-r", "__node:=start_signal_test"]

    cam = start_camera() if args.start_camera else None
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = StartSignalTest()
    node.setup_test(args.out_dir, args.snapshot_every, args.once)
    start = time.monotonic()
    warned = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.monotonic() - start
            if node._frames == 0 and elapsed > 15 and not warned:
                warned = True
                node.get_logger().warn(
                    f"no frames on {node._image_topic} after 15 s -- camera not running? "
                    "Use --start-camera, or check `ros2 topic list | grep oak`.")
            if args.timeout and elapsed > args.timeout:
                break
    except KeyboardInterrupt:
        pass
    finally:
        triggers = node.triggers
        node.destroy_node()
        rclpy.try_shutdown()
        if cam is not None:
            os.killpg(cam.pid, signal.SIGINT)
            try:
                cam.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(cam.pid, signal.SIGKILL)
    print(f"\n{triggers} trigger(s). Snapshots in {args.out_dir}")


if __name__ == "__main__":
    main()
