#!/usr/bin/env bash
# Record a run (e.g. a teleop pass through the car wash) to a rosbag for
# later study/replay. Run while bringup or mapping is up; Ctrl-C (or
# `scripts/record_run.sh --stop`) ends it.
#
#   scripts/record_run.sh carwash            # -> runs/carwash_<date-time>/
#   scripts/record_run.sh carwash --no-camera
#   scripts/record_run.sh --stop             # stop a recording started in the background
#
# Records what's needed to see what the robot saw and did: lidar, odometry,
# EKF, IMU, TF, drive commands, the map, and (unless --no-camera) the OAK's
# JPEG-compressed colour image (~3 MB/s; raw would be ~80 MB/s).
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"

if [[ "${1:-}" == "--stop" ]]; then
    pkill -INT -f "ros2 bag record .*$WS/runs/" && echo "recording stopped" || echo "no recording running"
    exit 0
fi

name="${1:-run}"
topics=(/scan /odom /odometry/filtered /imu /tf /tf_static /cmd_vel /map /map_metadata
        /ultrasonic/left /ultrasonic/right /start_signal /plan /amcl_pose)
[[ "${2:-}" == "--no-camera" ]] || topics+=(/oak/rgb/image_raw/compressed /oak/rgb/camera_info)

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
mkdir -p "$WS/runs"
out="$WS/runs/${name}_$(date +%Y%m%d_%H%M%S)"
echo "recording to $out (Ctrl-C or $0 --stop to end)"
# --include-unpublished-topics: topics that aren't up yet (e.g. /amcl_pose
# in mapping) are picked up if they appear, and missing ones don't fail.
exec ros2 bag record --include-unpublished-topics -o "$out" "${topics[@]}"
