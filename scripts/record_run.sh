#!/usr/bin/env bash
# Record a run (e.g. a teleop pass through the car wash) to a rosbag for
# later study/replay. Run while bringup or mapping is up; Ctrl-C (or
# `scripts/record_run.sh --stop`) ends it.
#
#   scripts/record_run.sh carwash            # -> runs/carwash_<date-time>/
#   scripts/record_run.sh carwash --no-camera
#   scripts/record_run.sh --stop             # stop a recording started in the background
#   scripts/record_run.sh x --out DIR        # exact bag directory (the dashboard uses this)
#
# The dashboard records every launch this way automatically ("Record
# rosbag", on by default) -- replay one with scripts/replay_mapping.sh.
#
# Records what's needed to see what the robot saw and did: lidar, odometry,
# EKF, IMU, TF, drive commands, the map, and (unless --no-camera) the OAK's
# JPEG-compressed colour image (~3 MB/s; raw would be ~80 MB/s).
set -eo pipefail  # no -u: ROS setup.bash reads unset variables
WS="$(cd "$(dirname "$0")/.." && pwd)"

if [[ "${1:-}" == "--stop" ]]; then
    pkill -TERM -f "^/usr/bin/python3 .*ros2 bag record .*$WS/runs/" && echo "recording stopped" || echo "no recording running"
    exit 0
fi

name=run camera=1 out=
while (($#)); do
    case "$1" in
        --no-camera) camera=0 ;;
        --out) out="$2"; shift ;;
        *) name="$1" ;;
    esac
    shift
done
topics=(/scan /odom /odometry/filtered /imu /tf /tf_static /cmd_vel /map /map_metadata /pose
        /ultrasonic/left /ultrasonic/right /start_signal /plan /amcl_pose
        /clock)  # sim only: message stamps are Gazebo time there, replay needs it
((camera)) && topics+=(/oak/rgb/image_raw/compressed /oak/rgb/camera_info)

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
mkdir -p "$WS/runs"
[[ -n "$out" ]] || out="$WS/runs/${name}_$(date +%Y%m%d_%H%M%S)"
echo "recording to $out (Ctrl-C or $0 --stop to end)"
# --include-unpublished-topics: topics that aren't up yet (e.g. /amcl_pose
# in mapping) are picked up if they appear, and missing ones don't fail.
exec ros2 bag record --include-unpublished-topics --qos-profile-overrides-path "$WS/scripts/rosbag_qos.yaml" \
    -o "$out" --topics "${topics[@]}"
