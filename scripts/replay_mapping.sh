#!/usr/bin/env bash
# Replay a recorded mapping run (runs/<name>/, from the dashboard's "Record
# rosbag" or scripts/record_run.sh) to study it or to re-map it with
# different settings. Runs on the laptop (or the Pi with no launch up).
#
#   scripts/replay_mapping.sh runs/desk_mapping_20260930_140000
#       re-runs the EKF + slam_toolbox from the recorded /scan, /odom, /imu
#       and /tf_static, so the map is rebuilt from scratch -- try SLAM
#       settings without driving again. Saves the result next to the bag
#       as <bag>_replay.pgm/.yaml (+ .posegraph) when the bag ends.
#   ... --overrides src/bot_bringup/config/slam_toolbox_overrides.yaml
#       extra slam_toolbox params on top of slam_toolbox_params.yaml
#   ... --as-recorded   play everything (incl. /tf, /map) to watch what happened
#   ... --rate 2        playback speed
#   ... --rviz          open RViz (laptop)
#
# Get a bag off the Pi (no sftp there):
#   ssh robot-pi "tar cf - -C bot_ws/runs desk_mapping_20260930_140000" | tar xf - -C runs/
#
# Quick look without replaying: ros2 bag info runs/<name>
set -eo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$WS/src/bot_bringup/config"

bag="" overrides="" rate=1.0 as_recorded=0 rviz=0
while (($#)); do
    case "$1" in
        --overrides) overrides="$2"; shift ;;
        --rate) rate="$2"; shift ;;
        --as-recorded) as_recorded=1 ;;
        --rviz) rviz=1 ;;
        -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
        *) bag="$1" ;;
    esac
    shift
done
bag="${bag%/}"
[[ -d "$bag" ]] || { echo "usage: $0 runs/<bag dir> [--overrides f.yaml] [--as-recorded] [--rate R] [--rviz]"; exit 1; }

source /opt/ros/jazzy/setup.bash
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
# Private DDS domain: don't mix replayed data with a live robot/sim.
export ROS_DOMAIN_ID="${REPLAY_DOMAIN_ID:-87}" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

pids=()
# SIGTERM, not SIGINT: background jobs of a script start with SIGINT ignored.
cleanup() { for p in "${pids[@]}"; do kill -TERM "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

if ((rviz)); then
    rviz2 -d "$WS/src/bot_bringup/launch/sensors.rviz" --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
    pids+=($!)
fi

if ((as_recorded)); then
    echo "playing $bag as recorded (domain $ROS_DOMAIN_ID)"
    ros2 bag play "$bag" --clock 100 -r "$rate"
    exit 0
fi

out="${bag}_replay"
slam_params=(--params-file "$CFG/slam_toolbox_params.yaml")
[[ -n "$overrides" ]] && slam_params+=(--params-file "$overrides")

/opt/ros/jazzy/lib/robot_localization/ekf_node --ros-args -r __node:=ekf_filter_node \
    --params-file "$CFG/ekf_params.yaml" -p use_sim_time:=true > "$out.ekf.log" 2>&1 &
pids+=($!)
/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node --ros-args -r __node:=slam_toolbox \
    "${slam_params[@]}" -p use_sim_time:=true > "$out.slam.log" 2>&1 &
pids+=($!)

# slam_toolbox is a lifecycle node (the launch files use a lifecycle manager)
for t in configure activate; do
    for _ in $(seq 20); do
        ros2 lifecycle set --no-daemon --spin-time 3 /slam_toolbox "$t" 2>/dev/null | grep -q successful && break
        sleep 1
    done
done
active=0
for _ in $(seq 10); do  # activation can lag the 'successful' reply on a busy machine
    [[ "$(ros2 lifecycle get --no-daemon --spin-time 3 /slam_toolbox 2>/dev/null)" == active* ]] && { active=1; break; }
    sleep 2
done
((active)) || { echo "slam_toolbox didn't start -- see $out.slam.log"; exit 1; }

# slam_toolbox only publishes /map while something subscribes, and its
# publish loop runs on the bag's clock, which stops when playback ends -- so
# without a subscriber for the whole replay, save_map never gets a map.
ros2 topic echo /map --field info.width --qos-durability transient_local --qos-reliability reliable \
    > /dev/null 2>&1 &
pids+=($!)

echo "replaying $bag through a fresh EKF + slam_toolbox (domain $ROS_DOMAIN_ID, rate $rate)"
echo "  watch it: ROS_DOMAIN_ID=$ROS_DOMAIN_ID rviz2   (or pass --rviz)"
# Robot bags are stamped in wall time, so the recording time is the clock.
# Sim bags are stamped in Gazebo time: replay their recorded /clock instead.
if ros2 bag info "$bag" 2>/dev/null | grep -Eq "Topic: /clock .*Count: [1-9]"; then
    clock=(--topics /scan /odom /imu /tf_static /clock)
else
    clock=(--clock 100 --topics /scan /odom /imu /tf_static)
fi
ros2 bag play "$bag" -r "$rate" "${clock[@]}"

sleep 3  # let the last scans get processed
ros2 run nav2_map_server map_saver_cli -f "$out" --ros-args \
    -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0 > "$out.save.log" 2>&1 \
    && echo "map saved: $out.pgm / $out.yaml" || echo "map save FAILED -- see $out.save.log"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$out'}" \
    | grep -q "result=0" && echo "pose graph saved: $out.posegraph (resumable)" \
    || echo "pose graph save FAILED"
