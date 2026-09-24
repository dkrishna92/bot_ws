#!/usr/bin/env bash
# Kills every ROS 2 / Gazebo process left over from a previous `ros2 launch`
# run (gz sim, nav2 servers, slam_toolbox, EKF, RViz, the ros2 daemon, stale
# Fast DDS shared-memory files, leftover core dump files), then verifies
# they actually exited before reporting success.
#
# Run this before starting a new bringup/mapping launch. A straggler
# process from a prior run (especially after Ctrl-C, a killed terminal, or
# a crashed launch) silently duplicates the ROS graph: two Gazebo
# instances, two EKF nodes, two /clock publishers, TF and topic data from
# both mixing together on the same domain. That looks exactly like a
# Nav2/SLAM bug -- stale costmaps, contradictory transforms, duplicate
# node names -- but is actually just a dirty process table left behind by
# the previous run. This is scoped to this workspace's typical launch
# footprint (gz sim, nav2_*, slam_toolbox, etc.) -- it's a broad pattern
# match intended for a dev machine actively iterating on this project, not
# something to run on a shared machine with unrelated ROS 2 processes.
#
# Usage:
#   scripts/clean_sim.sh
#
# Exit code 0: environment is clean, safe to launch.
# Exit code 1: some process(es) survived even SIGKILL -- do not launch;
#              investigate manually (a zombie, or a process stuck in
#              uninterruptible I/O wait, won't die from a signal at all).

set -uo pipefail

PATTERN='gz sim|gz-sim|ros2 launch|nav2_|ekf_node|component_container|slam_toolbox|frontier_explore|robot_state_publisher|parameter_bridge|lifecycle_manager|opennav_docking|rviz2|map_saver'

list_pids() {
    # grep -v grep excludes this pipeline's own grep invocation (its
    # command line literally contains the pattern text above, so it would
    # otherwise match itself).
    ps aux | grep -E "$PATTERN" | grep -v grep | awk '{print $2}'
}

echo "Checking for leftover ROS 2 / Gazebo processes..."
pids="$(list_pids)"

if [ -z "$pids" ]; then
    echo "None found."
else
    count="$(echo "$pids" | wc -l)"
    echo "Found $count leftover process(es); sending SIGTERM..."
    echo "$pids" | xargs -r kill -TERM 2>/dev/null
    sleep 2

    pids="$(list_pids)"
    if [ -n "$pids" ]; then
        echo "Still alive after SIGTERM; sending SIGKILL..."
        echo "$pids" | xargs -r kill -KILL 2>/dev/null
        sleep 2
    fi
fi

echo "Stopping the ROS 2 daemon (stale daemon state can mask a dirty process table)..."
ros2 daemon stop >/dev/null 2>&1 || true

echo "Clearing stale Fast DDS shared-memory files (left behind by any process that didn't exit cleanly)..."
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

# A SIGKILL/SIGSEGV crash (gz sim, a nav2 server, etc.) can dump a core file
# into whatever was the process's cwd -- normally this workspace root, since
# that's where `ros2 launch` is run from. Left in place, a multi-GB core
# file silently eats disk space and is easy to miss since it isn't reported
# by any of the checks above (the crashed process itself is already gone).
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "Clearing core dump files in $WORKSPACE_ROOT..."
core_files="$(find "$WORKSPACE_ROOT" -maxdepth 1 -type f \( -name 'core' -o -name 'core.[0-9]*' \) 2>/dev/null)"
if [ -n "$core_files" ]; then
    echo "$core_files" | while IFS= read -r f; do echo "  removing $f"; done
    echo "$core_files" | xargs -r rm -f
else
    echo "None found."
fi

echo "Verifying..."
remaining="$(list_pids)"
if [ -n "$remaining" ]; then
    echo "FAILED: the following process(es) survived SIGKILL -- investigate manually" >&2
    echo "(a zombie or a process in uninterruptible I/O wait won't die from a signal):" >&2
    # shellcheck disable=SC2086
    ps -o pid,stat,cmd -p $(echo "$remaining" | tr '\n' ',' | sed 's/,$//') >&2
    exit 1
fi

echo "Clean. Safe to start the next run."
exit 0
