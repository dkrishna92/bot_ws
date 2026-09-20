#!/usr/bin/env bash
# Runs the Nav2 planner/controller A/B test matrix (see CLAUDE.md's Nav2
# planner/controller comparison section). Launches each variant config,
# exercises it, and writes a log + one-line summary per run under
# scripts/nav2_variant_results/<timestamp>/.
#
# Usage:
#   scripts/run_nav2_variant_test.sh race <variant.yaml> [<variant.yaml> ...]
#   scripts/run_nav2_variant_test.sh mapping <variant.yaml> [<variant.yaml> ...]
#   scripts/run_nav2_variant_test.sh race --all
#   scripts/run_nav2_variant_test.sh mapping --all
#
# Requires ROS 2 + this workspace sourced first:
#   source /opt/ros/jazzy/setup.bash && source install/setup.bash
#
# This automates the mechanical part (launch/observe/clean/repeat) -- it
# does not replace manually sanity-checking the "near a wall" goal
# coordinate in RViz first (see CLAUDE.md), and the mapping timeout below
# (10 min) is a starting point, not a validated value.

set -uo pipefail

MODE="${1:?usage: $0 <race|mapping> <variant.yaml...|--all>}"; shift
RESULTS_DIR="scripts/nav2_variant_results/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULTS_DIR"

RACE_VARIANTS=(nav2_params.yaml nav2_params_thetastar.yaml nav2_params_smac_lattice.yaml nav2_params_mppi.yaml)
MAPPING_VARIANTS=(nav2_mapping_params.yaml nav2_mapping_params_thetastar.yaml nav2_mapping_params_smac_lattice.yaml nav2_mapping_params_mppi.yaml)

if [ "${1:-}" = "--all" ]; then
    if [ "$MODE" = "race" ]; then VARIANTS=("${RACE_VARIANTS[@]}"); else VARIANTS=("${MAPPING_VARIANTS[@]}"); fi
else
    VARIANTS=("$@")
fi

# x y qz qw per goal. Goal 3's y is an estimate; verify against the real
# corridor in RViz before trusting results from it.
GOALS_RACE=(
    "1.0 0.0 0.0 1.0"
    "3.5 0.2 0.7071 0.7071"
    "2.0 0.5 0.0 1.0"
)

launch_and_wait_active() {
    local launch_file="$1" params_file="$2" log="$3"
    scripts/clean_sim.sh
    ros2 launch bot_bringup "$launch_file" use_sim:=true nav2_params_file:="$params_file" > "$log" 2>&1 &
    echo $! > "$log.pid"
    for i in $(seq 1 60); do
        ros2 node list 2>/dev/null | grep -q "/bt_navigator" && return 0
        sleep 2
    done
    echo "WARNING: bt_navigator did not come up within 120s for $params_file" >> "$log"
    return 1
}

run_race_variant() {
    local variant="$1"
    for goal in "${GOALS_RACE[@]}"; do
        read -r x y qz qw <<< "$goal"
        local log="$RESULTS_DIR/race_${variant%.yaml}_goal_${x}_${y}.log"
        launch_and_wait_active bringup.launch.py "$variant" "$log" || continue
        local start end
        start=$(date +%s.%N)
        ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
            "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {z: $qz, w: $qw}}}}" \
            >> "$log" 2>&1
        end=$(date +%s.%N)
        echo "RESULT variant=$variant goal=($x,$y) elapsed=$(echo "$end - $start" | bc)s" \
            "$(grep -o 'Goal finished with status: [A-Z]*' "$log" | tail -1)" \
            >> "$RESULTS_DIR/summary.txt"
        kill "$(cat "$log.pid")" 2>/dev/null
    done
}

run_mapping_variant() {
    local variant="$1"
    local log="$RESULTS_DIR/mapping_${variant%.yaml}.log"
    launch_and_wait_active mapping.launch.py "$variant" "$log" || return
    # Bounded wait for exploration to finish; not finishing by then IS the
    # "stalled" signal this test is watching for.
    for i in $(seq 1 300); do
        grep -q "Exploration complete" "$log" && break
        sleep 2
    done
    local failed blacklisted completed
    failed=$(grep -c "failed to plan\|Failed to create plan" "$log")
    blacklisted=$(grep -c "did not succeed.*blacklisting" "$log")
    completed=$(grep -c "Exploration complete" "$log")
    echo "RESULT variant=$variant planner_failures=$failed blacklisted_frontiers=$blacklisted completed=$completed" \
        >> "$RESULTS_DIR/summary.txt"
    kill "$(cat "$log.pid")" 2>/dev/null
}

for v in "${VARIANTS[@]}"; do
    if [ "$MODE" = "race" ]; then run_race_variant "$v"; else run_mapping_variant "$v"; fi
done

scripts/clean_sim.sh
echo "Done. Summary: $RESULTS_DIR/summary.txt"
cat "$RESULTS_DIR/summary.txt"
