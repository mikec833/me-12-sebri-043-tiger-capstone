#!/usr/bin/env bash
# Replay one sensor bag through ballrobot_localization_pkg in the four Step 8
# configurations, record the outputs, evaluate each run and build
# results/summary.md.
#
# Usage (from ros2_ws, after colcon build and sourcing install/setup.bash):
#   src/ballrobot_localization_pkg/scripts/run_comparison.sh <sensor_bag> [dropout_start] [ground_truth.csv]
#
# dropout_start: seconds after the first UWB sample where run D blanks UWB
# for 10 s (default 30). Pick a stretch where the robot is moving.
#
# CONFIG selects the shared setup file the bag was recorded with (default
# 'default'), a name in ballrobot_pkg/config/ or a path, for example the copy
# bringup saved: CONFIG=src/outputs/run_<id>/config_outdoor.yaml
set -eo pipefail

BAG="${1:?usage: run_comparison.sh <sensor_bag> [dropout_start] [ground_truth.csv]}"
DROP_START="${2:-30}"
GT="${3:-}"
CONFIG="${CONFIG:-default}"
DROP_END=$(python3 -c "print(float('$DROP_START') + 10.0)")

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULTS="$PKG_DIR/results"
EVAL="$PKG_DIR/scripts/evaluate.py"
RECORD_TOPICS=(/uwb/position /imu/rpy /imu/accel /wheel_states
               /localization/uwb_pose /localization/imu /localization/wheel_odom
               /odometry/local /odometry/global /diagnostics /tf /tf_static)

run_one() {
    local label="$1"; shift
    local dropout="$1"; shift
    local out="$RESULTS/run_$label"
    rm -rf "$out"; mkdir -p "$out"
    echo "=== Run $label: $*"

    ros2 launch ballrobot_localization_pkg offline.launch.py "config:=$CONFIG" "$@" > "$out/launch.log" 2>&1 &
    local launch_pid=$!
    sleep 4  # let the EKFs and adapters come up before data arrives

    ros2 bag record -o "$out/bag" "${RECORD_TOPICS[@]}" > "$out/record.log" 2>&1 &
    local record_pid=$!
    sleep 2

    ros2 bag play "$BAG" --clock > "$out/play.log" 2>&1
    sleep 2

    kill -INT "$record_pid" 2>/dev/null || true
    wait "$record_pid" 2>/dev/null || true
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true

    local eval_args=(run --bag "$out/bag" --out "$out" --label "$label" --config "$CONFIG")
    [[ -n "$dropout" ]] && eval_args+=(--dropout "$dropout")
    [[ -n "$GT" ]] && eval_args+=(--ground-truth "$GT")
    python3 "$EVAL" "${eval_args[@]}"
}

run_one A "" fuse_imu_yaw:=true use_wheel_odom:=true
run_one B "" fuse_imu_yaw:=false use_wheel_odom:=true
run_one C "" fuse_imu_yaw:=true use_wheel_odom:=false
run_one D "$DROP_START,$DROP_END" fuse_imu_yaw:=true use_wheel_odom:=true \
    "uwb_dropout:=$DROP_START,$DROP_END"

python3 "$EVAL" summarize --results "$RESULTS"
echo "Wrote $RESULTS/summary.md"
