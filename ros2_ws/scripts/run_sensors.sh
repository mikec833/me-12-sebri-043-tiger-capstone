#!/usr/bin/env bash
# Start every sensor node (bringup.launch.py) and record them to a bag.
# The bag lands in ros2_ws/bags/run_<run_id>, the CSVs in
# ros2_ws/src/outputs/run_<run_id>. Ctrl-C stops everything cleanly.
#
# Extra arguments are passed straight to the launch file, e.g.
#   ./scripts/run_sensors.sh teensy_port:=/dev/ttyACM1 uwb_port:=/dev/ttyACM0
#   ./scripts/run_sensors.sh record:=false      # nodes only, no bag
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
    echo "Error: $WS_DIR/install/setup.bash not found. Run 'colcon build' in $WS_DIR first."
    exit 1
fi

# Source ROS itself if this shell hasn't already
if [[ -z "${ROS_DISTRO:-}" ]]; then
    ros_setup=(/opt/ros/*/setup.bash)
    if [[ ! -f "${ros_setup[0]}" ]]; then
        echo "Error: no ROS 2 install found under /opt/ros."
        exit 1
    fi
    source "${ros_setup[0]}"
fi
source "$WS_DIR/install/setup.bash"

# The nodes write CSVs relative to the working directory, so run from ros2_ws/
cd "$WS_DIR"
mkdir -p bags

exec ros2 launch ballrobot_pkg bringup.launch.py record:=true "$@"
