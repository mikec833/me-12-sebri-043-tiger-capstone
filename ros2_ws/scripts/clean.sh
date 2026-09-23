#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ ! -d "$WS_DIR/src" ]]; then
    echo "Error: $WS_DIR is not a ROS 2 workspace."
    exit 1
fi

rm -rf -- "$WS_DIR/build" "$WS_DIR/log" "$WS_DIR/install"
echo "Deleted $WS_DIR/build, $WS_DIR/log and $WS_DIR/install"