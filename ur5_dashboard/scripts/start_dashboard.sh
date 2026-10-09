#!/usr/bin/env bash
set -eo pipefail
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WORKSPACE/.dashboard_runtime/ros_logs}"
mkdir -p "$ROS_LOG_DIR"
exec python3 -m ur5_dashboard.server
