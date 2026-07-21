#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/env_ros_local.sh"
echo "ROS workspace: $ROOT"
echo "python3: $(command -v python3) ($(python3 --version 2>&1))"
echo "robot_bringup: $(ros2 pkg prefix robot_bringup 2>/dev/null || echo MISSING)"
