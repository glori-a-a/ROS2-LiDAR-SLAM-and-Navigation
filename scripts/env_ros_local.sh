#!/usr/bin/env bash
# Source ROS + optional local overlay (when apt packages are not installed system-wide).
set -eo pipefail

while [[ -n "${CONDA_DEFAULT_ENV:-}" ]]; do
  conda deactivate >/dev/null 2>&1 || break
done
unset PYTHONPATH
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:${PATH}"
ROOT_NAV2="${NAV_LOCAL_ROOT_NAV2:-/home/gloriaa/navigation/.local/root-nav2}"
ROOT_SLAM="${NAV_LOCAL_ROOT:-/home/gloriaa/navigation/.local/root-phase4}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

for root in "$ROOT_NAV2" "$ROOT_SLAM"; do
  if [[ -d "$root/opt/ros/humble" ]] || [[ -d "$root/usr/lib" ]]; then
    export LD_LIBRARY_PATH="${root}/usr/lib/x86_64-linux-gnu:${root}/usr/lib:${root}/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
  fi
done
export AMENT_PREFIX_PATH="${ROOT_NAV2}/opt/ros/humble:${ROOT_SLAM}/opt/ros/humble:/opt/ros/humble"

export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/gloriaa/navigation/install/setup.bash
set +eu
