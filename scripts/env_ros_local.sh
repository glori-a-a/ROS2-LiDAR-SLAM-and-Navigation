#!/usr/bin/env bash
# Source ROS + optional local overlay (when apt packages are not installed system-wide).
set -eo pipefail

ROOT_NAV2="${NAV_LOCAL_ROOT_NAV2:-/home/gloriaa/navigation/.local/root-nav2}"
ROOT_SLAM="${NAV_LOCAL_ROOT:-/home/gloriaa/navigation/.local/root-phase4}"
export PATH="/usr/bin:/bin:/opt/ros/humble/bin:${PATH}"

overlay_paths=()
for root in "$ROOT_NAV2" "$ROOT_SLAM"; do
  if [[ -d "$root/opt/ros/humble" ]]; then
    overlay_paths+=("$root/opt/ros/humble")
    export LD_LIBRARY_PATH="${root}/usr/lib:${root}/usr/lib/x86_64-linux-gnu:${root}/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
  fi
done

if ((${#overlay_paths[@]} > 0)); then
  IFS=: read -r -a _existing <<< "${AMENT_PREFIX_PATH:-}"
  export AMENT_PREFIX_PATH="$(IFS=:; echo "${overlay_paths[*]}"):/opt/ros/humble"
fi

export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/gloriaa/navigation/install/setup.bash
set -u
