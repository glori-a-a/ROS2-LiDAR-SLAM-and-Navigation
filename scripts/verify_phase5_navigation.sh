#!/usr/bin/env bash
set -eo pipefail
REPO="/home/gloriaa/navigation"
LOG="$REPO/log/phase5_verify"
mkdir -p "$LOG"
# shellcheck disable=SC1091
source "$REPO/scripts/env_ros_local.sh"

pkill -9 gzserver 2>/dev/null || true
pkill -9 -f "navigation.launch" 2>/dev/null || true
sleep 2

ros2 launch slam_navigation navigation.launch.py headless:=true >"$LOG/launch.log" 2>&1 &
LPID=$!
cleanup(){ kill "$LPID" 2>/dev/null || true; pkill -9 gzserver 2>/dev/null || true; }
trap cleanup EXIT

echo "Waiting for Nav2 stack..."
sleep 40

ros2 run robot_bringup nav_three_goals.py --ros-args -p goal_timeout_sec:=90.0 &
NAVPID=$!
sleep 5

{
  echo "=== lifecycle ==="
  ros2 lifecycle get /map_server 2>&1 || true
  ros2 lifecycle get /amcl 2>&1 || true
  ros2 lifecycle get /bt_navigator 2>&1 || true
  ros2 lifecycle get /controller_server 2>&1 || true

  echo "=== tf map odom (after initial pose) ==="
  timeout 12s ros2 run tf2_ros tf2_echo map odom 2>&1 | head -18 || true

  wait "$NAVPID" || true

  echo "=== plan topic ==="
  timeout 5s ros2 topic echo /plan --once 2>&1 | head -10 || echo "no /plan sample"

  echo "=== cmd_vel pubs ==="
  ros2 topic info /cmd_vel -v 2>&1 | head -25 || true
} | tee "$LOG/verify.log"

echo "Logs in $LOG"
