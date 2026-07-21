#!/usr/bin/env bash
# Collect 9-row trajectory_metrics.csv only (~45–60 min). No full colcon rebuild loop.
set +e
set -u
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# shellcheck disable=SC1091
source "$ROOT/scripts/env_ros_local.sh"
chmod +x "$ROOT"/src/robot_bringup/scripts/*.py "$ROOT"/src/navigation_evaluation/scripts/*.py 2>/dev/null || true
colcon build --packages-select robot_bringup navigation_evaluation --allow-overriding robot_bringup navigation_evaluation
set +u
source "$ROOT/install/setup.bash"
set +u

TRAJ_CSV="$ROOT/evaluations/results/trajectory_metrics.csv"
mkdir -p "$ROOT/evaluations/results"
rm -f "$TRAJ_CSV"

# shellcheck disable=SC1091
source "$ROOT/scripts/kill_ros_stack.sh"

run_case() {
  local fusion="$1" lidar="$2" range_std="$3" dropout="$4"
  local scenario="${fusion}_${lidar}"

  source "$ROOT/scripts/kill_ros_stack.sh"
  sleep 2

  local use_noise=false publish_odom_tf=true
  [[ "$lidar" != clean ]] && use_noise=true
  [[ "$fusion" != wheel ]] && publish_odom_tf=false

  ros2 launch robot_bringup simulation.launch.py headless:=true \
    use_scan_noise:="$use_noise" publish_odom_tf:="$publish_odom_tf" &
  sleep 14

  if [[ "$use_noise" == true ]]; then
    ros2 param set /noise_injection range_noise_std "$range_std" 2>/dev/null || true
    ros2 param set /noise_injection dropout_probability "$dropout" 2>/dev/null || true
  fi

  [[ "$fusion" != wheel ]] && ros2 launch robot_bringup ekf.launch.py fusion_mode:="$fusion" & sleep 5
  [[ "$fusion" == wheel_imu_icp ]] && ros2 launch icp_odometry icp_odometry.launch.py & sleep 4

  ros2 run robot_bringup send_test_velocity.py --ros-args -p duration_sec:=18 -p linear_x:=0.12 &

  local est=/odom
  [[ "$fusion" != wheel ]] && est=/odometry/filtered

  echo ">>> Evaluating $scenario (estimate $est) ..."
  if ros2 run navigation_evaluation trajectory_evaluator.py --ros-args \
    -p reference_topic:=/ground_truth/odom \
    -p estimate_topic:="$est" \
    -p scenario:="$scenario" \
    -p fusion_mode:="$fusion" \
    -p lidar_mode:="$lidar" \
    -p duration_sec:=22 \
    -p output_csv:="$TRAJ_CSV"; then
    echo ">>> OK $scenario"
  else
    echo ">>> FAILED $scenario (see above)"
  fi

  source "$ROOT/scripts/kill_ros_stack.sh"
  sleep 2
}

for lidar in clean gaussian dropout; do
  range_std=0.0 dropout=0.0
  [[ "$lidar" == gaussian ]] && range_std=0.02
  [[ "$lidar" == dropout ]] && dropout=0.08
  for fusion in wheel wheel_imu wheel_imu_icp; do
    run_case "$fusion" "$lidar" "$range_std" "$dropout"
  done
done

rows=0
[[ -f "$TRAJ_CSV" ]] && rows=$(($(wc -l <"$TRAJ_CSV") - 1))
echo ""
echo "Trajectory CSV: $TRAJ_CSV ($rows / 9 rows)"
/usr/bin/python3 "$ROOT/src/navigation_evaluation/scripts/plot_trajectory_metrics.py" "$TRAJ_CSV" "$ROOT/evaluations/plots" 2>/dev/null || true
/usr/bin/python3 "$ROOT/scripts/render_resume_metrics.py" 2>/dev/null || true

if [[ "$rows" -lt 9 ]]; then
  echo "ERROR: need 9 rows for resume table." >&2
  exit 1
fi
echo "Done. Open evaluations/results/RESUME_METRICS.md"
