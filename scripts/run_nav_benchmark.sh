#!/usr/bin/env bash
set +e
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/env_ros_local.sh"
chmod +x "$ROOT"/src/navigation_evaluation/scripts/*.py 2>/dev/null || true
colcon build --packages-select navigation_evaluation --allow-overriding navigation_evaluation
set +u
source "$ROOT/install/setup.bash"
# shellcheck disable=SC1091
source "$ROOT/scripts/kill_ros_stack.sh"
sleep 2
ros2 launch slam_navigation navigation.launch.py headless:=true use_ekf:=false &
sleep 80
ros2 run navigation_evaluation nav_benchmark.py --ros-args \
  -p output_json:="$ROOT/evaluations/results/nav_benchmark.json"
code=$?
source "$ROOT/scripts/kill_ros_stack.sh"
/usr/bin/python3 "$ROOT/scripts/render_resume_metrics.py" || true
exit "$code"
