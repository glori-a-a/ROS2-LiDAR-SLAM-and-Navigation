#!/usr/bin/env bash
# Stop Gazebo + this workspace's ROS stacks (safe patterns — does not pkill "ros2 launch").
pkill -9 gzserver 2>/dev/null || true
pkill -9 -f "robot_bringup/simulation" 2>/dev/null || true
pkill -9 -f "robot_bringup/ekf" 2>/dev/null || true
pkill -9 -f "icp_odometry/icp_odometry" 2>/dev/null || true
pkill -9 -f "slam_navigation/navigation" 2>/dev/null || true
pkill -9 -f "trajectory_evaluator.py" 2>/dev/null || true
pkill -9 -f "nav_benchmark.py" 2>/dev/null || true
pkill -9 -f "send_test_velocity.py" 2>/dev/null || true
for proc in ekf_filter_node amcl planner_server controller_server bt_navigator \
  behavior_server smoother_server waypoint_follower velocity_smoother map_server; do
  pkill -9 -x "$proc" 2>/dev/null || true
done
sleep 1
