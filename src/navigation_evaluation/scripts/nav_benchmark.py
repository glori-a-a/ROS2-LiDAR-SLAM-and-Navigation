#!/usr/bin/env python3

import csv
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavBenchmark(Node):
    def __init__(self):
        super().__init__("nav_benchmark")
        self.declare_parameter("output_json", "evaluations/results/nav_benchmark.json")
        self.declare_parameter("goal_timeout_sec", 120.0)
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._initial_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._results: list[dict] = []

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        for _ in range(5):
            self._initial_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.2)

    def navigate(self, x: float, y: float, yaw: float) -> dict:
        timeout = float(self.get_parameter("goal_timeout_sec").value)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        t0 = time.time()
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=30.0)
        if not send_future.done() or not send_future.result().accepted:
            return {"x": x, "y": y, "success": False, "time_sec": time.time() - t0, "status": -1}

        handle = send_future.result()
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        elapsed = time.time() - t0
        if not result_future.done():
            return {"x": x, "y": y, "success": False, "time_sec": elapsed, "status": -2}
        status = result_future.result().status
        return {
            "x": x,
            "y": y,
            "success": status == GoalStatus.STATUS_SUCCEEDED,
            "time_sec": elapsed,
            "status": int(status),
        }


def main() -> None:
    rclpy.init()
    node = NavBenchmark()
    out_path = node.get_parameter("output_json").value
    if not node._client.wait_for_server(timeout_sec=90.0):
        node.get_logger().error("navigate_to_pose unavailable")
        raise SystemExit(1)

    node.publish_initial_pose(0.0, 0.0, 0.0)
    time.sleep(8.0)
    goals = [(0.6, 0.0, 0.0), (0.4, 0.6, 1.2), (-0.8, 0.5, 0.0)]
    for x, y, yaw in goals:
        node._results.append(node.navigate(x, y, yaw))

    passed = sum(1 for r in node._results if r["success"])
    summary = {
        "goals": node._results,
        "success_rate": passed / len(node._results),
        "passed": passed,
        "total": len(node._results),
    }
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    csv_path = out_path.replace(".json", ".csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["x", "y", "success", "time_sec", "status"])
        writer.writeheader()
        for row in node._results:
            writer.writerow(row)

    node.get_logger().info(f"Navigation benchmark {passed}/{len(node._results)}")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed == len(node._results) else 1)


if __name__ == "__main__":
    main()
