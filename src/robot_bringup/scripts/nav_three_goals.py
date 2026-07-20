#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavThreeGoals(Node):
    def __init__(self):
        super().__init__("nav_three_goals")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("goal_timeout_sec", 120.0)

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._initial_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

    def publish_initial_pose(self):
        x = float(self.get_parameter("initial_x").value)
        y = float(self.get_parameter("initial_y").value)
        yaw = float(self.get_parameter("initial_yaw").value)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        for index in range(36):
            msg.pose.covariance[index] = 0.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        for _ in range(5):
            self._initial_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.2)
        self.get_logger().info(f"Published /initialpose at ({x:.2f}, {y:.2f}, {yaw:.2f})")

    def navigate_to(self, x, y, yaw, timeout_sec):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"Sending goal ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=30.0)
        if not send_future.done():
            self.get_logger().error("Timed out waiting for goal acceptance")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error("Timed out waiting for navigation result")
            return False

        status = result_future.result().status
        success = status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f"Goal ({x:.2f}, {y:.2f}) finished with status={status} success={success}"
        )
        return success


def main(args=None):
    rclpy.init(args=args)
    node = NavThreeGoals()
    timeout_sec = float(node.get_parameter("goal_timeout_sec").value)

    if not node._client.wait_for_server(timeout_sec=60.0):
        node.get_logger().error("navigate_to_pose action server not available")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    node.publish_initial_pose()
    time.sleep(5.0)

    goals = [
        (0.6, 0.0, 0.0),
        (0.4, 0.6, 1.2),
        (-0.8, 0.5, 0.0),
    ]

    results = []
    for x, y, yaw in goals:
        results.append(node.navigate_to(x, y, yaw, timeout_sec))

    passed = sum(1 for ok in results if ok)
    node.get_logger().info(f"Navigation summary: {passed}/{len(results)} goals succeeded")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed == len(results) else 1)


if __name__ == "__main__":
    main()
