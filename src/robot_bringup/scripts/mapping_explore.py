#!/usr/bin/env python3
"""Publish a simple /cmd_vel pattern to explore the indoor world for mapping."""

import time

import rclpy
from geometry_msgs.msg import Twist


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("mapping_explore")

    node.declare_parameter("linear_x", 0.12)
    node.declare_parameter("angular_z", 0.35)
    node.declare_parameter("forward_sec", 3.0)
    node.declare_parameter("turn_sec", 2.5)
    node.declare_parameter("cycles", 4)
    node.declare_parameter("cmd_vel_topic", "/cmd_vel")

    linear_x = float(node.get_parameter("linear_x").value)
    angular_z = float(node.get_parameter("angular_z").value)
    forward_sec = float(node.get_parameter("forward_sec").value)
    turn_sec = float(node.get_parameter("turn_sec").value)
    cycles = int(node.get_parameter("cycles").value)
    cmd_vel_topic = str(node.get_parameter("cmd_vel_topic").value)

    publisher = node.create_publisher(Twist, cmd_vel_topic, 10)
    stop = Twist()

    def publish_for(duration_sec, cmd):
        end_time = time.monotonic() + duration_sec
        while time.monotonic() < end_time and rclpy.ok():
            publisher.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

    forward = Twist()
    forward.linear.x = linear_x
    turn = Twist()
    turn.angular.z = angular_z

    node.get_logger().info(
        f"Exploration: {cycles} cycles of "
        f"{forward_sec:.1f}s forward + {turn_sec:.1f}s turn"
    )

    try:
        for cycle in range(cycles):
            node.get_logger().info(f"Cycle {cycle + 1}/{cycles}")
            publish_for(forward_sec, forward)
            publish_for(turn_sec, turn)
    finally:
        for _ in range(10):
            publisher.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
