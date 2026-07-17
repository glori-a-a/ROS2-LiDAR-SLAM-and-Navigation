#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def planar_distance(first, latest):
    if first is None or latest is None:
        return 0.0

    dx = latest.pose.pose.position.x - first.pose.pose.position.x
    dy = latest.pose.pose.position.y - first.pose.pose.position.y
    return math.hypot(dx, dy)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("send_test_velocity")

    node.declare_parameter("linear_x", 0.15)
    node.declare_parameter("duration_sec", 3.0)
    node.declare_parameter("cmd_vel_topic", "/cmd_vel")

    linear_x = float(node.get_parameter("linear_x").value)
    duration_sec = float(node.get_parameter("duration_sec").value)
    cmd_vel_topic = str(node.get_parameter("cmd_vel_topic").value)

    odom_first = {"msg": None}
    odom_latest = {"msg": None}

    def odom_callback(msg):
        if odom_first["msg"] is None:
            odom_first["msg"] = msg
        odom_latest["msg"] = msg

    node.create_subscription(
        Odometry,
        "/odom",
        odom_callback,
        qos_profile_sensor_data,
    )

    publisher = node.create_publisher(Twist, cmd_vel_topic, 10)

    move_cmd = Twist()
    move_cmd.linear.x = linear_x

    stop_cmd = Twist()

    node.get_logger().info(
        f"Publishing {linear_x:.2f} m/s to {cmd_vel_topic} for {duration_sec:.1f} seconds"
    )

    try:
        end_time = time.monotonic() + duration_sec
        while time.monotonic() < end_time:
            publisher.publish(move_cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
    finally:
        for _ in range(10):
            publisher.publish(stop_cmd)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)

    moved = planar_distance(odom_first["msg"], odom_latest["msg"])
    if odom_first["msg"] is None:
        node.get_logger().warning("/odom was not received during the motion test")
        exit_code = 1
    else:
        node.get_logger().info(f"Observed odometry position change: {moved:.3f} m")
        exit_code = 0 if moved > 0.01 else 1

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
