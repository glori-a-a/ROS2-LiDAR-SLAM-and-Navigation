#!/usr/bin/env python3

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("scan_relay")

    node.declare_parameter("input_topic", "/scan_raw")
    node.declare_parameter("output_topic", "/scan")

    input_topic = str(node.get_parameter("input_topic").value)
    output_topic = str(node.get_parameter("output_topic").value)

    publisher = node.create_publisher(
        LaserScan,
        output_topic,
        qos_profile_sensor_data,
    )

    def relay_scan(scan_msg):
        publisher.publish(scan_msg)

    node.create_subscription(
        LaserScan,
        input_topic,
        relay_scan,
        qos_profile_sensor_data,
    )

    node.get_logger().info(f"Relaying {input_topic} to {output_topic}")

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
