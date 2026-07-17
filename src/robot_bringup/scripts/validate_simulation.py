#!/usr/bin/env python3

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformListener


def topic_rate(receive_times):
    if len(receive_times) < 2:
        return 0.0

    elapsed = receive_times[-1] - receive_times[0]
    if elapsed <= 0.0:
        return 0.0

    return (len(receive_times) - 1) / elapsed


def status_line(topic, data, frame_text):
    if data["count"] == 0:
        return f"[MISSING] {topic} not received"

    return (
        f"[OK] {topic} received, {frame_text}, "
        f"rate={topic_rate(data['times']):.1f} Hz"
    )


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("simulation_validator")

    node.declare_parameter("duration_sec", 5.0)
    duration_sec = float(node.get_parameter("duration_sec").value)

    received = {
        "/scan_raw": {"count": 0, "times": [], "frame": ""},
        "/scan": {"count": 0, "times": [], "frame": ""},
        "/imu/data": {"count": 0, "times": [], "frame": ""},
        "/odom": {
            "count": 0,
            "times": [],
            "frame": "",
            "child_frame": "",
        },
        "/joint_states": {"count": 0, "times": [], "frame": "n/a"},
    }

    def mark(topic, frame_id=""):
        received[topic]["count"] += 1
        received[topic]["times"].append(time.monotonic())
        if frame_id:
            received[topic]["frame"] = frame_id

    def scan_raw_callback(msg):
        mark("/scan_raw", msg.header.frame_id)

    def scan_callback(msg):
        mark("/scan", msg.header.frame_id)

    def imu_callback(msg):
        mark("/imu/data", msg.header.frame_id)

    def odom_callback(msg):
        mark("/odom", msg.header.frame_id)
        received["/odom"]["child_frame"] = msg.child_frame_id

    def joint_states_callback(msg):
        mark("/joint_states")

    node.create_subscription(
        LaserScan,
        "/scan_raw",
        scan_raw_callback,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        LaserScan,
        "/scan",
        scan_callback,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Imu,
        "/imu/data",
        imu_callback,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Odometry,
        "/odom",
        odom_callback,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        JointState,
        "/joint_states",
        joint_states_callback,
        qos_profile_sensor_data,
    )

    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    node.get_logger().info(f"Collecting simulation data for {duration_sec:.1f} seconds")
    end_time = time.monotonic() + duration_sec
    while time.monotonic() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(status_line("/scan_raw", received["/scan_raw"], f"frame={received['/scan_raw']['frame']}"))
    print(status_line("/scan", received["/scan"], f"frame={received['/scan']['frame']}"))
    print(status_line("/imu/data", received["/imu/data"], f"frame={received['/imu/data']['frame']}"))
    print(
        status_line(
            "/odom",
            received["/odom"],
            (
                f"frame={received['/odom']['frame']}, "
                f"child_frame={received['/odom']['child_frame']}"
            ),
        )
    )
    print(status_line("/joint_states", received["/joint_states"], "frame=n/a"))

    required_transforms = [
        ("odom", "base_link"),
        ("base_link", "laser"),
        ("base_link", "imu_link"),
    ]

    missing_transforms = []
    for target_frame, source_frame in required_transforms:
        available = tf_buffer.can_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        if available:
            print(f"[OK] TF {target_frame} -> {source_frame} available")
        else:
            print(f"[MISSING] TF {target_frame} -> {source_frame} unavailable")
            missing_transforms.append((target_frame, source_frame))

    missing_topics = [
        topic for topic, data in received.items() if data["count"] == 0
    ]
    if missing_topics:
        node.get_logger().error(
            "Missing required topics: " + ", ".join(missing_topics)
        )
        exit_code = 1
    elif missing_transforms:
        node.get_logger().error(
            "Missing required TF transforms: "
            + ", ".join(
                f"{target} -> {source}" for target, source in missing_transforms
            )
        )
        exit_code = 1
    else:
        exit_code = 0

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
