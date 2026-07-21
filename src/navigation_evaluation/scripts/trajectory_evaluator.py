#!/usr/bin/env python3

import csv
import math
import os
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


@dataclass
class Sample:
    t: float
    x: float
    y: float
    yaw: float


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class TrajectoryEvaluator(Node):
    def __init__(self):
        super().__init__("trajectory_evaluator")
        self.declare_parameter("reference_topic", "/ground_truth/odom")
        self.declare_parameter("estimate_topic", "/odom")
        self.declare_parameter("output_csv", "evaluations/results/trajectory_row.csv")
        self.declare_parameter("scenario", "unspecified")
        self.declare_parameter("fusion_mode", "wheel")
        self.declare_parameter("lidar_mode", "clean")
        self.declare_parameter("duration_sec", 25)
        self.declare_parameter("max_time_diff_sec", 0.06)

        ref_topic = self.get_parameter("reference_topic").value
        est_topic = self.get_parameter("estimate_topic").value
        self._output = self.get_parameter("output_csv").value
        self._scenario = self.get_parameter("scenario").value
        self._fusion = self.get_parameter("fusion_mode").value
        self._lidar = self.get_parameter("lidar_mode").value
        self._max_dt = float(self.get_parameter("max_time_diff_sec").value)
        duration = float(self.get_parameter("duration_sec").value)

        self._reference: list[Sample] = []
        self._estimate: list[Sample] = []

        self.create_subscription(Odometry, ref_topic, self._on_ref, qos_profile_sensor_data)
        self.create_subscription(Odometry, est_topic, self._on_est, qos_profile_sensor_data)
        self.create_timer(duration, self._finish)

    def _on_ref(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._reference.append(
            Sample(t, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quat(msg.pose.pose.orientation))
        )

    def _on_est(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._estimate.append(
            Sample(t, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_from_quat(msg.pose.pose.orientation))
        )

    def _nearest(self, t: float, samples: list[Sample]) -> Sample | None:
        if not samples:
            return None
        best = min(samples, key=lambda s: abs(s.t - t))
        if abs(best.t - t) > self._max_dt:
            return None
        return best

    def _finish(self) -> None:
        errors_x: list[float] = []
        errors_y: list[float] = []
        errors_yaw: list[float] = []
        errors_xy: list[float] = []

        for est in self._estimate:
            ref = self._nearest(est.t, self._reference)
            if ref is None:
                continue
            ex = est.x - ref.x
            ey = est.y - ref.y
            eyaw = angle_diff(est.yaw, ref.yaw)
            errors_x.append(ex)
            errors_y.append(ey)
            errors_yaw.append(eyaw)
            errors_xy.append(math.hypot(ex, ey))

        n = len(errors_xy)
        if n == 0:
            self.get_logger().error(
                f"No aligned samples; is {self.get_parameter('reference_topic').value} publishing?"
            )
            row = {
                "scenario": self._scenario,
                "fusion_mode": self._fusion,
                "lidar_mode": self._lidar,
                "samples": 0,
                "ate_xy_m": "",
                "rmse_x_m": "",
                "rmse_y_m": "",
                "rmse_yaw_rad": "",
            }
        else:
            ate = sum(errors_xy) / n
            rmse_x = math.sqrt(sum(e * e for e in errors_x) / n)
            rmse_y = math.sqrt(sum(e * e for e in errors_y) / n)
            rmse_yaw = math.sqrt(sum(e * e for e in errors_yaw) / n)
            row = {
                "scenario": self._scenario,
                "fusion_mode": self._fusion,
                "lidar_mode": self._lidar,
                "samples": n,
                "ate_xy_m": f"{ate:.6f}",
                "rmse_x_m": f"{rmse_x:.6f}",
                "rmse_y_m": f"{rmse_y:.6f}",
                "rmse_yaw_rad": f"{rmse_yaw:.6f}",
            }
            self.get_logger().info(
                f"n={n} ATE={ate:.4f} RMSE x/y/yaw={rmse_x:.4f}/{rmse_y:.4f}/{rmse_yaw:.4f}"
            )

        os.makedirs(os.path.dirname(self._output) or ".", exist_ok=True)
        write_header = not os.path.exists(self._output)
        with open(self._output, "a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
            if write_header:
                writer.writeheader()
            writer.writerow(row)

        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = TrajectoryEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
