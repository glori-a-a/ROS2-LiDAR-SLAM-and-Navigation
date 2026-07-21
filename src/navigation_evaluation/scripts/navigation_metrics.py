#!/usr/bin/env python3
"""Record odometry error samples and write CSV metrics (no fabricated defaults)."""

import csv
import math
import os
from dataclasses import dataclass, field

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


@dataclass
class PoseSample:
    x: float
    y: float
    yaw: float


@dataclass
class MetricsState:
    reference: list[PoseSample] = field(default_factory=list)
    estimate: list[PoseSample] = field(default_factory=list)
    errors: list[float] = field(default_factory=list)


class NavigationMetrics(Node):
    def __init__(self):
        super().__init__("navigation_metrics")
        self.declare_parameter("reference_topic", "/odom")
        self.declare_parameter("estimate_topic", "/odometry/filtered")
        self.declare_parameter("output_csv", "evaluations/metrics.csv")
        self.declare_parameter("max_samples", 500)
        ref_topic = self.get_parameter("reference_topic").value
        est_topic = self.get_parameter("estimate_topic").value
        self._output = self.get_parameter("output_csv").value
        self._max_samples = int(self.get_parameter("max_samples").value)
        self._state = MetricsState()
        self._ref_ready = False
        self._est_ready = False
        self.create_subscription(
            Odometry, ref_topic, self._on_ref, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, est_topic, self._on_est, qos_profile_sensor_data
        )
        self.create_timer(2.0, self._maybe_write)
        self.get_logger().info(
            "Recording %s vs %s -> %s",
            ref_topic,
            est_topic,
            self._output,
        )

    def _on_ref(self, msg: Odometry) -> None:
        self._state.reference.append(self._pose(msg))
        self._ref_ready = True
        self._trim()

    def _on_est(self, msg: Odometry) -> None:
        self._state.estimate.append(self._pose(msg))
        self._est_ready = True
        self._pair_errors()
        self._trim()

    def _trim(self) -> None:
        if len(self._state.reference) > self._max_samples:
            self._state.reference = self._state.reference[-self._max_samples :]
        if len(self._state.estimate) > self._max_samples:
            self._state.estimate = self._state.estimate[-self._max_samples :]

    def _pair_errors(self) -> None:
        n = min(len(self._state.reference), len(self._state.estimate))
        if n == 0:
            return
        self._state.errors.clear()
        for index in range(n):
            ref = self._state.reference[index]
            est = self._state.estimate[index]
            self._state.errors.append(
                math.hypot(est.x - ref.x, est.y - ref.y)
            )

    def _maybe_write(self) -> None:
        if not self._state.errors:
            return
        os.makedirs(os.path.dirname(self._output) or ".", exist_ok=True)
        ate = sum(self._state.errors) / len(self._state.errors)
        rpe_values: list[float] = []
        for index in range(1, len(self._state.errors)):
            rpe_values.append(
                abs(self._state.errors[index] - self._state.errors[index - 1])
            )
        rpe = sum(rpe_values) / len(rpe_values) if rpe_values else 0.0
        with open(self._output, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["metric", "value", "samples"])
            writer.writerow(["ate_m", f"{ate:.6f}", len(self._state.errors)])
            writer.writerow(["rpe_m", f"{rpe:.6f}", len(rpe_values)])
        self.get_logger().info(
            "Wrote %s (ATE=%.4f m, RPE=%.4f m, n=%d)",
            self._output,
            ate,
            rpe,
            len(self._state.errors),
        )

    @staticmethod
    def _pose(msg: Odometry) -> PoseSample:
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return PoseSample(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )


def main() -> None:
    rclpy.init()
    node = NavigationMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
