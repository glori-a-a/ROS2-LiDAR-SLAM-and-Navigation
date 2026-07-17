import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from noise_injection.noise import apply_scan_noise


class LaserScanNoiseInjectionNode(Node):
    INPUT_TOPIC = "/scan_raw"

    def __init__(self):
        super().__init__("noise_injection")

        self.declare_parameter("range_noise_std", 0.0)
        self.declare_parameter("dropout_probability", 0.0)
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("random_seed", 0)

        self._range_noise_std = float(
            self.get_parameter("range_noise_std").value
        )
        self._dropout_probability = float(
            self.get_parameter("dropout_probability").value
        )
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._random_seed = int(self.get_parameter("random_seed").value)
        self._rng = np.random.default_rng(self._random_seed)

        self._publisher = self.create_publisher(LaserScan, self._output_topic, 10)
        self._subscription = self.create_subscription(
            LaserScan,
            self.INPUT_TOPIC,
            self._scan_callback,
            10,
        )

        self.get_logger().info(
            "Injecting LaserScan noise from "
            f"{self.INPUT_TOPIC} to {self._output_topic}"
        )

    def _scan_callback(self, scan_msg):
        noisy_ranges = apply_scan_noise(
            scan_msg.ranges,
            scan_msg.range_min,
            scan_msg.range_max,
            self._range_noise_std,
            self._dropout_probability,
            self._rng,
        )

        output_msg = LaserScan()
        output_msg.header = scan_msg.header
        output_msg.angle_min = scan_msg.angle_min
        output_msg.angle_max = scan_msg.angle_max
        output_msg.angle_increment = scan_msg.angle_increment
        output_msg.time_increment = scan_msg.time_increment
        output_msg.scan_time = scan_msg.scan_time
        output_msg.range_min = scan_msg.range_min
        output_msg.range_max = scan_msg.range_max
        output_msg.ranges = noisy_ranges.tolist()
        output_msg.intensities = scan_msg.intensities

        self._publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanNoiseInjectionNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
