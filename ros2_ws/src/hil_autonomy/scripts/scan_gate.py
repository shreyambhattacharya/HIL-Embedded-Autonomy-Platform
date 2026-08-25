#!/usr/bin/env python3
"""Test-only LiDAR relay that intentionally stops publishing after a delay."""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanGate(Node):
    def __init__(self):
        super().__init__("milestone_05a_scan_gate")
        self.drop_after_sec = float(self.declare_parameter("drop_after_sec", 3.0).value)
        self.output_topic = str(self.declare_parameter("output_topic", "/hil/autonomy/test_scan").value)
        self.first_scan_time = None
        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan, "/hil/sensors/scan", self.callback, rclpy.qos.qos_profile_sensor_data
        )
        self.get_logger().info(
            f"test-only LiDAR gate forwarding to {self.output_topic} for {self.drop_after_sec:.2f}s"
        )

    def callback(self, message):
        current_time = self.get_clock().now().nanoseconds * 1.0e-9
        if self.first_scan_time is None:
            if (not message.ranges or message.angle_increment <= 0.0 or
                    not any((math.isinf(value) and value > 0.0) or
                            (math.isfinite(value) and value > 0.0)
                            for value in message.ranges)):
                return
            self.first_scan_time = current_time
        if current_time - self.first_scan_time <= self.drop_after_sec:
            self.publisher.publish(message)


def main():
    rclpy.init()
    node = ScanGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
