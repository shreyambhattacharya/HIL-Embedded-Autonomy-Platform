#!/usr/bin/env python3
"""Check required Milestone 1 topics against an already-running stack."""

import math
import sys
import time
from typing import Set

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float64


class Milestone01SmokeTest(Node):
    """Small observable-behavior check; it does not claim HIL coverage."""

    def __init__(self) -> None:
        super().__init__("milestone_01_smoke_test")
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 25.0).value)
        self.required_topics: Set[str] = {
            "wheel_states",
            "imu",
            "scan",
            "odom",
            "left_effort",
            "right_effort",
        }
        self.received: Set[str] = set()
        self.nonfinite = False
        self.saw_motion = False
        self.saw_nonzero_effort = False
        self.last_nonzero_effort_time = 0.0
        self.first_position = None
        self.min_x = math.inf
        self.max_x = -math.inf
        self.min_y = math.inf
        self.max_y = -math.inf

        sensor_qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(JointState, "/hil/sensors/wheel_states", self._wheel_cb, sensor_qos)
        self.create_subscription(Imu, "/hil/sensors/imu", self._imu_cb, sensor_qos)
        self.create_subscription(LaserScan, "/hil/sensors/scan", self._scan_cb, sensor_qos)
        self.create_subscription(Odometry, "/hil/ground_truth/odom", self._odom_cb, sensor_qos)
        self.create_subscription(Float64, "/hil/actuator/left_effort", self._left_effort_cb, 10)
        self.create_subscription(Float64, "/hil/actuator/right_effort", self._right_effort_cb, 10)

    def _finite(self, values) -> bool:
        return all(math.isfinite(float(value)) for value in values)

    def _wheel_cb(self, message: JointState) -> None:
        self.received.add("wheel_states")
        if not self._finite(message.position) or not self._finite(message.velocity):
            self.nonfinite = True

    def _imu_cb(self, message: Imu) -> None:
        self.received.add("imu")
        values = [
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ]
        if not self._finite(values):
            self.nonfinite = True

    def _scan_cb(self, message: LaserScan) -> None:
        self.received.add("scan")
        if not self._finite(message.ranges):
            self.nonfinite = True

    def _odom_cb(self, message: Odometry) -> None:
        self.received.add("odom")
        values = [
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        ]
        if not self._finite(values):
            self.nonfinite = True
            return
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        if self.first_position is None:
            self.first_position = (x, y)
        self.min_x = min(self.min_x, x)
        self.max_x = max(self.max_x, x)
        self.min_y = min(self.min_y, y)
        self.max_y = max(self.max_y, y)
        if self.first_position is not None:
            distance = math.hypot(x - self.first_position[0], y - self.first_position[1])
            self.saw_motion = self.saw_motion or distance > 0.05

    def _effort_cb(self, name: str, value: float) -> None:
        self.received.add(name)
        if not math.isfinite(value):
            self.nonfinite = True
        if abs(value) > 1.0e-4:
            self.saw_nonzero_effort = True
            self.last_nonzero_effort_time = time.monotonic()

    def _left_effort_cb(self, message: Float64) -> None:
        self._effort_cb("left_effort", float(message.data))

    def _right_effort_cb(self, message: Float64) -> None:
        self._effort_cb("right_effort", float(message.data))


def main() -> int:
    rclpy.init()
    node = Milestone01SmokeTest()
    deadline = time.monotonic() + node.timeout_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.nonfinite:
                print("FAIL: non-finite value observed")
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    missing = sorted(node.required_topics - node.received)
    if missing:
        print("FAIL: missing messages on " + ", ".join(missing))
        return 1
    if not node.saw_nonzero_effort:
        print("FAIL: no non-zero actuator effort observed")
        return 1
    if not node.saw_motion:
        print("FAIL: ground-truth odometry did not show motion")
        return 1
    if time.monotonic() - node.last_nonzero_effort_time < 1.0:
        print("FAIL: actuator effort had not returned to zero before timeout")
        return 1

    print("PASS: required sensors, efforts, finite values, motion, and stop behavior observed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
