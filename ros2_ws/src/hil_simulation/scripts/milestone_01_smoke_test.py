#!/usr/bin/env python3
"""Trigger and validate the complete Milestone 1 simulation sequence."""

import json
import math
import sys
import time
from typing import Dict, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


Pose2D = Tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def planar_yaw(message: Odometry) -> float:
    """Extract yaw from an odometry quaternion."""
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def projected_displacement(start: Pose2D, end: Pose2D) -> float:
    """Project planar displacement along the heading at the start."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    return dx * math.cos(start[2]) + dy * math.sin(start[2])


class Milestone01SmokeTest(Node):
    """Coordinate and validate the deterministic Milestone 1 profile."""

    def __init__(self) -> None:
        super().__init__("milestone_01_smoke_test")
        self.readiness_timeout_sec = float(
            self.declare_parameter("readiness_timeout_sec", 30.0).value
        )
        self.stationary_observation_sec = float(
            self.declare_parameter("stationary_observation_sec", 2.0).value
        )
        self.profile_timeout_sec = float(
            self.declare_parameter("profile_timeout_sec", 30.0).value
        )
        self.stop_observation_sec = float(
            self.declare_parameter("stop_observation_sec", 2.0).value
        )
        self.max_effort_nm = float(
            self.declare_parameter("max_effort_nm", 1.5).value
        )

        self.required_topics: Set[str] = {
            "target_twist",
            "wheel_states",
            "imu",
            "scan",
            "odom",
            "left_effort",
            "right_effort",
        }
        self.received: Set[str] = set()
        self.invalid_reason: Optional[str] = None
        self.wheel_names_valid = False
        self.latest_pose: Optional[Pose2D] = None
        self.latest_linear_speed = math.inf
        self.latest_yaw_rate = math.inf
        self.latest_efforts = {"left": math.inf, "right": math.inf}
        self.latest_wheels = {"left": math.inf, "right": math.inf}

        self.stage = "readiness"
        self.stationary_origin: Optional[Pose2D] = None
        self.stationary_max_drift = 0.0
        self.stationary_max_speed = 0.0
        self.stationary_max_yaw_rate = 0.0
        self.stationary_max_effort = 0.0
        self.max_abs_roll = 0.0
        self.max_abs_pitch = 0.0

        self.current_phase: Optional[str] = None
        self.phase_order = []
        self.phase_pose: Dict[str, Dict[str, Pose2D]] = {}
        self.phase_wheels: Dict[str, Dict[str, float]] = {}
        self.seen_turn = False
        self.seen_forward_2 = False
        self.complete_started_wall: Optional[float] = None
        self.max_abs_effort = 0.0
        self.saw_nonzero_effort = False

        sensor_qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(
            Twist, "/hil/control/target_twist", self._target_cb, 10
        )
        self.create_subscription(
            JointState,
            "/hil/sensors/wheel_states",
            self._wheel_cb,
            sensor_qos,
        )
        self.create_subscription(
            Imu, "/hil/sensors/imu", self._imu_cb, sensor_qos
        )
        self.create_subscription(
            LaserScan, "/hil/sensors/scan", self._scan_cb, sensor_qos
        )
        self.create_subscription(
            Odometry,
            "/hil/ground_truth/odom",
            self._odom_cb,
            sensor_qos,
        )
        self.create_subscription(
            Float64, "/hil/actuator/left_effort", self._left_effort_cb, 10
        )
        self.create_subscription(
            Float64, "/hil/actuator/right_effort", self._right_effort_cb, 10
        )
        self.start_client = self.create_client(Trigger, "/hil/test/start_motion")

    def _invalidate(self, reason: str) -> None:
        if self.invalid_reason is None:
            self.invalid_reason = reason

    def _finite(self, values) -> bool:
        return all(math.isfinite(float(value)) for value in values)

    def _target_cb(self, message: Twist) -> None:
        self.received.add("target_twist")
        values = [
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        ]
        if not self._finite(values):
            self._invalidate("non-finite target twist")
            return
        if self.stage != "profile":
            return

        linear = float(message.linear.x)
        yaw = float(message.angular.z)
        epsilon = 1.0e-6
        if abs(linear) > epsilon and abs(yaw) <= epsilon:
            if self.seen_turn:
                phase = "forward_2"
                self.seen_forward_2 = True
            else:
                phase = "forward_1"
        elif abs(yaw) > epsilon and abs(linear) <= epsilon:
            phase = "turn"
            self.seen_turn = True
        elif abs(linear) <= epsilon and abs(yaw) <= epsilon:
            phase = "complete_zero" if self.seen_forward_2 else "initial_zero"
        else:
            self._invalidate("motion profile commanded simultaneous linear and yaw motion")
            return

        if phase != self.current_phase:
            self.current_phase = phase
            self.phase_order.append(phase)
            if phase == "complete_zero":
                self.complete_started_wall = time.monotonic()

    def _wheel_cb(self, message: JointState) -> None:
        self.received.add("wheel_states")
        if not self._finite(message.position) or not self._finite(message.velocity):
            self._invalidate("non-finite wheel state")
            return

        try:
            left_index = message.name.index("left_wheel_joint")
            right_index = message.name.index("right_wheel_joint")
        except ValueError:
            self._invalidate("wheel feedback is missing a configured drive joint")
            return
        if left_index >= len(message.velocity) or right_index >= len(message.velocity):
            self._invalidate("wheel feedback has no velocity for a drive joint")
            return

        left = float(message.velocity[left_index])
        right = float(message.velocity[right_index])
        self.latest_wheels = {"left": left, "right": right}
        self.wheel_names_valid = True
        if self.stage == "profile" and self.current_phase is not None:
            metrics = self.phase_wheels.setdefault(
                self.current_phase,
                {
                    "left_min": math.inf,
                    "left_max": -math.inf,
                    "right_min": math.inf,
                    "right_max": -math.inf,
                },
            )
            metrics["left_min"] = min(metrics["left_min"], left)
            metrics["left_max"] = max(metrics["left_max"], left)
            metrics["right_min"] = min(metrics["right_min"], right)
            metrics["right_max"] = max(metrics["right_max"], right)

    def _imu_cb(self, message: Imu) -> None:
        self.received.add("imu")
        q = message.orientation
        values = [
            q.x,
            q.y,
            q.z,
            q.w,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ]
        if not self._finite(values):
            self._invalidate("non-finite IMU data")
            return

        sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
        cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sin_pitch)
        self.max_abs_roll = max(self.max_abs_roll, abs(roll))
        self.max_abs_pitch = max(self.max_abs_pitch, abs(pitch))

    def _scan_cb(self, message: LaserScan) -> None:
        self.received.add("scan")
        metadata = [
            message.angle_min,
            message.angle_max,
            message.angle_increment,
            message.time_increment,
            message.scan_time,
            message.range_min,
            message.range_max,
        ]
        invalid_metadata = (
            not self._finite(metadata)
            or message.angle_increment <= 0.0
            or message.time_increment < 0.0
            or message.scan_time < 0.0
            or message.range_min < 0.0
            or message.range_max <= message.range_min
        )
        if invalid_metadata:
            self._invalidate("invalid LaserScan metadata")
            return

        tolerance = 1.0e-5
        for value in message.ranges:
            numeric = float(value)
            if math.isnan(numeric) or numeric == -math.inf:
                self._invalidate("LaserScan contains NaN or negative infinity")
                return
            if math.isfinite(numeric) and (
                numeric < message.range_min - tolerance
                or numeric > message.range_max + tolerance
            ):
                self._invalidate("finite LaserScan range is outside sensor limits")
                return

    def _odom_cb(self, message: Odometry) -> None:
        self.received.add("odom")
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        values = [
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            linear.x,
            linear.y,
            linear.z,
            angular.x,
            angular.y,
            angular.z,
        ]
        if not self._finite(values):
            self._invalidate("non-finite odometry")
            return

        pose = (float(position.x), float(position.y), planar_yaw(message))
        self.latest_pose = pose
        self.latest_linear_speed = float(linear.x)
        self.latest_yaw_rate = float(angular.z)

        if self.stage == "stationary" and self.stationary_origin is not None:
            drift = math.hypot(
                pose[0] - self.stationary_origin[0],
                pose[1] - self.stationary_origin[1],
            )
            self.stationary_max_drift = max(self.stationary_max_drift, drift)
            self.stationary_max_speed = max(
                self.stationary_max_speed, abs(self.latest_linear_speed)
            )
            self.stationary_max_yaw_rate = max(
                self.stationary_max_yaw_rate, abs(self.latest_yaw_rate)
            )

        if self.stage == "profile" and self.current_phase is not None:
            metrics = self.phase_pose.setdefault(
                self.current_phase, {"start": pose, "end": pose}
            )
            metrics["end"] = pose

    def _effort_cb(self, side: str, value: float) -> None:
        self.received.add(f"{side}_effort")
        if not math.isfinite(value):
            self._invalidate(f"non-finite {side} effort")
            return
        self.latest_efforts[side] = value
        magnitude = abs(value)
        self.max_abs_effort = max(self.max_abs_effort, magnitude)
        self.saw_nonzero_effort = self.saw_nonzero_effort or magnitude > 1.0e-4
        if magnitude > self.max_effort_nm + 1.0e-6:
            self._invalidate(f"{side} effort exceeded configured limit")
        if self.stage == "stationary":
            self.stationary_max_effort = max(self.stationary_max_effort, magnitude)

    def _left_effort_cb(self, message: Float64) -> None:
        self._effort_cb("left", float(message.data))

    def _right_effort_cb(self, message: Float64) -> None:
        self._effort_cb("right", float(message.data))

    def ready(self) -> bool:
        return (
            not (self.required_topics - self.received)
            and self.wheel_names_valid
            and self.latest_pose is not None
            and self.start_client.service_is_ready()
        )

    def begin_stationary_observation(self) -> None:
        self.stage = "stationary"
        self.stationary_origin = self.latest_pose
        self.stationary_max_drift = 0.0
        self.stationary_max_speed = 0.0
        self.stationary_max_yaw_rate = 0.0
        self.stationary_max_effort = 0.0
        self.max_abs_roll = 0.0
        self.max_abs_pitch = 0.0

    def begin_profile_observation(self) -> None:
        self.stage = "profile"
        self.current_phase = None
        self.phase_order.clear()
        self.phase_pose.clear()
        self.phase_wheels.clear()
        self.seen_turn = False
        self.seen_forward_2 = False
        self.complete_started_wall = None
        self.max_abs_effort = 0.0
        self.saw_nonzero_effort = False


def spin_until(node: Milestone01SmokeTest, deadline: float, predicate) -> bool:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.invalid_reason is not None:
            return False
        if predicate():
            return True
    return False


def main() -> int:
    rclpy.init()
    node = Milestone01SmokeTest()
    failures = []
    metrics = {}

    try:
        readiness_deadline = time.monotonic() + node.readiness_timeout_sec
        if not spin_until(node, readiness_deadline, node.ready):
            missing = sorted(node.required_topics - node.received)
            reason = node.invalid_reason or (
                "interfaces not ready; missing=" + ",".join(missing)
            )
            print("FAIL: " + reason)
            return 1

        node.begin_stationary_observation()
        stationary_deadline = time.monotonic() + node.stationary_observation_sec
        if not spin_until(node, stationary_deadline, lambda: False):
            if node.invalid_reason is not None:
                print("FAIL: " + node.invalid_reason)
                return 1

        metrics["stationary_drift_m"] = node.stationary_max_drift
        metrics["stationary_max_speed_m_s"] = node.stationary_max_speed
        metrics["stationary_max_yaw_rate_rad_s"] = node.stationary_max_yaw_rate
        metrics["stationary_max_effort_nm"] = node.stationary_max_effort
        if node.stationary_max_drift > 0.02:
            failures.append("stationary drift exceeded 0.02 m")
        if node.stationary_max_speed > 0.02:
            failures.append("stationary speed exceeded 0.02 m/s")
        if node.stationary_max_yaw_rate > 0.02:
            failures.append("stationary yaw rate exceeded 0.02 rad/s")
        if node.stationary_max_effort > 0.02:
            failures.append("initial effort exceeded 0.02 N m")
        if node.max_abs_roll > 0.05 or node.max_abs_pitch > 0.05:
            failures.append("stationary roll or pitch exceeded 0.05 rad")
        if failures:
            print("FAIL: " + "; ".join(failures))
            print("METRICS: " + json.dumps(metrics, sort_keys=True))
            return 1

        node.begin_profile_observation()
        future = node.start_client.call_async(Trigger.Request())
        trigger_deadline = time.monotonic() + 5.0
        if not spin_until(node, trigger_deadline, future.done):
            print("FAIL: motion trigger service did not respond")
            return 1
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            print("FAIL: motion trigger rejected request: " + message)
            return 1

        profile_deadline = time.monotonic() + node.profile_timeout_sec
        complete = spin_until(
            node,
            profile_deadline,
            lambda: (
                node.complete_started_wall is not None
                and time.monotonic() - node.complete_started_wall
                >= node.stop_observation_sec
            ),
        )
        if not complete:
            reason = node.invalid_reason or "motion profile or final stop timed out"
            print("FAIL: " + reason)
            return 1

        expected_order = [
            "initial_zero",
            "forward_1",
            "turn",
            "forward_2",
            "complete_zero",
        ]
        metrics["phase_order"] = node.phase_order
        if node.phase_order != expected_order:
            failures.append("unexpected phase order")

        for phase in ("forward_1", "turn", "forward_2"):
            if phase not in node.phase_pose:
                failures.append(f"no odometry captured during {phase}")

        first_forward = math.nan
        second_forward = math.nan
        turn_yaw = math.nan
        if "forward_1" in node.phase_pose:
            phase = node.phase_pose["forward_1"]
            first_forward = projected_displacement(phase["start"], phase["end"])
            if first_forward <= 0.05:
                failures.append("first forward phase did not move forward by 0.05 m")
        if "turn" in node.phase_pose:
            phase = node.phase_pose["turn"]
            turn_yaw = normalize_angle(phase["end"][2] - phase["start"][2])
            if turn_yaw <= 0.03:
                failures.append("positive-yaw phase did not increase yaw by 0.03 rad")
        if "forward_2" in node.phase_pose:
            phase = node.phase_pose["forward_2"]
            second_forward = projected_displacement(phase["start"], phase["end"])
            if second_forward <= 0.05:
                failures.append("second forward phase did not move forward by 0.05 m")

        forward_wheels = node.phase_wheels.get("forward_1", {})
        turn_wheels = node.phase_wheels.get("turn", {})
        if (
            forward_wheels.get("left_max", -math.inf) <= 0.05
            or forward_wheels.get("right_max", -math.inf) <= 0.05
        ):
            failures.append("forward phase did not produce positive speed on both wheels")
        if (
            turn_wheels.get("left_min", math.inf) >= -0.02
            or turn_wheels.get("right_max", -math.inf) <= 0.02
        ):
            failures.append("turn phase wheel signs were not left-negative/right-positive")

        if not node.saw_nonzero_effort:
            failures.append("no non-zero wheel effort observed")
        if max(abs(value) for value in node.latest_efforts.values()) > 0.02:
            failures.append("final wheel effort did not return below 0.02 N m")
        if max(abs(value) for value in node.latest_wheels.values()) > 0.02:
            failures.append("final wheel speed did not return below 0.02 rad/s")
        if abs(node.latest_linear_speed) > 0.02 or abs(node.latest_yaw_rate) > 0.02:
            failures.append("final body velocity did not return below stop tolerance")
        if node.max_abs_roll > 0.05 or node.max_abs_pitch > 0.05:
            failures.append("roll or pitch exceeded 0.05 rad during motion")

        metrics.update(
            {
                "first_forward_m": first_forward,
                "turn_yaw_rad": turn_yaw,
                "second_forward_m": second_forward,
                "max_abs_effort_nm": node.max_abs_effort,
                "final_left_effort_nm": node.latest_efforts["left"],
                "final_right_effort_nm": node.latest_efforts["right"],
                "final_left_wheel_rad_s": node.latest_wheels["left"],
                "final_right_wheel_rad_s": node.latest_wheels["right"],
                "final_linear_speed_m_s": node.latest_linear_speed,
                "final_yaw_rate_rad_s": node.latest_yaw_rate,
                "max_abs_roll_rad": node.max_abs_roll,
                "max_abs_pitch_rad": node.max_abs_pitch,
            }
        )

        if failures:
            print("FAIL: " + "; ".join(failures))
            print("METRICS: " + json.dumps(metrics, sort_keys=True))
            return 1

        print("PASS: triggered Milestone 1 sequence and all acceptance checks")
        print("METRICS: " + json.dumps(metrics, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
