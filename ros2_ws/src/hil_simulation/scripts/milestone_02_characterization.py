#!/usr/bin/env python3
"""Measure the Milestone 2 software boundary under repeatable command excitation."""

from datetime import UTC, datetime
import json
import math
from pathlib import Path
import socket
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


class TopicStats:
    def __init__(self, expected_rate_hz=None):
        self.expected_rate_hz = expected_rate_hz
        self.times = []

    def record(self, timestamp):
        if self.times and timestamp < self.times[-1]:
            return
        self.times.append(timestamp)

    def reset(self):
        self.times.clear()

    def summarize(self):
        intervals = [
            later - earlier
            for earlier, later in zip(self.times, self.times[1:])
            if later > earlier
        ]
        duration = self.times[-1] - self.times[0] if len(self.times) > 1 else 0.0
        rate = (len(self.times) - 1) / duration if duration > 0.0 else 0.0
        median_period = percentile(intervals, 0.5)
        reference_period = (
            1.0 / self.expected_rate_hz
            if self.expected_rate_hz is not None
            else median_period
        )
        jitter = (
            [abs(interval - reference_period) for interval in intervals]
            if reference_period is not None
            else []
        )
        return {
            "samples": len(self.times),
            "rate_hz": rate,
            "median_period_ms": (
                median_period * 1000.0 if median_period is not None else None
            ),
            "p95_abs_jitter_ms": (
                percentile(jitter, 0.95) * 1000.0 if jitter else None
            ),
        }


class CharacterizationNode(Node):
    def __init__(self):
        super().__init__("milestone_02_characterization")
        self.result_path = str(self.declare_parameter("result_path", "").value)
        self.run_id = str(self.declare_parameter("run_id", "single").value)
        self.test_mode = str(self.declare_parameter("test_mode", "normal").value)
        self.stats = {
            "target_twist": TopicStats(20.0),
            "wheel_states": TopicStats(),
            "imu": TopicStats(100.0),
            "scan": TopicStats(10.0),
            "odometry": TopicStats(50.0),
            "left_effort": TopicStats(100.0),
            "right_effort": TopicStats(100.0),
        }
        self.control_period_ms = []
        self.command_age_ms = []
        self.feedback_age_ms = []
        self.control_execution_ms = []
        self.invalid_diagnostic = False
        self.triggered = False
        self.saw_active_command = False
        self.previous_command_active = False
        self.command_transition_time = None
        self.effort_response_time = None
        self.wheel_response_time = None
        self.completion_time = None
        self.latest_wheel_speed = math.inf
        self.stationary_max_wheel_speed = 0.0
        self.stationary_max_effort = 0.0
        self.invalid_wheel_feedback = False

        sensor_qos = rclpy.qos.qos_profile_sensor_data
        self._topic_subscriptions = [
            self.create_subscription(
                Twist, "/hil/control/target_twist", self.target_callback, 10
            ),
            self.create_subscription(
                JointState,
                "/hil/sensors/wheel_states",
                self.wheel_callback,
                sensor_qos,
            ),
            self.create_subscription(Imu, "/hil/sensors/imu", self.imu_callback, sensor_qos),
            self.create_subscription(
                LaserScan, "/hil/sensors/scan", self.scan_callback, sensor_qos
            ),
            self.create_subscription(
                Odometry,
                "/hil/ground_truth/odom",
                self.odom_callback,
                sensor_qos,
            ),
            self.create_subscription(
                Float64, "/hil/actuator/left_effort", self.left_effort_callback, 10
            ),
            self.create_subscription(
                Float64,
                "/hil/actuator/right_effort",
                self.right_effort_callback,
                10,
            ),
            self.create_subscription(
                Float64,
                "/hil/diagnostics/control_period_ms",
                lambda message: self.record_diagnostic(
                    self.control_period_ms, message.data
                ),
                10,
            ),
            self.create_subscription(
                Float64,
                "/hil/diagnostics/command_age_ms",
                lambda message: self.record_diagnostic(self.command_age_ms, message.data),
                10,
            ),
            self.create_subscription(
                Float64,
                "/hil/diagnostics/feedback_age_ms",
                lambda message: self.record_diagnostic(self.feedback_age_ms, message.data),
                10,
            ),
            self.create_subscription(
                Float64,
                "/hil/diagnostics/control_execution_ms",
                lambda message: self.record_diagnostic(
                    self.control_execution_ms, message.data
                ),
                10,
            ),
        ]
        self.start_client = self.create_client(Trigger, "/hil/test/start_motion")

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1.0e9

    def record(self, topic):
        self.stats[topic].record(self.now_seconds())

    def record_diagnostic(self, destination, value):
        if math.isfinite(value) and value >= 0.0:
            destination.append(float(value))
        else:
            self.invalid_diagnostic = True

    def target_callback(self, message):
        self.record("target_twist")
        active = abs(message.linear.x) > 1.0e-6 or abs(message.angular.z) > 1.0e-6
        now = self.now_seconds()
        if self.triggered and active and not self.saw_active_command:
            self.saw_active_command = True
            self.command_transition_time = now
        if self.triggered and self.saw_active_command and self.previous_command_active and not active:
            self.completion_time = now
        self.previous_command_active = active

    def wheel_callback(self, message):
        self.record("wheel_states")
        try:
            left_index = message.name.index("left_wheel_joint")
            right_index = message.name.index("right_wheel_joint")
            left = float(message.velocity[left_index])
            right = float(message.velocity[right_index])
            if not math.isfinite(left) or not math.isfinite(right):
                raise ValueError("non-finite velocity")
        except (ValueError, IndexError):
            self.invalid_wheel_feedback = True
            return
        self.latest_wheel_speed = max(abs(left), abs(right))
        if not self.triggered:
            self.stationary_max_wheel_speed = max(
                self.stationary_max_wheel_speed, self.latest_wheel_speed
            )
        if (
            self.command_transition_time is not None
            and self.wheel_response_time is None
            and self.latest_wheel_speed > 0.05
        ):
            self.wheel_response_time = self.now_seconds()

    def imu_callback(self, _message):
        self.record("imu")

    def scan_callback(self, _message):
        self.record("scan")

    def odom_callback(self, _message):
        self.record("odometry")

    def effort_callback(self, topic, value):
        self.record(topic)
        if not math.isfinite(value):
            self.invalid_diagnostic = True
            return
        if not self.triggered:
            self.stationary_max_effort = max(self.stationary_max_effort, abs(value))
        if (
            self.command_transition_time is not None
            and self.effort_response_time is None
            and abs(value) > 0.05
        ):
            self.effort_response_time = self.now_seconds()

    def left_effort_callback(self, message):
        self.effort_callback("left_effort", float(message.data))

    def right_effort_callback(self, message):
        self.effort_callback("right_effort", float(message.data))

    def ready(self):
        minimum_samples = {
            "target_twist": 5,
            "wheel_states": 5,
            "imu": 5,
            "scan": 2,
            "odometry": 5,
            "left_effort": 5,
            "right_effort": 5,
        }
        return (
            self.now_seconds() > 0.0
            and all(
                len(self.stats[name].times) >= count
                for name, count in minimum_samples.items()
            )
            and len(self.control_period_ms) >= 5
            and len(self.command_age_ms) >= 5
            and len(self.feedback_age_ms) >= 5
            and len(self.control_execution_ms) >= 5
            and self.start_client.service_is_ready()
        )

    def reset_measurement(self):
        for statistic in self.stats.values():
            statistic.reset()
        self.control_period_ms.clear()
        self.command_age_ms.clear()
        self.feedback_age_ms.clear()
        self.control_execution_ms.clear()
        self.triggered = True


def spin_until(node, predicate, wall_timeout_sec):
    deadline = time.monotonic() + wall_timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def numeric_summary(values, target=None):
    result = {
        "samples": len(values),
        "median_ms": percentile(values, 0.5),
        "p95_ms": percentile(values, 0.95),
        "max_ms": max(values) if values else None,
    }
    if values and target is not None:
        result["p95_abs_jitter_ms"] = percentile(
            [abs(value - target) for value in values], 0.95
        )
    return result


def build_result_document(node, metrics, result, failures, failure_category=None):
    return {
        "schema_version": 1,
        "run_metadata": {
            "run_id": node.run_id,
            "test_mode": node.test_mode,
            "timestamp_utc": datetime.now(UTC).isoformat(),
            "hostname": socket.gethostname(),
            "clock_domains": {
                "topic_and_response": "gazebo_simulation_time",
                "controller_timing": "steady_wall_clock",
            },
        },
        "topics": metrics.get("topics", {}),
        "controller_timing": metrics.get("controller_timing", {}),
        "controller_budget": metrics.get("controller_budget", {}),
        "response_latency_ms": metrics.get("response_latency_ms", {}),
        "stationary": metrics.get("stationary", {}),
        "result": result,
        "failure_category": failure_category,
        "failures": failures,
    }


def emit_result(node, document):
    serialized = json.dumps(document, indent=2, sort_keys=True)
    print(serialized)
    if not node.result_path:
        return
    output_path = Path(node.result_path).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary_path.write_text(serialized + "\n", encoding="utf-8")
    temporary_path.replace(output_path)


def main():
    rclpy.init()
    node = CharacterizationNode()
    failures = []
    metrics = {}
    document = None
    try:
        if not spin_until(node, node.ready, 60.0):
            readiness_snapshot = {
                "topics": {
                    name: len(statistic.times)
                    for name, statistic in node.stats.items()
                },
                "control_period": len(node.control_period_ms),
                "command_age": len(node.command_age_ms),
                "feedback_age": len(node.feedback_age_ms),
                "control_execution": len(node.control_execution_ms),
                "trigger_service": node.start_client.service_is_ready(),
            }
            raise RuntimeError(f"interfaces not ready: {json.dumps(readiness_snapshot, sort_keys=True)}")

        stationary_start = node.now_seconds()
        if not spin_until(
            node, lambda: node.now_seconds() - stationary_start >= 2.0, 10.0
        ):
            raise RuntimeError("simulation clock did not advance through stationary window")

        node.reset_measurement()
        future = node.start_client.call_async(Trigger.Request())
        if not spin_until(node, future.done, 5.0):
            raise RuntimeError("motion Trigger service timed out")
        response = future.result()
        if response is None or not response.success:
            detail = "no response" if response is None else response.message
            raise RuntimeError(f"motion Trigger service failed: {detail}")

        if not spin_until(
            node,
            lambda: node.completion_time is not None
            and node.now_seconds() - node.completion_time >= 2.0,
            45.0,
        ):
            raise RuntimeError("motion profile did not complete with a two-second stop window")

        metrics["topics"] = {
            name: statistic.summarize() for name, statistic in node.stats.items()
        }
        execution_summary = numeric_summary(node.control_execution_ms)
        metrics["controller_timing"] = {
            "control_period": numeric_summary(node.control_period_ms, 10.0),
            "command_age": numeric_summary(node.command_age_ms),
            "feedback_age": numeric_summary(node.feedback_age_ms),
            "control_execution": execution_summary,
        }
        metrics["controller_budget"] = {
            "nominal_period_ms": 10.0,
            "median_execution_utilization_percent": (
                execution_summary["median_ms"] / 10.0 * 100.0
                if execution_summary["median_ms"] is not None else None
            ),
            "p95_execution_utilization_percent": (
                execution_summary["p95_ms"] / 10.0 * 100.0
                if execution_summary["p95_ms"] is not None else None
            ),
            "minimum_observed_margin_ms": (
                10.0 - execution_summary["max_ms"]
                if execution_summary["max_ms"] is not None else None
            ),
        }
        metrics["response_latency_ms"] = {
            "target_to_effort": (
                (node.effort_response_time - node.command_transition_time) * 1000.0
                if node.effort_response_time is not None
                and node.command_transition_time is not None
                else None
            ),
            "target_to_wheel_motion": (
                (node.wheel_response_time - node.command_transition_time) * 1000.0
                if node.wheel_response_time is not None
                and node.command_transition_time is not None
                else None
            ),
        }
        metrics["stationary"] = {
            "max_abs_wheel_speed_rad_s": node.stationary_max_wheel_speed,
            "max_abs_effort_nm": node.stationary_max_effort,
        }

        minimum_rates = {
            "target_twist": 15.0,
            "wheel_states": 50.0,
            "imu": 80.0,
            "scan": 8.0,
            "odometry": 40.0,
            "left_effort": 80.0,
            "right_effort": 80.0,
        }
        for name, minimum_rate in minimum_rates.items():
            if metrics["topics"][name]["rate_hz"] < minimum_rate:
                failures.append(
                    f"{name} rate below {minimum_rate:.1f} Hz minimum"
                )
        control_period = metrics["controller_timing"]["control_period"]
        if control_period["samples"] < 100:
            failures.append("insufficient controller timing samples")
        if control_period["p95_ms"] is None or control_period["p95_ms"] > 20.0:
            failures.append("controller period p95 exceeded 20 ms")
        if execution_summary["samples"] < 100:
            failures.append("insufficient controller execution-time samples")
        if execution_summary["p95_ms"] is None or execution_summary["p95_ms"] > 10.0:
            failures.append("controller execution-time p95 exceeded the nominal 10 ms period")
        if node.command_age_ms and percentile(node.command_age_ms, 0.95) > 100.0:
            failures.append("command age p95 exceeded 100 ms")
        if node.feedback_age_ms and percentile(node.feedback_age_ms, 0.95) > 100.0:
            failures.append("feedback age p95 exceeded 100 ms")

        target_to_effort = metrics["response_latency_ms"]["target_to_effort"]
        target_to_wheel = metrics["response_latency_ms"]["target_to_wheel_motion"]
        if target_to_effort is None or target_to_effort < 0.0 or target_to_effort > 100.0:
            failures.append("target-to-effort latency was absent or exceeded 100 ms")
        if target_to_wheel is None or target_to_wheel < 0.0 or target_to_wheel > 500.0:
            failures.append("target-to-wheel latency was absent or exceeded 500 ms")
        if node.stationary_max_wheel_speed > 0.02:
            failures.append("stationary wheel speed exceeded 0.02 rad/s")
        if node.stationary_max_effort > 0.02:
            failures.append("stationary effort exceeded 0.02 N m")
        if node.invalid_diagnostic:
            failures.append("a non-finite timing or effort sample was observed")
        if node.invalid_wheel_feedback:
            failures.append("invalid wheel feedback was observed")

        if failures:
            document = build_result_document(
                node, metrics, "FAIL", failures, "regression"
            )
            emit_result(node, document)
            for failure in failures:
                node.get_logger().error(failure)
            print("MILESTONE 2 CHARACTERIZATION: FAIL")
            return 1
        document = build_result_document(node, metrics, "PASS", [])
        emit_result(node, document)
        print("MILESTONE 2 CHARACTERIZATION: PASS")
        return 0
    except Exception as exception:
        failure = str(exception)
        node.get_logger().error(failure)
        document = build_result_document(
            node, metrics, "INVALID", [failure], "infrastructure"
        )
        emit_result(node, document)
        print("MILESTONE 2 CHARACTERIZATION: FAIL")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
