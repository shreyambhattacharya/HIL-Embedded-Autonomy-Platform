#!/usr/bin/env python3
"""Quantitative, ground-truth-observing evaluator for one Milestone 5A run.

Ground truth is subscribed to only for evaluation. The estimator, waypoint follower,
and collision filter do not subscribe to it.
"""

import json
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def distance_xy(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])

def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Milestone05AEvaluator(Node):
    def __init__(self):
        super().__init__("milestone_05a_evaluator")
        self.scenario = str(self.declare_parameter("scenario", "open_square").value)
        self.backend = str(self.declare_parameter("backend", "software").value)
        self.output_path = str(
            self.declare_parameter("output_path", "/tmp/milestone_05a_result.json").value
        )
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 80.0).value)
        self.start_delay_sec = float(self.declare_parameter("start_delay_sec", 2.0).value)
        self.arm_on_start = bool(self.declare_parameter("arm_on_start", False).value)
        if self.scenario not in {"open_square", "obstacle_stop", "stale_lidar"}:
            raise ValueError(f"unsupported scenario: {self.scenario}")

        self.estimate = None
        self.truth = None
        self.raw = Twist()
        self.target = Twist()
        self.autonomy_state = "IDLE"
        self.collision_state = "INVALID"
        self.minimum_distance = math.inf
        self.have_estimate = False
        self.have_truth = False
        self.have_valid_estimate = False
        self.run_start_time = None
        self.initial_estimate = None
        self.initial_truth = None
        self.first_stop_time = None
        self.first_stop_truth = None
        self.complete_time = None
        self.service_wait_ticks = 0
        self.start_future = None
        self.arm_future = None
        self.started = False
        self.finished = False
        self.status = "NOT_RUN"
        self.reason = "evaluation has not started"
        self.max_estimate_truth_error = 0.0
        self.max_raw_linear = 0.0
        self.max_target_linear = 0.0
        self.truth_samples = 0
        self.position_error_sum_sq = 0.0
        self.yaw_error_sum_sq = 0.0
        self.max_estimate_truth_yaw_error = 0.0
        self.last_estimate_truth_position_error = None
        self.last_estimate_truth_yaw_error = None

        self.create_subscription(Odometry, "/hil/estimate/odom", self.estimate_callback, 10)
        self.create_subscription(
            Odometry, "/hil/ground_truth/odom", self.truth_callback, qos_profile_sensor_data
        )
        self.create_subscription(String, "/hil/autonomy/state", self.autonomy_callback, 10)
        self.create_subscription(String, "/hil/autonomy/collision_state", self.collision_callback, 10)
        self.create_subscription(Twist, "/hil/autonomy/raw_twist", self.raw_callback, 10)
        self.create_subscription(Twist, "/hil/control/target_twist", self.target_callback, 10)
        self.create_subscription(Float64, "/hil/autonomy/min_front_distance_m", self.distance_callback, 10)
        self.arm_client = self.create_client(Trigger, "/hil/stm32/arm")
        self.start_client = self.create_client(Trigger, "/hil/autonomy/start")
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"evaluating scenario={self.scenario} backend={self.backend} output={self.output_path}"
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def estimate_callback(self, message):
        self.estimate = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw_from_quaternion(message.pose.pose.orientation),
        )
        self.have_estimate = True
        self.have_valid_estimate = message.pose.covariance[0] < 1.0e5

    def truth_callback(self, message):
        self.truth = (message.pose.pose.position.x, message.pose.pose.position.y, yaw_from_quaternion(message.pose.pose.orientation))
        self.have_truth = True

    def autonomy_callback(self, message):
        self.autonomy_state = message.data

    def collision_callback(self, message):
        self.collision_state = message.data

    def distance_callback(self, message):
        self.minimum_distance = float(message.data)

    def raw_callback(self, message):
        self.raw = message

    def target_callback(self, message):
        self.target = message

    @staticmethod
    def zero_twist(message, tolerance=0.01):
        return abs(message.linear.x) <= tolerance and abs(message.angular.z) <= tolerance

    def request_service(self):
        if self.backend == "stm32" and self.arm_on_start:
            if self.arm_future is None:
                if not self.arm_client.service_is_ready():
                    return
                self.arm_future = self.arm_client.call_async(Trigger.Request())
                return
            if not self.arm_future.done():
                return
            try:
                response = self.arm_future.result()
            except Exception as exception:  # pragma: no cover - depends on DDS/service failure
                self.finish("NOT_RUN", f"STM32 ARM service failed: {exception}")
                return
            if not response.success:
                self.finish("NOT_RUN", f"STM32 ARM rejected: {response.message}")
                return
            self.arm_on_start = False

        if self.start_future is None:
            if not self.start_client.service_is_ready():
                return
            self.start_future = self.start_client.call_async(Trigger.Request())
            return
        if not self.start_future.done():
            return
        try:
            response = self.start_future.result()
        except Exception as exception:  # pragma: no cover - depends on DDS/service failure
            self.finish("FAIL", f"autonomy start service failed: {exception}")
            return
        if not response.success:
            self.finish("FAIL", f"autonomy start rejected: {response.message}")
            return
        self.started = True
        self.run_start_time = self.now_sec()
        self.initial_estimate = self.estimate
        self.initial_truth = self.truth
        self.get_logger().info("autonomy run started")

    def update_metrics(self):
        self.max_raw_linear = max(self.max_raw_linear, abs(self.raw.linear.x))
        self.max_target_linear = max(self.max_target_linear, abs(self.target.linear.x))
        if self.have_estimate and self.have_truth and self.initial_estimate and self.initial_truth:
            estimate_delta = (
                self.estimate[0] - self.initial_estimate[0],
                self.estimate[1] - self.initial_estimate[1],
            )
            truth_delta = (
                self.truth[0] - self.initial_truth[0],
                self.truth[1] - self.initial_truth[1],
            )
            position_error = distance_xy(estimate_delta, truth_delta)
            yaw_error = wrap_angle(
                wrap_angle(self.estimate[2] - self.initial_estimate[2]) -
                wrap_angle(self.truth[2] - self.initial_truth[2]))
            self.max_estimate_truth_error = max(self.max_estimate_truth_error, position_error)
            self.position_error_sum_sq += position_error * position_error
            self.max_estimate_truth_yaw_error = max(self.max_estimate_truth_yaw_error, abs(yaw_error))
            self.yaw_error_sum_sq += yaw_error * yaw_error
            self.last_estimate_truth_position_error = position_error
            self.last_estimate_truth_yaw_error = abs(yaw_error)
            self.truth_samples += 1
        if self.collision_state == "STOP" and self.first_stop_time is None:
            self.first_stop_time = self.now_sec()
            self.first_stop_truth = self.truth
            self.get_logger().info("collision STOP observed")

    def tick(self):
        if self.finished:
            return
        current_time = self.now_sec()
        if current_time <= 0.0:
            return
        if not self.started:
            if not self.have_valid_estimate or not self.have_truth:
                return
            if self.run_start_time is None:
                self.run_start_time = current_time
            if current_time - self.run_start_time < self.start_delay_sec:
                return
            self.request_service()
            self.service_wait_ticks += 1
            if self.service_wait_ticks > 200 and self.start_future is None:
                result = "NOT_RUN" if self.backend == "stm32" else "FAIL"
                self.finish(result, "required start service did not become available")
            return

        self.update_metrics()
        elapsed = current_time - self.run_start_time
        if self.scenario == "open_square":
            if self.autonomy_state == "COMPLETE":
                if self.complete_time is None:
                    self.complete_time = current_time
                if current_time - self.complete_time >= 0.5 and self.zero_twist(self.target):
                    self.finish("PASS", "waypoint follower completed the open square")
                    return
        elif self.scenario == "obstacle_stop":
            if self.first_stop_time is not None and current_time - self.first_stop_time >= 1.0:
                if self.zero_twist(self.target):
                    self.finish("PASS", "LiDAR collision filter held zero target twist at obstacle")
                    return
        elif self.scenario == "stale_lidar":
            if self.collision_state == "STALE":
                if self.first_stop_time is None:
                    self.first_stop_time = current_time
                if current_time - self.first_stop_time >= 0.8 and self.zero_twist(self.target):
                    self.finish("PASS", "stale LiDAR caused a zero target twist")
                    return
        if elapsed >= self.timeout_sec:
            self.finish("FAIL", f"scenario timeout after {elapsed:.2f}s")

    def finish(self, status, reason):
        if self.finished:
            return
        self.finished = True
        self.status = status
        self.reason = reason
        final_return_error = None
        truth_progress_after_stop = None
        if self.truth and self.initial_truth:
            final_return_error = distance_xy(self.truth, self.initial_truth)
        if self.truth and self.first_stop_truth:
            truth_progress_after_stop = distance_xy(self.truth, self.first_stop_truth)
        completion_time = None
        if self.complete_time is not None and self.run_start_time is not None:
            completion_time = self.complete_time - self.run_start_time
        rms_position_error = None
        rms_yaw_error = None
        if self.truth_samples > 0:
            rms_position_error = math.sqrt(self.position_error_sum_sq / self.truth_samples)
            rms_yaw_error = math.sqrt(self.yaw_error_sum_sq / self.truth_samples)
        result = {
            "schema": "hil.autonomy.milestone5a.scenario.v1",
            "status": self.status,
            "reason": self.reason,
            "scenario": self.scenario,
            "backend": self.backend,
            "uses_ground_truth_for_control": False,
            "metrics": {
                "truth_samples": self.truth_samples,
                "max_estimate_truth_position_error_m": self.max_estimate_truth_error,
                "rms_estimate_truth_position_error_m": rms_position_error,
                "final_estimate_truth_position_error_m": self.last_estimate_truth_position_error,
                "max_estimate_truth_yaw_error_rad": self.max_estimate_truth_yaw_error,
                "rms_estimate_truth_yaw_error_rad": rms_yaw_error,
                "final_estimate_truth_yaw_error_rad": self.last_estimate_truth_yaw_error,
                "max_raw_linear_speed_m_s": self.max_raw_linear,
                "max_target_linear_speed_m_s": self.max_target_linear,
                "minimum_front_distance_m": None if math.isinf(self.minimum_distance) else self.minimum_distance,
                "final_return_error_m": final_return_error,
                "truth_progress_after_collision_stop_m": truth_progress_after_stop,
                "completion_time_sec": completion_time,
                "collision_state_at_finish": self.collision_state,
                "autonomy_state_at_finish": self.autonomy_state,
                "target_twist_zero_at_finish": self.zero_twist(self.target),
            },
        }
        output_directory = os.path.dirname(os.path.abspath(self.output_path))
        os.makedirs(output_directory, exist_ok=True)
        with open(self.output_path, "w", encoding="utf-8") as output_file:
            json.dump(result, output_file, indent=2, sort_keys=True)
            output_file.write("\n")
        self.get_logger().info(f"MILESTONE 5A {self.status}: {self.reason}")
        self.get_logger().info(json.dumps(result, sort_keys=True))
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    try:
        node = Milestone05AEvaluator()
        rclpy.spin(node)
        status = node.status
    except Exception as exception:  # pragma: no cover - launch failure path
        print(f"milestone_05a_evaluator failed: {exception}")
        return 1
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
