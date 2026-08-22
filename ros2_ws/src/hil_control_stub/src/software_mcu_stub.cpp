#include "hil_control_stub/control_math.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace
{

using std::placeholders::_1;

class SoftwareMcuStub final : public rclcpp::Node
{
public:
  SoftwareMcuStub()
  : Node("software_mcu_stub")
  {
    wheel_radius_m_ = declare_parameter("wheel_radius_m", 0.18);
    wheel_separation_m_ = declare_parameter("wheel_separation_m", 0.86);
    wheel_speed_kp_nm_per_rad_s_ = declare_parameter("wheel_speed_kp_nm_per_rad_s", 0.75);
    max_wheel_effort_nm_ = declare_parameter("max_wheel_effort_nm", 1.5);
    control_rate_hz_ = declare_parameter("control_rate_hz", 100.0);
    command_timeout_sec_ = declare_parameter("command_timeout_sec", 0.5);
    feedback_timeout_sec_ = declare_parameter("feedback_timeout_sec", 0.2);
    publish_timing_diagnostics_ = declare_parameter("publish_timing_diagnostics", false);
    left_joint_name_ = declare_parameter("left_joint_name", std::string("left_wheel_joint"));
    right_joint_name_ = declare_parameter("right_joint_name", std::string("right_wheel_joint"));

    if (!hil_control_stub::finite(wheel_radius_m_) || wheel_radius_m_ <= 0.0 ||
      !hil_control_stub::finite(wheel_separation_m_) || wheel_separation_m_ < 0.0 ||
      !hil_control_stub::finite(wheel_speed_kp_nm_per_rad_s_) || wheel_speed_kp_nm_per_rad_s_ < 0.0 ||
      !hil_control_stub::finite(max_wheel_effort_nm_) || max_wheel_effort_nm_ <= 0.0 ||
      !hil_control_stub::finite(control_rate_hz_) || control_rate_hz_ <= 0.0 ||
      !hil_control_stub::finite(command_timeout_sec_) || command_timeout_sec_ <= 0.0 ||
      !hil_control_stub::finite(feedback_timeout_sec_) || feedback_timeout_sec_ <= 0.0 ||
      left_joint_name_.empty() || right_joint_name_.empty())
    {
      throw std::invalid_argument("invalid software_mcu_stub parameter");
    }

    target_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/hil/control/target_twist", rclcpp::QoS(10),
      std::bind(&SoftwareMcuStub::target_callback, this, _1));
    wheel_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/hil/sensors/wheel_states", rclcpp::SensorDataQoS(),
      std::bind(&SoftwareMcuStub::wheel_callback, this, _1));
    left_effort_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/hil/actuator/left_effort", rclcpp::QoS(10));
    right_effort_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/hil/actuator/right_effort", rclcpp::QoS(10));

    if (publish_timing_diagnostics_) {
      control_period_publisher_ = create_publisher<std_msgs::msg::Float64>(
        "/hil/diagnostics/control_period_ms", rclcpp::QoS(10));
      command_age_publisher_ = create_publisher<std_msgs::msg::Float64>(
        "/hil/diagnostics/command_age_ms", rclcpp::QoS(10));
      feedback_age_publisher_ = create_publisher<std_msgs::msg::Float64>(
        "/hil/diagnostics/feedback_age_ms", rclcpp::QoS(10));
    }

    const auto period_ms = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::llround(1000.0 / control_rate_hz_)));
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms), std::bind(&SoftwareMcuStub::control_step, this));

    RCLCPP_INFO(
      get_logger(),
      "temporary software MCU stub: radius=%.3f m separation=%.3f m kp=%.3f Nm/(rad/s) "
      "effort_limit=%.3f Nm rate=%.1f Hz",
      wheel_radius_m_, wheel_separation_m_, wheel_speed_kp_nm_per_rad_s_,
      max_wheel_effort_nm_, control_rate_hz_);
  }

private:
  void target_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!hil_control_stub::finite(message->linear.x) || !hil_control_stub::finite(message->angular.z)) {
      have_command_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "invalid target twist; commanding zero effort");
      return;
    }
    target_linear_m_s_ = message->linear.x;
    target_yaw_rad_s_ = message->angular.z;
    last_command_time_ = now();
    last_command_steady_time_ = std::chrono::steady_clock::now();
    have_command_ = true;
  }

  void wheel_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const auto left_it = std::find(message->name.begin(), message->name.end(), left_joint_name_);
    const auto right_it = std::find(message->name.begin(), message->name.end(), right_joint_name_);
    if (left_it == message->name.end() || right_it == message->name.end()) {
      have_feedback_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "wheel feedback does not contain both configured joints");
      return;
    }

    const auto left_index = static_cast<std::size_t>(std::distance(message->name.begin(), left_it));
    const auto right_index = static_cast<std::size_t>(std::distance(message->name.begin(), right_it));
    if (left_index >= message->velocity.size() || right_index >= message->velocity.size() ||
      !hil_control_stub::finite(message->velocity[left_index]) ||
      !hil_control_stub::finite(message->velocity[right_index]))
    {
      have_feedback_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "invalid wheel feedback; commanding zero effort");
      return;
    }

    left_measured_rad_s_ = message->velocity[left_index];
    right_measured_rad_s_ = message->velocity[right_index];
    last_feedback_time_ = now();
    last_feedback_steady_time_ = std::chrono::steady_clock::now();
    have_feedback_ = true;
  }

  bool fresh(const rclcpp::Time & current_time, const rclcpp::Time & sample_time, double timeout) const
  {
    return (current_time - sample_time).seconds() <= timeout;
  }

  void publish_efforts(const double left_effort_nm, const double right_effort_nm)
  {
    std_msgs::msg::Float64 left_message;
    std_msgs::msg::Float64 right_message;
    left_message.data = left_effort_nm;
    right_message.data = right_effort_nm;
    left_effort_publisher_->publish(left_message);
    right_effort_publisher_->publish(right_message);
  }

  void publish_timing(const std::chrono::steady_clock::time_point current_time)
  {
    if (!publish_timing_diagnostics_) {
      return;
    }

    if (have_last_control_step_) {
      std_msgs::msg::Float64 period_message;
      period_message.data = std::chrono::duration<double, std::milli>(
        current_time - last_control_steady_time_).count();
      control_period_publisher_->publish(period_message);
    }
    last_control_steady_time_ = current_time;
    have_last_control_step_ = true;

    if (have_command_) {
      std_msgs::msg::Float64 age_message;
      age_message.data = std::chrono::duration<double, std::milli>(
        current_time - last_command_steady_time_).count();
      command_age_publisher_->publish(age_message);
    }
    if (have_feedback_) {
      std_msgs::msg::Float64 age_message;
      age_message.data = std::chrono::duration<double, std::milli>(
        current_time - last_feedback_steady_time_).count();
      feedback_age_publisher_->publish(age_message);
    }
  }

  void control_step()
  {
    publish_timing(std::chrono::steady_clock::now());
    const auto current_time = now();
    const bool input_fresh = have_command_ && have_feedback_ &&
      fresh(current_time, last_command_time_, command_timeout_sec_) &&
      fresh(current_time, last_feedback_time_, feedback_timeout_sec_);

    if (!input_fresh) {
      publish_efforts(0.0, 0.0);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "command or wheel feedback is stale; temporary safety behavior is zero effort");
      return;
    }

    const auto targets = hil_control_stub::body_twist_to_wheel_targets(
      target_linear_m_s_, target_yaw_rad_s_, wheel_radius_m_, wheel_separation_m_);
    const auto left_effort = hil_control_stub::limited_p_effort(
      targets.left_rad_s, left_measured_rad_s_, wheel_speed_kp_nm_per_rad_s_, max_wheel_effort_nm_);
    const auto right_effort = hil_control_stub::limited_p_effort(
      targets.right_rad_s, right_measured_rad_s_, wheel_speed_kp_nm_per_rad_s_, max_wheel_effort_nm_);
    publish_efforts(left_effort, right_effort);
  }

  double wheel_radius_m_{0.0};
  double wheel_separation_m_{0.0};
  double wheel_speed_kp_nm_per_rad_s_{0.0};
  double max_wheel_effort_nm_{0.0};
  double control_rate_hz_{0.0};
  double command_timeout_sec_{0.0};
  double feedback_timeout_sec_{0.0};
  bool publish_timing_diagnostics_{false};
  std::string left_joint_name_;
  std::string right_joint_name_;

  double target_linear_m_s_{0.0};
  double target_yaw_rad_s_{0.0};
  double left_measured_rad_s_{0.0};
  double right_measured_rad_s_{0.0};
  bool have_command_{false};
  bool have_feedback_{false};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_command_steady_time_;
  std::chrono::steady_clock::time_point last_feedback_steady_time_;
  std::chrono::steady_clock::time_point last_control_steady_time_;
  bool have_last_control_step_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_effort_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_effort_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_period_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_age_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_age_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SoftwareMcuStub>());
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "software_mcu_stub failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
