#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{

class MotionTestNode final : public rclcpp::Node
{
public:
  MotionTestNode()
  : Node("motion_test_node")
  {
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
    initial_delay_sec_ = declare_parameter("initial_delay_sec", 2.0);
    first_forward_duration_sec_ = declare_parameter("first_forward_duration_sec", 5.0);
    turn_duration_sec_ = declare_parameter("turn_duration_sec", 3.0);
    second_forward_duration_sec_ = declare_parameter("second_forward_duration_sec", 5.0);
    forward_speed_m_s_ = declare_parameter("forward_speed_m_s", 0.25);
    turn_rate_rad_s_ = declare_parameter("turn_rate_rad_s", 0.5);

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
      !std::isfinite(initial_delay_sec_) || initial_delay_sec_ < 0.0 ||
      !std::isfinite(first_forward_duration_sec_) || first_forward_duration_sec_ <= 0.0 ||
      !std::isfinite(turn_duration_sec_) || turn_duration_sec_ <= 0.0 ||
      !std::isfinite(second_forward_duration_sec_) || second_forward_duration_sec_ <= 0.0 ||
      !std::isfinite(forward_speed_m_s_) || !std::isfinite(turn_rate_rad_s_))
    {
      throw std::invalid_argument("invalid motion_test_node parameter");
    }

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "/hil/control/target_twist", rclcpp::QoS(10));
    const auto period_ms = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::llround(1000.0 / publish_rate_hz_)));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this]() { publish_step(); });

    RCLCPP_INFO(
      get_logger(),
      "deterministic profile: zero %.1fs, forward %.1fs, turn %.1fs, forward %.1fs, stop",
      initial_delay_sec_, first_forward_duration_sec_, turn_duration_sec_,
      second_forward_duration_sec_);
  }

private:
  void publish_step()
  {
    const auto current_time = now();
    if (current_time.nanoseconds() == 0) {
      return;
    }
    if (!start_time_.has_value()) {
      start_time_ = current_time;
    }

    const double elapsed_sec = (current_time - *start_time_).seconds();
    geometry_msgs::msg::Twist command;
    std::string phase;

    const double first_forward_end = initial_delay_sec_ + first_forward_duration_sec_;
    const double turn_end = first_forward_end + turn_duration_sec_;
    const double complete_time = turn_end + second_forward_duration_sec_;

    if (elapsed_sec < initial_delay_sec_) {
      phase = "initial_zero";
    } else if (elapsed_sec < first_forward_end) {
      phase = "forward_1";
      command.linear.x = forward_speed_m_s_;
    } else if (elapsed_sec < turn_end) {
      phase = "turn";
      command.angular.z = turn_rate_rad_s_;
    } else if (elapsed_sec < complete_time) {
      phase = "forward_2";
      command.linear.x = forward_speed_m_s_;
    } else {
      phase = "complete_zero";
    }

    if (phase != last_phase_) {
      last_phase_ = phase;
      RCLCPP_INFO(get_logger(), "motion test phase: %s", phase.c_str());
      if (phase == "complete_zero") {
        RCLCPP_INFO(get_logger(), "motion test complete; continuing to publish zero command");
      }
    }
    publisher_->publish(command);
  }

  double publish_rate_hz_{0.0};
  double initial_delay_sec_{0.0};
  double first_forward_duration_sec_{0.0};
  double turn_duration_sec_{0.0};
  double second_forward_duration_sec_{0.0};
  double forward_speed_m_s_{0.0};
  double turn_rate_rad_s_{0.0};
  std::optional<rclcpp::Time> start_time_;
  std::string last_phase_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotionTestNode>());
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "motion_test_node failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
