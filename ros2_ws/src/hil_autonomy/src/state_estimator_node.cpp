#include "hil_autonomy/state_estimator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace
{
using std::placeholders::_1;
constexpr double kHalf = 0.5;

std::int64_t message_stamp_ns(const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & fallback)
{
  const auto seconds = static_cast<std::int64_t>(stamp.sec);
  const auto nanoseconds = static_cast<std::int64_t>(stamp.nanosec);
  const auto result = seconds * 1000000000LL + nanoseconds;
  return result > 0 ? result : fallback.nanoseconds();
}

class StateEstimatorNode final : public rclcpp::Node
{
public:
  StateEstimatorNode()
  : Node("state_estimator_node"),
    estimator_(
      declare_parameter("wheel_radius_m", 0.18),
      declare_parameter("wheel_separation_m", 0.86),
      declare_parameter("imu_yaw_weight", 0.25),
      declare_parameter("sensor_timeout_sec", 0.35),
      declare_parameter("max_sample_dt_sec", 0.2))
  {
    wheel_topic_ = declare_parameter("wheel_topic", std::string("/hil/sensors/wheel_states"));
    imu_topic_ = declare_parameter("imu_topic", std::string("/hil/sensors/imu"));
    odom_topic_ = declare_parameter("odom_topic", std::string("/hil/estimate/odom"));
    valid_topic_ = declare_parameter("valid_topic", std::string("/hil/estimate/valid"));
    left_joint_name_ = declare_parameter("left_joint_name", std::string("left_wheel_joint"));
    right_joint_name_ = declare_parameter("right_joint_name", std::string("right_wheel_joint"));
    odom_frame_ = declare_parameter("odom_frame", std::string("hil_estimate_odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
      wheel_topic_.empty() || imu_topic_.empty() || odom_topic_.empty() || valid_topic_.empty() ||
      left_joint_name_.empty() || right_joint_name_.empty())
    {
      throw std::invalid_argument("invalid state estimator node parameter");
    }

    wheel_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      wheel_topic_, rclcpp::SensorDataQoS(), std::bind(&StateEstimatorNode::wheel_callback, this, _1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(), std::bind(&StateEstimatorNode::imu_callback, this, _1));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    valid_publisher_ = create_publisher<std_msgs::msg::Bool>(valid_topic_, rclcpp::QoS(10));
    const auto period_ms = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::llround(1000.0 / publish_rate_hz_)));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this]() {publish_state();});
    RCLCPP_INFO(get_logger(), "wheel/IMU-only estimator publishing %s", odom_topic_.c_str());
  }

private:
  void wheel_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const auto left = std::find(message->name.begin(), message->name.end(), left_joint_name_);
    const auto right = std::find(message->name.begin(), message->name.end(), right_joint_name_);
    if (left == message->name.end() || right == message->name.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "wheel sample missing configured joints");
      return;
    }
    const auto left_index = static_cast<std::size_t>(std::distance(message->name.begin(), left));
    const auto right_index = static_cast<std::size_t>(std::distance(message->name.begin(), right));
    if (left_index >= message->position.size() || right_index >= message->position.size() ||
      left_index >= message->velocity.size() || right_index >= message->velocity.size())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "wheel sample missing position or velocity");
      return;
    }
    const auto stamp_ns = message_stamp_ns(message->header.stamp, now());
    estimator_.update_wheels({
        message->position[left_index], message->position[right_index],
        message->velocity[left_index], message->velocity[right_index], stamp_ns});
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const auto stamp_ns = message_stamp_ns(message->header.stamp, now());
    estimator_.update_imu({message->angular_velocity.z, stamp_ns});
  }

  void publish_state()
  {
    const auto current_time = now();
    const auto estimate = estimator_.state(current_time.nanoseconds());
    nav_msgs::msg::Odometry message;
    message.header.stamp = current_time;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    message.pose.pose.position.x = estimate.x_m;
    message.pose.pose.position.y = estimate.y_m;
    message.pose.pose.orientation.z = std::sin(kHalf * estimate.yaw_rad);
    message.pose.pose.orientation.w = std::cos(kHalf * estimate.yaw_rad);
    message.twist.twist.linear.x = estimate.linear_velocity_m_s;
    message.twist.twist.angular.z = estimate.yaw_rate_rad_s;
    message.pose.covariance[0] = estimate.valid ? 0.01 : 1.0e6;
    message.pose.covariance[7] = estimate.valid ? 0.01 : 1.0e6;
    message.pose.covariance[35] = estimate.valid ? 0.02 : 1.0e6;
    odom_publisher_->publish(message);
    std_msgs::msg::Bool valid_message;
    valid_message.data = estimate.valid;
    valid_publisher_->publish(valid_message);
  }

  hil_autonomy::StateEstimator estimator_;
  std::string wheel_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string valid_topic_;
  std::string left_joint_name_;
  std::string right_joint_name_;
  std::string odom_frame_;
  std::string base_frame_;
  double publish_rate_hz_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<StateEstimatorNode>());
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "state_estimator_node failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
