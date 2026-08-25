#include "hil_autonomy/waypoint_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
using std::placeholders::_1;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

class WaypointFollowerNode final : public rclcpp::Node
{
public:
  WaypointFollowerNode()
  : Node("waypoint_follower_node"),
    controller_(make_waypoints(), declare_parameter("waypoint_tolerance_m", 0.10),
      declare_parameter("heading_kp", 2.0), declare_parameter("linear_kp", 0.8),
      declare_parameter("max_linear_m_s", 0.25), declare_parameter("max_angular_rad_s", 0.8),
      declare_parameter("turn_in_place_error_rad", 0.65))
  {
    odom_topic_ = declare_parameter("odom_topic", std::string("/hil/estimate/odom"));
    valid_topic_ = declare_parameter("valid_topic", std::string("/hil/estimate/valid"));
    raw_topic_ = declare_parameter("raw_topic", std::string("/hil/autonomy/raw_twist"));
    state_topic_ = declare_parameter("state_topic", std::string("/hil/autonomy/state"));
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
    estimate_timeout_sec_ = declare_parameter("estimate_timeout_sec", 0.5);
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
      !std::isfinite(estimate_timeout_sec_) || estimate_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("invalid waypoint follower node parameter");
    }
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10), std::bind(&WaypointFollowerNode::odom_callback, this, _1));
    valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      valid_topic_, rclcpp::QoS(10), [this](const std_msgs::msg::Bool::SharedPtr message) {
        estimate_valid_ = message->data;
      });
    raw_publisher_ = create_publisher<geometry_msgs::msg::Twist>(raw_topic_, rclcpp::QoS(10));
    state_publisher_ = create_publisher<std_msgs::msg::String>(state_topic_, rclcpp::QoS(10));
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/hil/autonomy/start", std::bind(&WaypointFollowerNode::start_callback, this,
      std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/hil/autonomy/stop", std::bind(&WaypointFollowerNode::stop_callback, this,
      std::placeholders::_1, std::placeholders::_2));
    const auto period_ms = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::llround(1000.0 / publish_rate_hz_)));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this]() {publish_step();});
    RCLCPP_INFO(get_logger(), "waypoint follower ready; waiting for /hil/autonomy/start");
  }

private:
  std::vector<hil_autonomy::Waypoint> make_waypoints()
  {
    const auto x = declare_parameter<std::vector<double>>(
      "waypoint_x_m", {1.0, 1.0, 0.0, 0.0});
    const auto y = declare_parameter<std::vector<double>>(
      "waypoint_y_m", {0.0, 1.0, 1.0, 0.0});
    if (x.empty() || x.size() != y.size()) {
      throw std::invalid_argument("waypoint_x_m and waypoint_y_m must be non-empty and equal length");
    }
    std::vector<hil_autonomy::Waypoint> waypoints;
    waypoints.reserve(x.size());
    for (std::size_t index = 0U; index < x.size(); ++index) {
      waypoints.push_back({x[index], y[index]});
    }
    return waypoints;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    latest_estimate_.x_m = message->pose.pose.position.x;
    latest_estimate_.y_m = message->pose.pose.position.y;
    latest_estimate_.yaw_rad = yaw_from_quaternion(message->pose.pose.orientation);
    latest_estimate_.linear_velocity_m_s = message->twist.twist.linear.x;
    latest_estimate_.yaw_rate_rad_s = message->twist.twist.angular.z;
    latest_estimate_.valid = estimate_valid_;
    have_estimate_ = true;
    last_estimate_time_ = now();
  }

  void start_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    controller_.start();
    response->success = true;
    response->message = "waypoint follower started; controller uses local estimated pose";
    RCLCPP_INFO(get_logger(), "autonomy start accepted");
  }

  void stop_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    controller_.stop();
    response->success = true;
    response->message = "waypoint follower stopped; publishing zero twist";
    RCLCPP_INFO(get_logger(), "autonomy stop accepted");
  }

  void publish_step()
  {
    const auto current_time = now();
    auto estimate = latest_estimate_;
    estimate.valid = have_estimate_ && estimate_valid_ &&
      (current_time - last_estimate_time_).seconds() <= estimate_timeout_sec_;
    const auto output = controller_.compute(estimate);
    geometry_msgs::msg::Twist command;
    command.linear.x = output.command.linear_x_m_s;
    command.angular.z = output.command.angular_z_rad_s;
    raw_publisher_->publish(command);
    std_msgs::msg::String state_message;
    if (controller_.complete()) {
      state_message.data = "COMPLETE";
    } else if (controller_.active() && !estimate.valid) {
      state_message.data = "FAULT_ESTIMATE_STALE";
    } else if (controller_.active()) {
      state_message.data = "RUNNING";
    } else {
      state_message.data = "IDLE";
    }
    state_publisher_->publish(state_message);
  }

  hil_autonomy::WaypointController controller_;
  nav_msgs::msg::Odometry latest_odom_{};
  hil_autonomy::EstimateState latest_estimate_{};
  bool have_estimate_{false};
  bool estimate_valid_{false};
  rclcpp::Time last_estimate_time_{0, 0, RCL_ROS_TIME};
  std::string odom_topic_;
  std::string valid_topic_;
  std::string raw_topic_;
  std::string state_topic_;
  double publish_rate_hz_{0.0};
  double estimate_timeout_sec_{0.0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr raw_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WaypointFollowerNode>());
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "waypoint_follower_node failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
