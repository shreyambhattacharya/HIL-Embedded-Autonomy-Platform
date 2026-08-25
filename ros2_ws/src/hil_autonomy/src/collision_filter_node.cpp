#include "hil_autonomy/collision_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
using std::placeholders::_1;

class CollisionFilterNode final : public rclcpp::Node
{
public:
  CollisionFilterNode()
  : Node("collision_filter_node")
  {
    raw_topic_ = declare_parameter("raw_topic", std::string("/hil/autonomy/raw_twist"));
    scan_topic_ = declare_parameter("scan_topic", std::string("/hil/sensors/scan"));
    output_topic_ = declare_parameter("output_topic", std::string("/hil/control/target_twist"));
    state_topic_ = declare_parameter("state_topic", std::string("/hil/autonomy/collision_state"));
    min_distance_topic_ = declare_parameter(
      "min_distance_topic", std::string("/hil/autonomy/min_front_distance_m"));
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 20.0);
    scan_timeout_sec_ = declare_parameter("scan_timeout_sec", 0.5);
    front_sector_half_angle_rad_ = declare_parameter("front_sector_half_angle_rad", 0.70);
    stop_distance_m_ = declare_parameter("stop_distance_m", 0.55);
    slow_distance_m_ = declare_parameter("slow_distance_m", 1.20);
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
      !std::isfinite(scan_timeout_sec_) || scan_timeout_sec_ <= 0.0 ||
      !std::isfinite(front_sector_half_angle_rad_) || front_sector_half_angle_rad_ <= 0.0 ||
      !std::isfinite(stop_distance_m_) || !std::isfinite(slow_distance_m_) ||
      stop_distance_m_ <= 0.0 || slow_distance_m_ <= stop_distance_m_)
    {
      throw std::invalid_argument("invalid collision filter parameter");
    }
    raw_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      raw_topic_, rclcpp::QoS(10), std::bind(&CollisionFilterNode::raw_callback, this, _1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(), std::bind(&CollisionFilterNode::scan_callback, this, _1));
    output_publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, rclcpp::QoS(10));
    state_publisher_ = create_publisher<std_msgs::msg::String>(state_topic_, rclcpp::QoS(10));
    min_distance_publisher_ = create_publisher<std_msgs::msg::Float64>(
      min_distance_topic_, rclcpp::QoS(10));
    const auto period_ms = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::llround(1000.0 / publish_rate_hz_)));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this]() {publish_step();});
    RCLCPP_INFO(get_logger(), "LiDAR safety filter ready; output=%s", output_topic_.c_str());
  }

private:
  void raw_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    raw_linear_x_m_s_ = message->linear.x;
    raw_angular_z_rad_s_ = message->angular.z;
    last_raw_time_ = now();
    have_raw_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    last_scan_.angle_min_rad = message->angle_min;
    last_scan_.angle_increment_rad = message->angle_increment;
    last_scan_.ranges = message->ranges;
    last_scan_.stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (last_scan_.stamp_ns <= 0) {
      last_scan_.stamp_ns = now().nanoseconds();
    }
    have_scan_ = true;
  }

  void publish_step()
  {
    const auto current_time = now();
    const auto output = hil_autonomy::filter_twist(
      have_raw_ ? raw_linear_x_m_s_ : 0.0,
      have_raw_ ? raw_angular_z_rad_s_ : 0.0,
      have_scan_ ? last_scan_ : hil_autonomy::ScanView{},
      current_time.nanoseconds(), scan_timeout_sec_, front_sector_half_angle_rad_,
      stop_distance_m_, slow_distance_m_);
    geometry_msgs::msg::Twist command;
    command.linear.x = output.linear_x_m_s;
    command.angular.z = output.angular_z_rad_s;
    output_publisher_->publish(command);
    std_msgs::msg::String state_message;
    state_message.data = hil_autonomy::collision_state_name(output.state);
    state_publisher_->publish(state_message);
    std_msgs::msg::Float64 distance_message;
    distance_message.data = output.minimum_front_distance_m;
    min_distance_publisher_->publish(distance_message);
    if (output.state == hil_autonomy::CollisionState::STOP) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "collision filter STOP: front distance %.3f m",
        output.minimum_front_distance_m);
    } else if (output.state == hil_autonomy::CollisionState::STALE) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "collision filter STOP: LiDAR stale");
    }
  }

  std::string raw_topic_;
  std::string scan_topic_;
  std::string output_topic_;
  std::string state_topic_;
  std::string min_distance_topic_;
  double publish_rate_hz_{0.0};
  double scan_timeout_sec_{0.0};
  double front_sector_half_angle_rad_{0.0};
  double stop_distance_m_{0.0};
  double slow_distance_m_{0.0};
  double raw_linear_x_m_s_{0.0};
  double raw_angular_z_rad_s_{0.0};
  bool have_raw_{false};
  bool have_scan_{false};
  rclcpp::Time last_raw_time_{0, 0, RCL_ROS_TIME};
  hil_autonomy::ScanView last_scan_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr min_distance_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CollisionFilterNode>());
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "collision_filter_node failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
