#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{

class PiStatusNode final : public rclcpp::Node
{
public:
  PiStatusNode()
  : Node("pi_status_node"), start_time_(std::chrono::steady_clock::now())
  {
    status_rate_hz_ = declare_parameter("status_rate_hz", 1.0);
    if (!std::isfinite(status_rate_hz_) || status_rate_hz_ <= 0.0) {
      throw std::invalid_argument("status_rate_hz must be finite and positive");
    }

    char hostname_buffer[256]{};
    hostname_ = gethostname(hostname_buffer, sizeof(hostname_buffer) - 1) == 0 ?
      std::string(hostname_buffer) : "unknown";
    startup_id_ = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        start_time_.time_since_epoch()).count());

    publisher_ = create_publisher<std_msgs::msg::String>("/hil/pi/status", rclcpp::QoS(10));
    ping_service_ = create_service<std_srvs::srv::Trigger>(
      "/hil/pi/ping",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        response->success = true;
        response->message = "pong";
      });

    const auto period_ms = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::llround(1000.0 / status_rate_hz_)));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms), [this]() { publish_status(); });
    publish_status();
  }

private:
  void publish_status()
  {
    const double uptime_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
    std_msgs::msg::String message;
    message.data = "hostname=" + hostname_ +
      ";uptime_sec=" + std::to_string(uptime_sec) +
      ";startup_id=" + std::to_string(startup_id_);
    publisher_->publish(message);
  }

  double status_rate_hz_{1.0};
  std::string hostname_;
  std::uint64_t startup_id_{0};
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ping_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PiStatusNode>());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("pi_status_node"), "failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
