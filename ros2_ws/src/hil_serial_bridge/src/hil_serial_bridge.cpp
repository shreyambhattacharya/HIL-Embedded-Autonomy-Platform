#include "hil_protocol.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <dirent.h>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{

using Clock = std::chrono::steady_clock;
using Trigger = std_srvs::srv::Trigger;

uint32_t monotonic_ms()
{
  return static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      Clock::now().time_since_epoch()).count());
}

void write_u32(uint8_t *out, uint32_t value)
{
  out[0] = static_cast<uint8_t>(value & 0xffU);
  out[1] = static_cast<uint8_t>((value >> 8U) & 0xffU);
  out[2] = static_cast<uint8_t>((value >> 16U) & 0xffU);
  out[3] = static_cast<uint8_t>((value >> 24U) & 0xffU);
}

uint16_t read_u16(const uint8_t *in)
{
  return static_cast<uint16_t>(in[0]) |
    static_cast<uint16_t>(static_cast<uint16_t>(in[1]) << 8U);
}

uint32_t read_u32(const uint8_t *in)
{
  return static_cast<uint32_t>(in[0]) |
    (static_cast<uint32_t>(in[1]) << 8U) |
    (static_cast<uint32_t>(in[2]) << 16U) |
    (static_cast<uint32_t>(in[3]) << 24U);
}

speed_t baud_constant(int baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
#ifdef B921600
    case 921600: return B921600;
#endif
    default: throw std::invalid_argument("unsupported baud_rate; use 115200, 230400, 460800, or 921600");
  }
}

class HilSerialBridge final : public rclcpp::Node
{
public:
  HilSerialBridge()
  : Node("hil_serial_bridge")
  {
    serial_device_ = declare_parameter("serial_device", std::string("/dev/serial/by-id"));
    baud_rate_ = static_cast<int>(declare_parameter<int64_t>("baud_rate", 115200));
    command_tx_rate_hz_ = declare_parameter("command_tx_rate_hz", 100.0);
    feedback_tx_rate_hz_ = declare_parameter("feedback_tx_rate_hz", 100.0);
    heartbeat_rate_hz_ = declare_parameter("heartbeat_rate_hz", 10.0);
    status_timeout_ms_ = static_cast<int>(declare_parameter<int64_t>("status_timeout_ms", 250));
    reconnect_period_ms_ = static_cast<int>(declare_parameter<int64_t>("reconnect_period_ms", 1000));
    left_joint_name_ = declare_parameter("left_joint_name", std::string("left_wheel_joint"));
    right_joint_name_ = declare_parameter("right_joint_name", std::string("right_wheel_joint"));

    if (serial_device_.empty() || baud_rate_ <= 0 || command_tx_rate_hz_ <= 0.0 ||
      feedback_tx_rate_hz_ <= 0.0 || heartbeat_rate_hz_ <= 0.0 || status_timeout_ms_ <= 0 ||
      reconnect_period_ms_ <= 0 || left_joint_name_.empty() || right_joint_name_.empty()) {
      throw std::invalid_argument("invalid hil_serial_bridge parameter");
    }
    (void)baud_constant(baud_rate_);

    effort_left_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/hil/actuator/left_effort", rclcpp::QoS(10));
    effort_right_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/hil/actuator/right_effort", rclcpp::QoS(10));
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/hil/stm32/status", rclcpp::QoS(10));

    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/hil/control/target_twist", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) { target_callback(message); });
    wheel_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/hil/sensors/wheel_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) { wheel_callback(message); });

    arm_service_ = create_service<Trigger>(
      "/hil/stm32/arm",
      [this](const Trigger::Request::SharedPtr, const Trigger::Response::SharedPtr response) {
        mode_request(HIL_MODE_ARM, response);
      });
    disarm_service_ = create_service<Trigger>(
      "/hil/stm32/disarm",
      [this](const Trigger::Request::SharedPtr, const Trigger::Response::SharedPtr response) {
        mode_request(HIL_MODE_DISARM, response);
      });
    ping_service_ = create_service<Trigger>(
      "/hil/stm32/ping",
      [this](const Trigger::Request::SharedPtr, const Trigger::Response::SharedPtr response) {
        hil_protocol_frame_t frame{};
        const bool packed = hil_protocol_pack_ping(&frame, monotonic_ms());
        response->success = packed && send_frame(frame);
        response->message = response->success ? "PING transmitted" : "serial link unavailable";
      });

    const auto command_period = std::chrono::milliseconds(
      std::max<int64_t>(1, static_cast<int64_t>(std::llround(1000.0 / command_tx_rate_hz_))));
    const auto feedback_period = std::chrono::milliseconds(
      std::max<int64_t>(1, static_cast<int64_t>(std::llround(1000.0 / feedback_tx_rate_hz_))));
    const auto heartbeat_period = std::chrono::milliseconds(
      std::max<int64_t>(1, static_cast<int64_t>(std::llround(1000.0 / heartbeat_rate_hz_))));
    command_timer_ = create_wall_timer(command_period, [this]() { send_command(); });
    feedback_timer_ = create_wall_timer(feedback_period, [this]() { send_feedback(); });
    heartbeat_timer_ = create_wall_timer(heartbeat_period, [this]() { send_heartbeat(); });
    read_timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() { read_serial(); });
    safety_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { safety_check(); });
    reconnect_timer_ = create_wall_timer(
      std::chrono::milliseconds(reconnect_period_ms_), [this]() { try_open(); });

    try_open();
    publish_zero_effort();
    RCLCPP_INFO(
      get_logger(), "serial bridge configured: device=%s baud=%d command=%.1f Hz feedback=%.1f Hz",
      serial_device_.c_str(), baud_rate_, command_tx_rate_hz_, feedback_tx_rate_hz_);
  }

  ~HilSerialBridge() override
  {
    close_serial();
  }

private:
  void target_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
      have_command_ = false;
      publish_zero_effort();
      return;
    }
    linear_m_s_ = static_cast<float>(message->linear.x);
    yaw_rad_s_ = static_cast<float>(message->angular.z);
    have_command_ = true;
    last_command_time_ = Clock::now();
  }

  void wheel_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const auto left = std::find(message->name.begin(), message->name.end(), left_joint_name_);
    const auto right = std::find(message->name.begin(), message->name.end(), right_joint_name_);
    if (left == message->name.end() || right == message->name.end()) {
      have_feedback_ = false;
      return;
    }
    const auto left_index = static_cast<size_t>(std::distance(message->name.begin(), left));
    const auto right_index = static_cast<size_t>(std::distance(message->name.begin(), right));
    if (left_index >= message->velocity.size() || right_index >= message->velocity.size() ||
      !std::isfinite(message->velocity[left_index]) || !std::isfinite(message->velocity[right_index])) {
      have_feedback_ = false;
      return;
    }
    left_wheel_rad_s_ = static_cast<float>(message->velocity[left_index]);
    right_wheel_rad_s_ = static_cast<float>(message->velocity[right_index]);
    have_feedback_ = true;
    last_feedback_time_ = Clock::now();
  }

  bool configure_serial(int fd)
  {
    termios settings{};
    if (tcgetattr(fd, &settings) != 0) {
      return false;
    }
    cfmakeraw(&settings);
    const speed_t speed = baud_constant(baud_rate_);
    cfsetispeed(&settings, speed);
    cfsetospeed(&settings, speed);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= static_cast<unsigned>(~CRTSCTS);
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    return tcsetattr(fd, TCSANOW, &settings) == 0;
  }

  std::string resolve_serial_device() const
  {
    struct stat info{};
    if (stat(serial_device_.c_str(), &info) != 0 || !S_ISDIR(info.st_mode)) {
      return serial_device_;
    }
    DIR *directory = opendir(serial_device_.c_str());
    if (directory == nullptr) {
      return {};
    }
    std::string fallback;
    std::string stable;
    while (dirent *entry = readdir(directory)) {
      const std::string name(entry->d_name);
      if (name == "." || name == "..") {
        continue;
      }
      const std::string path = serial_device_ + "/" + name;
      if (name.find("STMicroelectronics_STM32_STLink") != std::string::npos) {
        stable = path;
      } else if (fallback.empty() && name.find("ttyACM") != std::string::npos) {
        fallback = path;
      }
    }
    closedir(directory);
    return stable.empty() ? fallback : stable;
  }

  void publish_link_down(const char *reason)
  {
    if (!link_down_) {
      publish_status(std::string("link=DOWN;reason=") + reason);
    }
    link_down_ = true;
  }

  void try_open()
  {
    if (serial_fd_ >= 0) {
      return;
    }
    const std::string device = resolve_serial_device();
    if (device.empty()) {
      publish_link_down("device_unavailable");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "serial device unavailable under %s", serial_device_.c_str());
      return;
    }
    const int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      publish_link_down("open");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "serial device unavailable: %s", device.c_str());
      return;
    }
    if (!configure_serial(fd)) {
      close(fd);
      publish_link_down("configure");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "serial device configuration failed: %s", device.c_str());
      return;
    }
    tcflush(fd, TCIOFLUSH);
    serial_fd_ = fd;
    armed_ = false;
    have_boot_id_ = false;
    hil_protocol_decoder_init(&decoder_);
    last_rx_time_ = Clock::now();
    link_down_ = false;
    publish_zero_effort();
    publish_status("link=UP;state=WAIT_LINK;device=" + device + ";boot_id=unknown");

    send_hello();
    RCLCPP_INFO(get_logger(), "opened STM32 serial device %s", device.c_str());
  }
  void close_serial()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void link_failure(const char *reason)
  {
    close_serial();
    armed_ = false;
    have_boot_id_ = false;
    publish_zero_effort();
    publish_link_down(reason);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "STM32 serial link failure: %s", reason);
  }

  bool send_frame(hil_protocol_frame_t frame)
  {
    if (serial_fd_ < 0) {
      return false;
    }
    frame.protocol_version = HIL_PROTOCOL_VERSION;
    frame.sequence = tx_sequence_++;
    frame.sender_tick_ms = monotonic_ms();
    uint8_t encoded[HIL_PROTOCOL_MAX_ENCODED_FRAME]{};
    size_t encoded_length = 0U;
    if (!hil_protocol_encode_frame(&frame, encoded, sizeof(encoded), &encoded_length)) {
      link_failure("encode");
      return false;
    }
    size_t written = 0U;
    while (written < encoded_length) {
      const ssize_t count = write(serial_fd_, &encoded[written], encoded_length - written);
      if (count > 0) {
        written += static_cast<size_t>(count);
        continue;
      }
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd descriptor{serial_fd_, POLLOUT, 0};
        if (poll(&descriptor, 1, 2) > 0) {
          continue;
        }
      }
      link_failure("write");
      return false;
    }
    return true;
  }

  void send_hello()
  {
    hil_protocol_frame_t frame{};
    frame.message_type = HIL_MSG_HELLO;
    frame.payload_length = 10U;
    frame.payload[0] = HIL_ROLE_LINUX_BRIDGE;
    frame.payload[1] = HIL_PROTOCOL_VERSION;
    write_u32(&frame.payload[2], 0U);
    write_u32(&frame.payload[6], 0U);
    (void)send_frame(frame);
  }

  void send_command()
  {
    hil_protocol_frame_t frame{};
    if (!hil_protocol_pack_control(&frame, have_command_ ? linear_m_s_ : 0.0F, have_command_ ? yaw_rad_s_ : 0.0F)) {
      return;
    }
    (void)send_frame(frame);
  }

  void send_feedback()
  {
    hil_protocol_frame_t frame{};
    if (!hil_protocol_pack_wheel_pair(
        &frame, HIL_MSG_WHEEL_FEEDBACK,
        have_feedback_ ? left_wheel_rad_s_ : 0.0F,
        have_feedback_ ? right_wheel_rad_s_ : 0.0F)) {
      return;
    }
    (void)send_frame(frame);
  }

  void send_heartbeat()
  {
    hil_protocol_frame_t frame{};
    frame.message_type = HIL_MSG_HEARTBEAT;
    frame.payload_length = 5U;
    write_u32(&frame.payload[0], monotonic_ms());
    frame.payload[4] = armed_ ? HIL_STATE_ACTIVE : HIL_STATE_DISARMED;
    (void)send_frame(frame);
  }

  void read_serial()
  {
    if (serial_fd_ < 0) {
      return;
    }
    pollfd descriptor{serial_fd_, POLLIN | POLLERR | POLLHUP, 0};
    const int ready = poll(&descriptor, 1, 0);
    if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      link_failure("poll");
      return;
    }
    if (ready == 0 || (descriptor.revents & POLLIN) == 0) {
      return;
    }
    uint8_t buffer[256]{};
    for (;;) {
      const ssize_t count = read(serial_fd_, buffer, sizeof(buffer));
      if (count > 0) {
        for (ssize_t i = 0; i < count; ++i) {
          hil_protocol_frame_t frame{};
          const auto result = hil_protocol_decoder_feed(&decoder_, buffer[i], &frame);
          if (result == HIL_PROTOCOL_FRAME_READY) {
            handle_frame(frame);
          }
        }
        continue;
      }
      if (count == 0) {
        link_failure("hangup");
        return;
      } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
        link_failure("read");
      }
      return;
    }
  }

  void observe_boot_id(uint32_t boot_id)
  {
    if (have_boot_id_ && boot_id != boot_id_) {
      armed_ = false;
      publish_zero_effort();
    }
    boot_id_ = boot_id;
    have_boot_id_ = true;
  }

  void handle_frame(const hil_protocol_frame_t &frame)
  {
    last_rx_time_ = Clock::now();
    if (frame.message_type == HIL_MSG_HELLO) {
      if (frame.payload_length < 10U || frame.payload[0] != HIL_ROLE_STM32 ||
        frame.payload[1] != HIL_PROTOCOL_VERSION) {
        publish_status("link=UP;state=SAFE;safety_reason=protocol_incompatible");
        return;
      }
      const uint32_t boot_id = read_u32(&frame.payload[2]);
      observe_boot_id(boot_id);
      armed_ = false;
      publish_zero_effort();
      publish_status("link=UP;state=WAIT_LINK;hello=STM32_F446RE;boot_id=" + std::to_string(boot_id));
      return;
    }
    if (frame.message_type == HIL_MSG_WHEEL_EFFORT) {
      float left = 0.0F;
      float right = 0.0F;
      if (hil_protocol_unpack_wheel_pair(&frame, &left, &right) && std::isfinite(left) && std::isfinite(right)) {
        publish_effort(left, right);
        last_effort_time_ = Clock::now();
      }
      return;
    }
    if (frame.message_type == HIL_MSG_TIMING_STATUS &&
      frame.payload_length >= HIL_TIMING_STATUS_PAYLOAD_SIZE) {
      const auto state = frame.payload[0];
      const auto safety_reason = frame.payload[1];
      const auto reset_cause = frame.payload[2];
      const uint32_t boot_id = read_u32(&frame.payload[4]);
      observe_boot_id(boot_id);
      armed_ = state == HIL_STATE_ACTIVE;
      std::ostringstream status;
      status << "link=UP;timing=1;state=" << static_cast<int>(state)
        << ";safety_reason=" << static_cast<int>(safety_reason)
        << ";reset_cause=" << static_cast<int>(reset_cause)
        << ";boot_id=" << boot_id
        << ";sample_count=" << read_u32(&frame.payload[12])
        << ";exec_min_us=" << read_u32(&frame.payload[16])
        << ";exec_mean_us=" << read_u32(&frame.payload[20])
        << ";exec_max_us=" << read_u32(&frame.payload[24])
        << ";period_min_us=" << read_u32(&frame.payload[28])
        << ";period_mean_us=" << read_u32(&frame.payload[32])
        << ";period_max_us=" << read_u32(&frame.payload[36])
        << ";deadline_misses=" << read_u32(&frame.payload[40])
        << ";rx_stream_drops=" << read_u32(&frame.payload[44])
        << ";tx_queue_drops=" << read_u32(&frame.payload[48])
        << ";uart_overruns=" << read_u32(&frame.payload[52])
        << ";rx_stack_hwm=" << read_u16(&frame.payload[56])
        << ";control_stack_hwm=" << read_u16(&frame.payload[58])
        << ";tx_stack_hwm=" << read_u16(&frame.payload[60]);
      publish_status(status.str());
      if (frame.payload_length >= HIL_STATUS_PAYLOAD_SIZE) {
        const auto state = frame.payload[0];
        const auto safety_reason = frame.payload[1];
        const auto reset_cause = frame.payload[2];
        const uint32_t boot_id = read_u32(&frame.payload[4]);
        observe_boot_id(boot_id);
        armed_ = state == HIL_STATE_ACTIVE;
        std::ostringstream status;
        status << "link=UP;state=" << static_cast<int>(state)
          << ";safety_reason=" << static_cast<int>(safety_reason)
          << ";reset_cause=" << static_cast<int>(reset_cause)
          << ";boot_id=" << boot_id
          << ";rx_valid=" << read_u32(&frame.payload[12])
          << ";rx_crc=" << read_u32(&frame.payload[16])
          << ";rx_decode=" << read_u32(&frame.payload[20])
          << ";rx_len=" << read_u32(&frame.payload[24])
          << ";rx_version=" << read_u32(&frame.payload[28])
          << ";rx_dupe=" << read_u32(&frame.payload[32])
          << ";rx_stale=" << read_u32(&frame.payload[36])
          << ";rx_gaps=" << read_u32(&frame.payload[40])
          << ";rx_stream_drops=" << read_u32(&frame.payload[44])
          << ";tx_queue_drops=" << read_u32(&frame.payload[48])
          << ";uart_overruns=" << read_u32(&frame.payload[52]);
        publish_status(status.str());
      } else if (frame.payload_length >= 26U) {
        const auto state = frame.payload[0];
        const auto fault = frame.payload[1];
        const uint32_t boot_id = read_u32(&frame.payload[2]);
        observe_boot_id(boot_id);
        armed_ = state == HIL_STATE_ACTIVE;
        std::ostringstream status;
        status << "link=UP;state=" << static_cast<int>(state)
          << ";fault=" << static_cast<int>(fault)
          << ";boot_id=" << boot_id
          << ";rx_valid=" << read_u32(&frame.payload[10])
          << ";rx_crc=" << read_u32(&frame.payload[14]);
        publish_status(status.str());
      }
      return;
    }
    if (frame.message_type == HIL_MSG_ACK) {
      uint8_t command = 0U;
      uint32_t transaction = 0U;
      uint8_t result = HIL_ACK_INVALID;
      if (hil_protocol_unpack_ack(&frame, &command, &transaction, &result)) {
        publish_status(
          "link=UP;ack_command=" + std::to_string(command) +
          ";transaction=" + std::to_string(transaction) +
          ";result=" + std::to_string(result));
      }
      return;
    }
    if (frame.message_type == HIL_MSG_PONG) {
      publish_status("link=UP;pong=received");
    }
  }

  void safety_check()
  {
    if (serial_fd_ >= 0 &&
      std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - last_rx_time_).count() > status_timeout_ms_) {
      link_failure("timeout");
    }
  }

  void publish_effort(float left, float right)
  {
    std_msgs::msg::Float64 left_message;
    std_msgs::msg::Float64 right_message;
    left_message.data = static_cast<double>(left);
    right_message.data = static_cast<double>(right);
    effort_left_publisher_->publish(left_message);
    effort_right_publisher_->publish(right_message);
  }

  void publish_zero_effort()
  {
    publish_effort(0.0F, 0.0F);
  }

  void publish_status(const std::string &text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_publisher_->publish(message);
  }

  void mode_request(uint8_t mode, const Trigger::Response::SharedPtr &response)
  {
    if (mode == HIL_MODE_ARM && serial_fd_ < 0) {
      response->success = false;
      response->message = "serial link unavailable; refusing ARM";
      return;
    }
    if (mode == HIL_MODE_ARM && !have_boot_id_) {
      response->success = false;
      response->message = "STM32 HELLO has not arrived; refusing ARM";
      return;
    }
    if (mode == HIL_MODE_ARM && (!have_feedback_ || !have_command_)) {
      response->success = false;
      response->message = "command and wheel feedback are required; refusing ARM";
      return;
    }
    hil_protocol_frame_t frame{};
    const uint32_t transaction = ++transaction_id_;
    const bool packed = hil_protocol_pack_mode(&frame, mode, transaction);
    response->success = packed && send_frame(frame);
    response->message = response->success ?
      "MODE_COMMAND transmitted; await STM32 ACK" : "serial link unavailable";
    if (response->success) {
      armed_ = mode == HIL_MODE_ARM;
      if (mode == HIL_MODE_DISARM) {
        publish_zero_effort();
      }
    }
  }

  int serial_fd_{-1};
  bool link_down_{false};
  std::string serial_device_;
  int baud_rate_{115200};
  double command_tx_rate_hz_{100.0};
  double feedback_tx_rate_hz_{100.0};
  double heartbeat_rate_hz_{10.0};
  int status_timeout_ms_{250};
  int reconnect_period_ms_{1000};
  std::string left_joint_name_;
  std::string right_joint_name_;
  float linear_m_s_{0.0F};
  float yaw_rad_s_{0.0F};
  float left_wheel_rad_s_{0.0F};
  float right_wheel_rad_s_{0.0F};
  bool have_command_{false};
  bool have_feedback_{false};
  bool armed_{false};
  bool have_boot_id_{false};
  uint32_t boot_id_{0U};
  uint32_t tx_sequence_{0U};
  uint32_t transaction_id_{0U};
  Clock::time_point last_command_time_{Clock::now()};
  Clock::time_point last_feedback_time_{Clock::now()};
  Clock::time_point last_effort_time_{Clock::now()};
  Clock::time_point last_rx_time_{Clock::now()};
  hil_protocol_decoder_t decoder_{};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr effort_left_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr effort_right_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_subscription_;
  rclcpp::Service<Trigger>::SharedPtr arm_service_;
  rclcpp::Service<Trigger>::SharedPtr disarm_service_;
  rclcpp::Service<Trigger>::SharedPtr ping_service_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HilSerialBridge>());
  } catch (const std::exception &exception) {
    std::fprintf(stderr, "hil_serial_bridge failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
