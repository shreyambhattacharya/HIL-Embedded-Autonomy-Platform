#pragma once

#include <cstdint>

namespace hil_autonomy
{

struct WheelSample
{
  double left_position_rad{0.0};
  double right_position_rad{0.0};
  double left_velocity_rad_s{0.0};
  double right_velocity_rad_s{0.0};
  std::int64_t stamp_ns{0};
};

struct ImuSample
{
  double yaw_rate_rad_s{0.0};
  std::int64_t stamp_ns{0};
};

struct EstimateState
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double linear_velocity_m_s{0.0};
  double yaw_rate_rad_s{0.0};
  bool valid{false};
  bool wheel_fresh{false};
  bool imu_fresh{false};
  std::int64_t wheel_stamp_ns{0};
  std::int64_t imu_stamp_ns{0};
};

double wrap_angle(double angle_rad);

class StateEstimator final
{
public:
  StateEstimator(
    double wheel_radius_m,
    double wheel_separation_m,
    double imu_yaw_weight,
    double sensor_timeout_sec,
    double max_sample_dt_sec);

  bool update_wheels(const WheelSample & sample);
  bool update_imu(const ImuSample & sample);
  EstimateState state(std::int64_t now_ns) const;

private:
  double wheel_radius_m_;
  double wheel_separation_m_;
  double imu_yaw_weight_;
  double sensor_timeout_sec_;
  double max_sample_dt_sec_;

  bool have_previous_wheel_{false};
  bool have_wheel_{false};
  bool have_imu_{false};
  WheelSample previous_wheel_{};
  std::int64_t wheel_stamp_ns_{0};
  std::int64_t imu_stamp_ns_{0};
  double imu_yaw_rate_rad_s_{0.0};
  double x_m_{0.0};
  double y_m_{0.0};
  double yaw_rad_{0.0};
  double linear_velocity_m_s_{0.0};
  double yaw_rate_rad_s_{0.0};
};

}  // namespace hil_autonomy
