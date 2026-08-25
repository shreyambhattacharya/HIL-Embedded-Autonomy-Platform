#include "hil_autonomy/state_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace hil_autonomy
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

bool finite(const double value)
{
  return std::isfinite(value);
}

double age_sec(const std::int64_t now_ns, const std::int64_t sample_ns)
{
  if (now_ns <= 0 || sample_ns <= 0 || now_ns < sample_ns) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - sample_ns) * 1.0e-9;
}
}  // namespace

double wrap_angle(const double angle_rad)
{
  double wrapped = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

StateEstimator::StateEstimator(
  const double wheel_radius_m,
  const double wheel_separation_m,
  const double imu_yaw_weight,
  const double sensor_timeout_sec,
  const double max_sample_dt_sec)
: wheel_radius_m_(wheel_radius_m),
  wheel_separation_m_(wheel_separation_m),
  imu_yaw_weight_(imu_yaw_weight),
  sensor_timeout_sec_(sensor_timeout_sec),
  max_sample_dt_sec_(max_sample_dt_sec)
{
  if (!finite(wheel_radius_m_) || wheel_radius_m_ <= 0.0 ||
    !finite(wheel_separation_m_) || wheel_separation_m_ <= 0.0 ||
    !finite(imu_yaw_weight_) || imu_yaw_weight_ < 0.0 || imu_yaw_weight_ > 1.0 ||
    !finite(sensor_timeout_sec_) || sensor_timeout_sec_ <= 0.0 ||
    !finite(max_sample_dt_sec_) || max_sample_dt_sec_ <= 0.0)
  {
    throw std::invalid_argument("invalid state estimator configuration");
  }
}

bool StateEstimator::update_wheels(const WheelSample & sample)
{
  if (!finite(sample.left_position_rad) || !finite(sample.right_position_rad) ||
    !finite(sample.left_velocity_rad_s) || !finite(sample.right_velocity_rad_s) ||
    sample.stamp_ns <= 0)
  {
    return false;
  }

  if (have_previous_wheel_) {
    const auto delta_ns = sample.stamp_ns - previous_wheel_.stamp_ns;
    const double dt_sec = static_cast<double>(delta_ns) * 1.0e-9;
    if (delta_ns <= 0 || !finite(dt_sec) || dt_sec > max_sample_dt_sec_) {
      previous_wheel_ = sample;
      wheel_stamp_ns_ = sample.stamp_ns;
      have_wheel_ = true;
      return false;
    }

    const double left_delta_m = wheel_radius_m_ *
      (sample.left_position_rad - previous_wheel_.left_position_rad);
    const double right_delta_m = wheel_radius_m_ *
      (sample.right_position_rad - previous_wheel_.right_position_rad);
    const double distance_m = 0.5 * (left_delta_m + right_delta_m);
    const double wheel_yaw_delta = (right_delta_m - left_delta_m) / wheel_separation_m_;
    const double imu_yaw_delta = have_imu_ ? imu_yaw_rate_rad_s_ * dt_sec : wheel_yaw_delta;
    const double yaw_delta = (1.0 - imu_yaw_weight_) * wheel_yaw_delta +
      imu_yaw_weight_ * imu_yaw_delta;
    const double midpoint_yaw = yaw_rad_ + 0.5 * yaw_delta;
    x_m_ += distance_m * std::cos(midpoint_yaw);
    y_m_ += distance_m * std::sin(midpoint_yaw);
    yaw_rad_ = wrap_angle(yaw_rad_ + yaw_delta);
    linear_velocity_m_s_ = 0.5 * wheel_radius_m_ *
      (sample.left_velocity_rad_s + sample.right_velocity_rad_s);
    const double wheel_yaw_rate = wheel_radius_m_ /
      wheel_separation_m_ * (sample.right_velocity_rad_s - sample.left_velocity_rad_s);
    yaw_rate_rad_s_ = (1.0 - imu_yaw_weight_) * wheel_yaw_rate +
      imu_yaw_weight_ * imu_yaw_rate_rad_s_;
  }

  previous_wheel_ = sample;
  wheel_stamp_ns_ = sample.stamp_ns;
  have_previous_wheel_ = true;
  have_wheel_ = true;
  return true;
}

bool StateEstimator::update_imu(const ImuSample & sample)
{
  if (!finite(sample.yaw_rate_rad_s) || sample.stamp_ns <= 0) {
    return false;
  }
  imu_yaw_rate_rad_s_ = sample.yaw_rate_rad_s;
  imu_stamp_ns_ = sample.stamp_ns;
  have_imu_ = true;
  return true;
}

EstimateState StateEstimator::state(const std::int64_t now_ns) const
{
  EstimateState output;
  output.x_m = x_m_;
  output.y_m = y_m_;
  output.yaw_rad = yaw_rad_;
  output.linear_velocity_m_s = linear_velocity_m_s_;
  output.yaw_rate_rad_s = yaw_rate_rad_s_;
  output.wheel_stamp_ns = wheel_stamp_ns_;
  output.imu_stamp_ns = imu_stamp_ns_;
  output.wheel_fresh = have_wheel_ && age_sec(now_ns, wheel_stamp_ns_) <= sensor_timeout_sec_;
  output.imu_fresh = have_imu_ && age_sec(now_ns, imu_stamp_ns_) <= sensor_timeout_sec_;
  output.valid = output.wheel_fresh && output.imu_fresh;
  return output;
}

}  // namespace hil_autonomy
