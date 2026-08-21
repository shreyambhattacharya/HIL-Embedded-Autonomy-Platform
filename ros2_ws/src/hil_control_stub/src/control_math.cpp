#include "hil_control_stub/control_math.hpp"

#include <algorithm>

namespace hil_control_stub
{

bool finite(const double value)
{
  return std::isfinite(value);
}

WheelTargets body_twist_to_wheel_targets(
  const double linear_m_s,
  const double yaw_rad_s,
  const double wheel_radius_m,
  const double wheel_separation_m)
{
  if (!finite(linear_m_s) || !finite(yaw_rad_s) || !finite(wheel_radius_m) ||
    !finite(wheel_separation_m) || wheel_radius_m <= 0.0 || wheel_separation_m < 0.0)
  {
    return {};
  }

  const double half_track_m = wheel_separation_m * 0.5;
  return {
    (linear_m_s - yaw_rad_s * half_track_m) / wheel_radius_m,
    (linear_m_s + yaw_rad_s * half_track_m) / wheel_radius_m};
}

double limited_p_effort(
  const double target_rad_s,
  const double measured_rad_s,
  const double kp_nm_per_rad_s,
  const double max_effort_nm)
{
  if (!finite(target_rad_s) || !finite(measured_rad_s) || !finite(kp_nm_per_rad_s) ||
    !finite(max_effort_nm))
  {
    return 0.0;
  }

  const double limit = std::abs(max_effort_nm);
  const double effort = kp_nm_per_rad_s * (target_rad_s - measured_rad_s);
  if (!finite(effort)) {
    return 0.0;
  }
  return std::clamp(effort, -limit, limit);
}

}  // namespace hil_control_stub
