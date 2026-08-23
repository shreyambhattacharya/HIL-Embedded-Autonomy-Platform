#include "hil_control.h"

#include <math.h>

bool hil_control_finite(float value)
{
  return isfinite(value) != 0;
}

hil_control_wheel_pair_t hil_control_body_to_wheels(
  float linear_m_s, float yaw_rad_s, float wheel_radius_m, float wheel_separation_m)
{
  hil_control_wheel_pair_t result = {0.0F, 0.0F};
  if (!hil_control_finite(linear_m_s) || !hil_control_finite(yaw_rad_s) ||
    !hil_control_finite(wheel_radius_m) || !hil_control_finite(wheel_separation_m) ||
    wheel_radius_m <= 0.0F || wheel_separation_m < 0.0F) {
    return result;
  }
  const float half_track_m = wheel_separation_m * 0.5F;
  result.left_rad_s = (linear_m_s - yaw_rad_s * half_track_m) / wheel_radius_m;
  result.right_rad_s = (linear_m_s + yaw_rad_s * half_track_m) / wheel_radius_m;
  return result;
}

float hil_control_limited_p_effort(
  float target_rad_s, float measured_rad_s, float kp_nm_per_rad_s, float max_effort_nm)
{
  if (!hil_control_finite(target_rad_s) || !hil_control_finite(measured_rad_s) ||
    !hil_control_finite(kp_nm_per_rad_s) || !hil_control_finite(max_effort_nm)) {
    return 0.0F;
  }
  const float limit = max_effort_nm < 0.0F ? -max_effort_nm : max_effort_nm;
  const float effort = kp_nm_per_rad_s * (target_rad_s - measured_rad_s);
  if (!hil_control_finite(effort)) {
    return 0.0F;
  }
  if (effort > limit) {
    return limit;
  }
  if (effort < -limit) {
    return -limit;
  }
  return effort;
}
