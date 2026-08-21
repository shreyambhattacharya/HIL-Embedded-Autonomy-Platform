#pragma once

#include <cmath>

namespace hil_control_stub
{

struct WheelTargets
{
  double left_rad_s{0.0};
  double right_rad_s{0.0};
};

WheelTargets body_twist_to_wheel_targets(
  double linear_m_s,
  double yaw_rad_s,
  double wheel_radius_m,
  double wheel_separation_m);

double limited_p_effort(
  double target_rad_s,
  double measured_rad_s,
  double kp_nm_per_rad_s,
  double max_effort_nm);

bool finite(double value);

}  // namespace hil_control_stub
