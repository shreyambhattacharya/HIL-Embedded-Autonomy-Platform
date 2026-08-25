#include "hil_autonomy/collision_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hil_autonomy
{
namespace
{
bool finite(const double value)
{
  return std::isfinite(value);
}

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}
}  // namespace

const char * collision_state_name(const CollisionState state)
{
  switch (state) {
    case CollisionState::CLEAR: return "CLEAR";
    case CollisionState::SLOWDOWN: return "SLOWDOWN";
    case CollisionState::STOP: return "STOP";
    case CollisionState::STALE: return "STALE";
    case CollisionState::INVALID: return "INVALID";
  }
  return "INVALID";
}

FilterOutput filter_twist(
  const double raw_linear_x_m_s,
  const double raw_angular_z_rad_s,
  const ScanView & scan,
  const std::int64_t now_ns,
  const double scan_timeout_sec,
  const double front_sector_half_angle_rad,
  const double stop_distance_m,
  const double slow_distance_m)
{
  FilterOutput output;
  if (!finite(raw_linear_x_m_s) || !finite(raw_angular_z_rad_s) ||
    !finite(scan_timeout_sec) || scan_timeout_sec <= 0.0 ||
    !finite(front_sector_half_angle_rad) || front_sector_half_angle_rad <= 0.0 ||
    !finite(stop_distance_m) || !finite(slow_distance_m) ||
    stop_distance_m <= 0.0 || slow_distance_m <= stop_distance_m ||
    !finite(scan.angle_min_rad) || !finite(scan.angle_increment_rad) ||
    scan.angle_increment_rad <= 0.0 || scan.ranges.empty() ||
    scan.stamp_ns <= 0 || now_ns <= 0)
  {
    output.state = CollisionState::INVALID;
    return output;
  }

  const auto age_ns = now_ns - scan.stamp_ns;
  if (age_ns < 0 || static_cast<double>(age_ns) * 1.0e-9 > scan_timeout_sec) {
    output.state = CollisionState::STALE;
    return output;
  }

  double minimum_distance = std::numeric_limits<double>::infinity();
  bool saw_valid_sample = false;
  for (std::size_t index = 0U; index < scan.ranges.size(); ++index) {
    const double angle = scan.angle_min_rad +
      static_cast<double>(index) * scan.angle_increment_rad;
    const double range = static_cast<double>(scan.ranges[index]);
    if (std::abs(angle) > front_sector_half_angle_rad || std::isnan(range) ||
      range <= 0.0)
    {
      continue;
    }
    saw_valid_sample = true;
    if (finite(range)) {
      minimum_distance = std::min(minimum_distance, range);
    }
  }

  if (!saw_valid_sample) {
    output.state = CollisionState::INVALID;
    return output;
  }

  output.minimum_front_distance_m = minimum_distance;
  if (minimum_distance <= stop_distance_m) {
    output.state = CollisionState::STOP;
    return output;
  }

  if (minimum_distance < slow_distance_m) {
    const double scale = clamp(
      (minimum_distance - stop_distance_m) / (slow_distance_m - stop_distance_m), 0.0, 1.0);
    output.linear_x_m_s = raw_linear_x_m_s * scale;
    output.angular_z_rad_s = raw_angular_z_rad_s;
    output.state = CollisionState::SLOWDOWN;
    return output;
  }

  output.linear_x_m_s = raw_linear_x_m_s;
  output.angular_z_rad_s = raw_angular_z_rad_s;
  output.state = CollisionState::CLEAR;
  return output;
}

}  // namespace hil_autonomy
