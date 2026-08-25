#include "hil_autonomy/waypoint_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hil_autonomy
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}
}  // namespace

WaypointController::WaypointController(
  std::vector<Waypoint> waypoints,
  const double waypoint_tolerance_m,
  const double heading_kp,
  const double linear_kp,
  const double max_linear_m_s,
  const double max_angular_rad_s,
  const double turn_in_place_error_rad)
: waypoints_(std::move(waypoints)),
  waypoint_tolerance_m_(waypoint_tolerance_m),
  heading_kp_(heading_kp),
  linear_kp_(linear_kp),
  max_linear_m_s_(max_linear_m_s),
  max_angular_rad_s_(max_angular_rad_s),
  turn_in_place_error_rad_(turn_in_place_error_rad)
{
  if (waypoints_.empty() || !std::isfinite(waypoint_tolerance_m_) || waypoint_tolerance_m_ <= 0.0 ||
    !std::isfinite(heading_kp_) || heading_kp_ < 0.0 ||
    !std::isfinite(linear_kp_) || linear_kp_ <= 0.0 ||
    !std::isfinite(max_linear_m_s_) || max_linear_m_s_ <= 0.0 ||
    !std::isfinite(max_angular_rad_s_) || max_angular_rad_s_ <= 0.0 ||
    !std::isfinite(turn_in_place_error_rad_) || turn_in_place_error_rad_ <= 0.0 ||
    turn_in_place_error_rad_ > kPi)
  {
    throw std::invalid_argument("invalid waypoint controller configuration");
  }
  for (const auto & waypoint : waypoints_) {
    if (!std::isfinite(waypoint.x_m) || !std::isfinite(waypoint.y_m)) {
      throw std::invalid_argument("waypoint is not finite");
    }
  }
}

void WaypointController::start()
{
  active_ = true;
  complete_ = false;
  waypoint_index_ = 0U;
}

void WaypointController::stop()
{
  active_ = false;
}

WaypointOutput WaypointController::compute(const EstimateState & estimate)
{
  WaypointOutput output;
  output.active = active_;
  output.complete = complete_;
  output.waypoint_index = waypoint_index_;
  if (!active_ || complete_ || !estimate.valid) {
    return output;
  }

  const auto & waypoint = waypoints_[waypoint_index_];
  const double dx = waypoint.x_m - estimate.x_m;
  const double dy = waypoint.y_m - estimate.y_m;
  const double distance = std::hypot(dx, dy);
  output.distance_to_waypoint_m = distance;
  if (distance <= waypoint_tolerance_m_) {
    if (waypoint_index_ + 1U >= waypoints_.size()) {
      complete_ = true;
      active_ = false;
      output.active = false;
      output.complete = true;
      return output;
    }
    ++waypoint_index_;
    output.waypoint_index = waypoint_index_;
    output.distance_to_waypoint_m = 0.0;
    return output;
  }

  const double desired_heading = std::atan2(dy, dx);
  const double heading_error = wrap_angle(desired_heading - estimate.yaw_rad);
  output.command.angular_z_rad_s = clamp(
    heading_kp_ * heading_error, -max_angular_rad_s_, max_angular_rad_s_);
  output.command.linear_x_m_s = clamp(linear_kp_ * distance, 0.0, max_linear_m_s_);
  if (std::abs(heading_error) > turn_in_place_error_rad_) {
    output.command.linear_x_m_s = 0.0;
  } else {
    output.command.linear_x_m_s *= clamp(
      std::cos(heading_error), 0.0, 1.0);
  }
  return output;
}

}  // namespace hil_autonomy
