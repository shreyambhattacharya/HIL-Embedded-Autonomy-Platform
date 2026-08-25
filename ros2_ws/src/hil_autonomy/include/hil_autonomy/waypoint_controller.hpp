#pragma once

#include <cstddef>
#include <vector>

#include "hil_autonomy/state_estimator.hpp"

namespace hil_autonomy
{

struct Waypoint
{
  double x_m{0.0};
  double y_m{0.0};
};

struct TwistCommand
{
  double linear_x_m_s{0.0};
  double angular_z_rad_s{0.0};
};

struct WaypointOutput
{
  TwistCommand command{};
  bool active{false};
  bool complete{false};
  std::size_t waypoint_index{0U};
  double distance_to_waypoint_m{0.0};
};

class WaypointController final
{
public:
  WaypointController(
    std::vector<Waypoint> waypoints,
    double waypoint_tolerance_m,
    double heading_kp,
    double linear_kp,
    double max_linear_m_s,
    double max_angular_rad_s,
    double turn_in_place_error_rad);

  void start();
  void stop();
  WaypointOutput compute(const EstimateState & estimate);
  bool active() const {return active_;}
  bool complete() const {return complete_;}
  std::size_t waypoint_index() const {return waypoint_index_;}

private:
  std::vector<Waypoint> waypoints_;
  double waypoint_tolerance_m_;
  double heading_kp_;
  double linear_kp_;
  double max_linear_m_s_;
  double max_angular_rad_s_;
  double turn_in_place_error_rad_;
  bool active_{false};
  bool complete_{false};
  std::size_t waypoint_index_{0U};
};

}  // namespace hil_autonomy
