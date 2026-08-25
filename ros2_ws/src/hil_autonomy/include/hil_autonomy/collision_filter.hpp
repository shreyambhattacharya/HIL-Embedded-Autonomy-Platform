#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace hil_autonomy
{

enum class CollisionState
{
  CLEAR,
  SLOWDOWN,
  STOP,
  STALE,
  INVALID
};

struct ScanView
{
  double angle_min_rad{0.0};
  double angle_increment_rad{0.0};
  std::vector<float> ranges;
  std::int64_t stamp_ns{0};
};

struct FilterOutput
{
  double linear_x_m_s{0.0};
  double angular_z_rad_s{0.0};
  CollisionState state{CollisionState::INVALID};
  double minimum_front_distance_m{0.0};
};

const char * collision_state_name(CollisionState state);

FilterOutput filter_twist(
  double raw_linear_x_m_s,
  double raw_angular_z_rad_s,
  const ScanView & scan,
  std::int64_t now_ns,
  double scan_timeout_sec,
  double front_sector_half_angle_rad,
  double stop_distance_m,
  double slow_distance_m);

}  // namespace hil_autonomy
