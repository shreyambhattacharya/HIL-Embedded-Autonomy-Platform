#ifndef HIL_CONTROL_H
#define HIL_CONTROL_H

#include <stdbool.h>

typedef struct {
  float left_rad_s;
  float right_rad_s;
} hil_control_wheel_pair_t;

bool hil_control_finite(float value);
hil_control_wheel_pair_t hil_control_body_to_wheels(
  float linear_m_s, float yaw_rad_s, float wheel_radius_m, float wheel_separation_m);
float hil_control_limited_p_effort(
  float target_rad_s, float measured_rad_s, float kp_nm_per_rad_s, float max_effort_nm);

#endif
