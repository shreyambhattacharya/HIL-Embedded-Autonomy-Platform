#include "hil_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void close_to(float actual, float expected)
{
  assert(fabsf(actual - expected) < 1.0e-5F);
}

int main(void)
{
  hil_control_wheel_pair_t pair = hil_control_body_to_wheels(0.3F, 0.5F, 0.2F, 0.8F);
  close_to(pair.left_rad_s, 0.5F);
  close_to(pair.right_rad_s, 2.5F);
  pair = hil_control_body_to_wheels(1.0F, 0.0F, 0.0F, 0.8F);
  close_to(pair.left_rad_s, 0.0F);
  close_to(pair.right_rad_s, 0.0F);
  close_to(hil_control_limited_p_effort(5.0F, 0.0F, 0.5F, 1.5F), 1.5F);
  close_to(hil_control_limited_p_effort(-5.0F, 0.0F, 0.5F, 1.5F), -1.5F);
  close_to(hil_control_limited_p_effort(NAN, 0.0F, 0.5F, 1.5F), 0.0F);
  puts("hil_control tests passed");
  return 0;
}
