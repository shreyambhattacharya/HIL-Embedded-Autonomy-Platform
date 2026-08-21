#include "hil_control_stub/control_math.hpp"

#include <cmath>

#include <gtest/gtest.h>

using hil_control_stub::body_twist_to_wheel_targets;
using hil_control_stub::limited_p_effort;

TEST(ControlMath, ZeroBodyCommandProducesZeroWheelTargets)
{
  const auto targets = body_twist_to_wheel_targets(0.0, 0.0, 0.18, 0.86);
  EXPECT_DOUBLE_EQ(targets.left_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(targets.right_rad_s, 0.0);
}

TEST(ControlMath, ForwardCommandProducesEqualWheelTargets)
{
  const auto targets = body_twist_to_wheel_targets(0.36, 0.0, 0.18, 0.86);
  EXPECT_DOUBLE_EQ(targets.left_rad_s, 2.0);
  EXPECT_DOUBLE_EQ(targets.right_rad_s, 2.0);
}

TEST(ControlMath, PositiveYawIncreasesRightWheelTarget)
{
  const auto targets = body_twist_to_wheel_targets(0.0, 1.0, 0.2, 0.8);
  EXPECT_DOUBLE_EQ(targets.left_rad_s, -2.0);
  EXPECT_DOUBLE_EQ(targets.right_rad_s, 2.0);
}

TEST(ControlMath, InvalidRadiusIsSafe)
{
  const auto targets = body_twist_to_wheel_targets(1.0, 0.0, 0.0, 0.8);
  EXPECT_DOUBLE_EQ(targets.left_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(targets.right_rad_s, 0.0);
}

TEST(ControlMath, EffortIsProportionalAndLimited)
{
  EXPECT_DOUBLE_EQ(limited_p_effort(2.0, 1.0, 0.5, 1.5), 0.5);
  EXPECT_DOUBLE_EQ(limited_p_effort(5.0, 0.0, 0.5, 1.5), 1.5);
  EXPECT_DOUBLE_EQ(limited_p_effort(-5.0, 0.0, 0.5, 1.5), -1.5);
}

TEST(ControlMath, NonFiniteInputCommandsZeroEffort)
{
  EXPECT_DOUBLE_EQ(limited_p_effort(NAN, 0.0, 0.5, 1.5), 0.0);
  EXPECT_DOUBLE_EQ(limited_p_effort(0.0, INFINITY, 0.5, 1.5), 0.0);
}
