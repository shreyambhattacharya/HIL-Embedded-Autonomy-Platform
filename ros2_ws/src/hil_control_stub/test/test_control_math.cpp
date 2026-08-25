#include "hil_control_stub/control_math.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using hil_control_stub::body_twist_to_wheel_targets;
using hil_control_stub::limited_p_effort;

TEST(ControlMath, ZeroBodyCommandProducesZeroWheelTargets)
{
  const auto targets = body_twist_to_wheel_targets(0.0, 0.0, 0.18, 0.86);
  EXPECT_DOUBLE_EQ(targets.left_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(targets.right_rad_s, 0.0);
}

TEST(ControlMath, ForwardCommandProducesEqualPositiveTargets)
{
  const auto targets = body_twist_to_wheel_targets(0.36, 0.0, 0.18, 0.86);
  EXPECT_DOUBLE_EQ(targets.left_rad_s, 2.0);
  EXPECT_DOUBLE_EQ(targets.right_rad_s, 2.0);
}

TEST(ControlMath, ReverseCommandProducesEqualNegativeTargets)
{
  const auto targets = body_twist_to_wheel_targets(-0.36, 0.0, 0.18, 0.86);
  EXPECT_DOUBLE_EQ(targets.left_rad_s, -2.0);
  EXPECT_DOUBLE_EQ(targets.right_rad_s, -2.0);
}

TEST(ControlMath, PositiveYawReversesLeftAndAdvancesRight)
{
  const auto targets = body_twist_to_wheel_targets(0.0, 1.0, 0.2, 0.8);
  EXPECT_DOUBLE_EQ(targets.left_rad_s, -2.0);
  EXPECT_DOUBLE_EQ(targets.right_rad_s, 2.0);
}

TEST(ControlMath, NegativeYawAdvancesLeftAndReversesRight)
{
  const auto targets = body_twist_to_wheel_targets(0.0, -1.0, 0.2, 0.8);
  EXPECT_DOUBLE_EQ(targets.left_rad_s, 2.0);
  EXPECT_DOUBLE_EQ(targets.right_rad_s, -2.0);
}

TEST(ControlMath, CombinedForwardYawProducesExpectedTargets)
{
  const auto targets = body_twist_to_wheel_targets(0.3, 0.5, 0.2, 0.8);
  EXPECT_DOUBLE_EQ(targets.left_rad_s, 0.5);
  EXPECT_DOUBLE_EQ(targets.right_rad_s, 2.5);
}

TEST(ControlMath, InvalidOrNonFiniteKinematicsInputIsSafe)
{
  const auto invalid_radius = body_twist_to_wheel_targets(1.0, 0.0, 0.0, 0.8);
  const auto invalid_separation = body_twist_to_wheel_targets(1.0, 0.0, 0.2, -0.8);
  const auto nonfinite = body_twist_to_wheel_targets(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.2, 0.8);
  EXPECT_DOUBLE_EQ(invalid_radius.left_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(invalid_radius.right_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(invalid_separation.left_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(invalid_separation.right_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(nonfinite.left_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(nonfinite.right_rad_s, 0.0);
}

TEST(ControlMath, ZeroWheelSpeedErrorProducesZeroEffort)
{
  EXPECT_DOUBLE_EQ(limited_p_effort(2.0, 2.0, 0.5, 1.5), 0.0);
}

TEST(ControlMath, PositiveAndNegativeErrorsProduceSignedEffort)
{
  EXPECT_DOUBLE_EQ(limited_p_effort(2.0, 1.0, 0.5, 1.5), 0.5);
  EXPECT_DOUBLE_EQ(limited_p_effort(1.0, 2.0, 0.5, 1.5), -0.5);
}

TEST(ControlMath, EffortSaturatesAtBothLimits)
{
  EXPECT_DOUBLE_EQ(limited_p_effort(5.0, 0.0, 0.5, 1.5), 1.5);
  EXPECT_DOUBLE_EQ(limited_p_effort(-5.0, 0.0, 0.5, 1.5), -1.5);
}

TEST(ControlMath, NonFiniteEffortInputCommandsZero)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(limited_p_effort(nan, 0.0, 0.5, 1.5), 0.0);
  EXPECT_DOUBLE_EQ(limited_p_effort(0.0, infinity, 0.5, 1.5), 0.0);
  EXPECT_DOUBLE_EQ(limited_p_effort(0.0, 0.0, nan, 1.5), 0.0);
  EXPECT_DOUBLE_EQ(limited_p_effort(0.0, 0.0, 0.5, infinity), 0.0);
}
