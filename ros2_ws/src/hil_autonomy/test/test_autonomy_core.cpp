#include "hil_autonomy/collision_filter.hpp"
#include "hil_autonomy/state_estimator.hpp"
#include "hil_autonomy/waypoint_controller.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace
{

TEST(StateEstimator, IntegratesStraightLineAndRequiresBothSensors)
{
  hil_autonomy::StateEstimator estimator(0.18, 0.86, 0.25, 0.3, 0.2);
  EXPECT_TRUE(estimator.update_wheels({0.0, 0.0, 1.0, 1.0, 1000000000}));
  EXPECT_TRUE(estimator.update_imu({0.0, 1000000000}));
  EXPECT_TRUE(estimator.update_wheels({1.0, 1.0, 1.0, 1.0, 1100000000}));
  const auto state = estimator.state(1200000000);
  EXPECT_TRUE(state.valid);
  EXPECT_NEAR(state.x_m, 0.18, 1.0e-9);
  EXPECT_NEAR(state.y_m, 0.0, 1.0e-9);
  EXPECT_NEAR(state.yaw_rad, 0.0, 1.0e-9);
}

TEST(StateEstimator, MarksSamplesStale)
{
  hil_autonomy::StateEstimator estimator(0.18, 0.86, 0.25, 0.3, 0.2);
  estimator.update_imu({0.0, 1000000000});
  estimator.update_wheels({0.0, 0.0, 0.0, 0.0, 1000000000});
  EXPECT_FALSE(estimator.state(1400000001).valid);
}

TEST(WaypointController, StartsTurnsAndCompletes)
{
  hil_autonomy::WaypointController controller(
    {{1.0, 0.0}, {1.0, 1.0}, {0.0, 0.0}}, 0.05, 2.0, 1.0, 0.3, 0.8, 0.8);
  hil_autonomy::EstimateState estimate;
  estimate.valid = true;
  controller.start();
  auto output = controller.compute(estimate);
  EXPECT_GT(output.command.linear_x_m_s, 0.0);
  estimate.x_m = 1.0;
  output = controller.compute(estimate);
  EXPECT_EQ(output.waypoint_index, 1U);
  output = controller.compute(estimate);
  EXPECT_GT(std::abs(output.command.angular_z_rad_s), 0.0);
  estimate.yaw_rad = 1.5707963267948966;
  estimate.y_m = 0.8;
  output = controller.compute(estimate);
  EXPECT_GT(output.command.linear_x_m_s, 0.0);
  estimate.x_m = 1.0;
  estimate.y_m = 1.0;
  output = controller.compute(estimate);
  EXPECT_EQ(output.waypoint_index, 2U);
  estimate.x_m = 0.0;
  estimate.y_m = 0.0;
  estimate.yaw_rad = 0.0;
  output = controller.compute(estimate);
  EXPECT_TRUE(output.complete);
  EXPECT_FALSE(controller.active());
}

TEST(CollisionFilter, IgnoresInfiniteRangesAndStopsOnFiniteObstacle)
{
  hil_autonomy::ScanView scan;
  scan.angle_min_rad = -1.0;
  scan.angle_increment_rad = 1.0;
  scan.ranges = {std::numeric_limits<float>::infinity(), 2.0F, 0.3F};
  scan.stamp_ns = 1000000000;
  auto output = hil_autonomy::filter_twist(0.3, 0.2, scan, 1100000000, 0.5, 1.0, 0.5, 1.2);
  EXPECT_EQ(output.state, hil_autonomy::CollisionState::STOP);
  EXPECT_DOUBLE_EQ(output.linear_x_m_s, 0.0);
  scan.ranges[2] = 0.8F;
  output = hil_autonomy::filter_twist(0.3, 0.2, scan, 1100000000, 0.5, 1.0, 0.5, 1.2);
  EXPECT_EQ(output.state, hil_autonomy::CollisionState::SLOWDOWN);
  EXPECT_LT(output.linear_x_m_s, 0.3);
}

TEST(CollisionFilter, StaleAndInvalidScansCommandZero)
{
  hil_autonomy::ScanView scan;
  scan.angle_min_rad = -1.0;
  scan.angle_increment_rad = 1.0;
  scan.ranges = {1.0F, 1.0F, 1.0F};
  scan.stamp_ns = 1000000000;
  auto output = hil_autonomy::filter_twist(0.3, 0.2, scan, 2000000000, 0.5, 1.0, 0.5, 1.2);
  EXPECT_EQ(output.state, hil_autonomy::CollisionState::STALE);
  scan.stamp_ns = 2000000000;
  scan.ranges[1] = std::numeric_limits<float>::quiet_NaN();
  scan.ranges[0] = std::numeric_limits<float>::quiet_NaN();
  scan.ranges[2] = std::numeric_limits<float>::infinity();
  output = hil_autonomy::filter_twist(0.3, 0.2, scan, 2000000000, 0.5, 1.0, 0.5, 1.2);
  EXPECT_EQ(output.state, hil_autonomy::CollisionState::CLEAR);
  EXPECT_DOUBLE_EQ(output.linear_x_m_s, 0.3);
  scan.ranges[2] = -std::numeric_limits<float>::infinity();
  output = hil_autonomy::filter_twist(0.3, 0.2, scan, 2000000000, 0.5, 1.0, 0.5, 1.2);
  EXPECT_EQ(output.state, hil_autonomy::CollisionState::INVALID);
  EXPECT_DOUBLE_EQ(output.linear_x_m_s, 0.0);
}

}  // namespace
