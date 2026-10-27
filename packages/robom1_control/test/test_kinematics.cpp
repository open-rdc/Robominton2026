#include <gtest/gtest.h>
#include "robom1_control/kinematics.hpp"
using namespace robom1_control;

TEST(Swerve, ReconstructsVelocityAtEveryWheel)
{
  for (double x : {-0.26, 0.26}) {
    for (double y : {-0.24, 0.24}) {
      for (double previous : {-3.0, 0.0, 2.0}) {
        auto w = inverse(0.7, -0.4, 0.8, x, y, 0.09, previous);
        EXPECT_NEAR(0.09 * w.velocity * std::cos(w.steering), 0.7 - 0.8 * y, 1e-10);
        EXPECT_NEAR(0.09 * w.velocity * std::sin(w.steering), -0.4 + 0.8 * x, 1e-10);
        EXPECT_LE(std::abs(w.steering - previous), pi / 2 + 1e-10);
      }
    }
  }
}
TEST(Swerve, StopRetainsSteeringAndReverseAvoidsHalfTurn)
{
  EXPECT_DOUBLE_EQ(inverse(0, 0, 0, 0.26, 0.24, 0.09, 0.7).steering, 0.7);
  auto w = inverse(-1, 0, 0, 0.26, 0.24, 0.1, 0);
  EXPECT_NEAR(w.steering, 0, 1e-10);
  EXPECT_NEAR(w.velocity, -10, 1e-10);
}
TEST(Odometry, BodyTranslationAndCircularMotion)
{
  double x = 0, y = 0, yaw = pi / 2;
  integrate(x, y, yaw, 1, 0, 0, 1);
  EXPECT_NEAR(x, 0, 1e-10); EXPECT_NEAR(y, 1, 1e-10);
  x = y = yaw = 0;
  integrate(x, y, yaw, 1, 0, 1, pi / 2);
  EXPECT_NEAR(x, 1, 1e-10); EXPECT_NEAR(y, 1, 1e-10);
}

TEST(Swerve, WheelLimitsScaleTheCompleteBodyTwist)
{
  const double scale = drive_scale(2.0, -1.0, 2.0, 0.68, 0.64, 0.12, 8.0);
  EXPECT_GT(scale, 0); EXPECT_LT(scale, 1);
  double largest = 0;
  for (double x : {-0.34, 0.34}) {
    for (double y : {-0.32, 0.32}) {
      auto wheel = inverse(2*scale, -scale, 2*scale, x, y, 0.12, 0);
      largest = std::max(largest, std::abs(wheel.velocity));
    }
  }
  EXPECT_NEAR(largest, 8.0, 1e-10);
  EXPECT_DOUBLE_EQ(drive_scale(0, 0, 0, .68, .64, .12, 8), 1.0);
}
