// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_wheel_odometry.cpp
 * @brief Unit tests for differential-drive odometry kinematics.
 *
 * These tests exercise the pure mathematical core of the wheel odometry
 * integration in isolation, without any ROS2 spin loop or message passing.
 * The kinematics are reimplemented here as a thin helper that mirrors exactly
 * what WheelOdometryNode::on_wheel_tick() computes so the logic is validated
 * independently of the ROS2 subscriber wiring.
 *
 * Test cases:
 *   1. straight_line   – equal ticks on both wheels → x advances, y and theta
 *                        remain zero.
 *   2. rotation        – equal and opposite ticks → robot rotates in place,
 *                        x and y remain zero.
 *   3. zero_movement   – zero ticks on both wheels → pose unchanged.
 *   4. left_arc        – more ticks on right than left → arcs left (positive θ).
 *   5. right_arc       – more ticks on left than right → arcs right (negative θ).
 *   6. quarter_circle  – geometry check: robot traces a quarter-circle arc and
 *                        ends up at the expected Cartesian position.
 */

#include <cmath>
#include <tuple>

#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Pure kinematics helper (mirrors WheelOdometryNode internals)
// ---------------------------------------------------------------------------

struct Pose
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};  ///< radians, positive = counter-clockwise
};

/**
 * @brief Apply one odometry step from raw tick deltas.
 *
 * @param pose            Current pose (modified in place — immutable callers
 *                        should pass a copy).
 * @param delta_ticks_l   Signed left-wheel tick increment.
 * @param delta_ticks_r   Signed right-wheel tick increment.
 * @param ticks_per_meter Encoder resolution in ticks/m.
 * @param wheel_distance  Track width in metres.
 * @return Updated pose.
 */
Pose integrate_step(Pose pose,
                    double delta_ticks_l,
                    double delta_ticks_r,
                    double ticks_per_meter,
                    double wheel_distance)
{
  const double d_left = delta_ticks_l / ticks_per_meter;
  const double d_right = delta_ticks_r / ticks_per_meter;
  const double d_center = (d_left + d_right) / 2.0;
  const double d_theta = (d_right - d_left) / wheel_distance;

  pose.x += d_center * std::cos(pose.theta + d_theta / 2.0);
  pose.y += d_center * std::sin(pose.theta + d_theta / 2.0);
  pose.theta += d_theta;
  pose.theta = std::atan2(std::sin(pose.theta), std::cos(pose.theta));

  return pose;
}

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class WheelOdometryKinematicsTest : public ::testing::Test
{
protected:
  static constexpr double kTicksPerMeter = 1000.0;
  static constexpr double kWheelDistance = 0.35;  // metres
  static constexpr double kEpsilon = 1e-9;  // tolerance for doubles

  Pose start_{};  // default-initialised to origin
};

// ---------------------------------------------------------------------------
// Test 1 — straight line
// ---------------------------------------------------------------------------

TEST_F(WheelOdometryKinematicsTest, StraightLine)
{
  // Both wheels rotate the same amount → pure forward translation.
  constexpr double kDistance = 1.0;  // metres
  constexpr double kTicks = kDistance * kTicksPerMeter;

  const Pose result = integrate_step(start_, kTicks, kTicks, kTicksPerMeter, kWheelDistance);

  EXPECT_NEAR(result.x, kDistance, kEpsilon);
  EXPECT_NEAR(result.y, 0.0, kEpsilon);
  EXPECT_NEAR(result.theta, 0.0, kEpsilon);
}

// ---------------------------------------------------------------------------
// Test 2 — pure rotation
// ---------------------------------------------------------------------------

TEST_F(WheelOdometryKinematicsTest, PureRotation)
{
  // Equal and opposite ticks → robot spins about its centre.
  // Arc length for a 90° turn (π/2 rad) = radius × angle
  // For pure rotation, both wheels travel an arc of (wheel_distance/2) × angle.
  constexpr double kAngle = M_PI / 2.0;  // quarter turn CCW
  const double arc = (kWheelDistance / 2.0) * kAngle;
  const double ticks = arc * kTicksPerMeter;

  const Pose result = integrate_step(start_, -ticks, ticks, kTicksPerMeter, kWheelDistance);

  EXPECT_NEAR(result.x, 0.0, kEpsilon);
  EXPECT_NEAR(result.y, 0.0, kEpsilon);
  EXPECT_NEAR(result.theta, kAngle, kEpsilon);
}

// ---------------------------------------------------------------------------
// Test 3 — zero movement
// ---------------------------------------------------------------------------

TEST_F(WheelOdometryKinematicsTest, ZeroMovement)
{
  const Pose result = integrate_step(start_, 0.0, 0.0, kTicksPerMeter, kWheelDistance);

  EXPECT_NEAR(result.x, 0.0, kEpsilon);
  EXPECT_NEAR(result.y, 0.0, kEpsilon);
  EXPECT_NEAR(result.theta, 0.0, kEpsilon);
}

// ---------------------------------------------------------------------------
// Test 4 — arc to the left (more right ticks)
// ---------------------------------------------------------------------------

TEST_F(WheelOdometryKinematicsTest, LeftArc)
{
  // Right wheel travels further → positive d_theta → turns left (CCW).
  constexpr double kLeftTicks = 500.0;
  constexpr double kRightTicks = 1000.0;

  const Pose result =
      integrate_step(start_, kLeftTicks, kRightTicks, kTicksPerMeter, kWheelDistance);

  // Heading should be positive (CCW).
  EXPECT_GT(result.theta, 0.0);
  // Should have moved forward (positive x component before the arc rotates it).
  // The final x depends on the arc geometry but must be > 0.
  EXPECT_GT(result.x, 0.0);
}

// ---------------------------------------------------------------------------
// Test 5 — arc to the right (more left ticks)
// ---------------------------------------------------------------------------

TEST_F(WheelOdometryKinematicsTest, RightArc)
{
  // Left wheel travels further → negative d_theta → turns right (CW).
  constexpr double kLeftTicks = 1000.0;
  constexpr double kRightTicks = 500.0;

  const Pose result =
      integrate_step(start_, kLeftTicks, kRightTicks, kTicksPerMeter, kWheelDistance);

  EXPECT_LT(result.theta, 0.0);
  EXPECT_GT(result.x, 0.0);
}

// ---------------------------------------------------------------------------
// Test 6 — quarter-circle arc geometry
// ---------------------------------------------------------------------------

TEST_F(WheelOdometryKinematicsTest, QuarterCircleArc)
{
  // Robot starts facing +X.  The right wheel is the outer wheel of a left-
  // turning arc with radius R measured at the robot centre-line.
  //
  //   R = 1.0 m (chosen for round numbers)
  //   arc_left  = (R - wheel_distance/2) × (π/2)
  //   arc_right = (R + wheel_distance/2) × (π/2)
  //
  // After a quarter circle the robot should be at:
  //   x = R × sin(π/2) = R = 1.0
  //   y = R × (1 - cos(π/2)) = R = 1.0
  //   theta = π/2

  constexpr double kRadius = 1.0;
  constexpr double kAngle = M_PI / 2.0;

  const double arc_left = (kRadius - kWheelDistance / 2.0) * kAngle;
  const double arc_right = (kRadius + kWheelDistance / 2.0) * kAngle;

  // Integrate in many small steps for numerical accuracy.
  constexpr int kSteps = 1000;
  const double dt_l = (arc_left * kTicksPerMeter) / kSteps;
  const double dt_r = (arc_right * kTicksPerMeter) / kSteps;

  Pose pose = start_;
  for (int i = 0; i < kSteps; ++i)
  {
    pose = integrate_step(pose, dt_l, dt_r, kTicksPerMeter, kWheelDistance);
  }

  constexpr double kTolerance = 1e-4;  // 0.1 mm — tight enough for 1000-step integration
  EXPECT_NEAR(pose.x, kRadius, kTolerance);
  EXPECT_NEAR(pose.y, kRadius, kTolerance);
  EXPECT_NEAR(pose.theta, kAngle, kTolerance);
}

// ---------------------------------------------------------------------------
// Test 7 — reversing straight line
// ---------------------------------------------------------------------------

TEST_F(WheelOdometryKinematicsTest, ReverseStraightLine)
{
  constexpr double kDistance = 1.0;
  constexpr double kTicks = -(kDistance * kTicksPerMeter);  // negative = backward

  const Pose result = integrate_step(start_, kTicks, kTicks, kTicksPerMeter, kWheelDistance);

  EXPECT_NEAR(result.x, -kDistance, kEpsilon);
  EXPECT_NEAR(result.y, 0.0, kEpsilon);
  EXPECT_NEAR(result.theta, 0.0, kEpsilon);
}
