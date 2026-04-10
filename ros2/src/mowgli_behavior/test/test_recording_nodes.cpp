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
 * @file test_recording_nodes.cpp
 * @brief Unit tests for RecordArea pure algorithm methods.
 *
 * Tests the static helper functions (perpendicular_distance, polygon_area,
 * douglas_peucker) in isolation without any BT tick() or ROS2 spin.
 * The test class is declared as a friend of RecordArea to access the private
 * static methods directly.
 */

#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point32.hpp"
#include "mowgli_behavior/recording_nodes.hpp"
#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Global ROS2 init/shutdown
// ---------------------------------------------------------------------------

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------

static geometry_msgs::msg::Point32 make_point(float x, float y)
{
  geometry_msgs::msg::Point32 pt;
  pt.x = x;
  pt.y = y;
  pt.z = 0.0f;
  return pt;
}

// ---------------------------------------------------------------------------
// Test fixture — friend of RecordArea so we can call private static methods
// ---------------------------------------------------------------------------

class RecordAreaAlgorithmTest : public ::testing::Test
{
protected:
  // Wrappers that forward to RecordArea's private static methods.

  static double perpendicular_distance(const geometry_msgs::msg::Point32& pt,
                                       const geometry_msgs::msg::Point32& line_start,
                                       const geometry_msgs::msg::Point32& line_end)
  {
    return mowgli_behavior::RecordArea::perpendicular_distance(pt, line_start, line_end);
  }

  static double polygon_area(const std::vector<geometry_msgs::msg::Point32>& points)
  {
    return mowgli_behavior::RecordArea::polygon_area(points);
  }

  static std::vector<geometry_msgs::msg::Point32> douglas_peucker(
      const std::vector<geometry_msgs::msg::Point32>& points, double tolerance)
  {
    return mowgli_behavior::RecordArea::douglas_peucker(points, tolerance);
  }
};

// ===========================================================================
// perpendicular_distance tests
// ===========================================================================

TEST_F(RecordAreaAlgorithmTest, PerpendicularDistance_PointOnLine)
{
  // Midpoint of line from (0,0) to (10,0) is (5,0) — distance should be 0
  auto pt = make_point(5.0f, 0.0f);
  auto start = make_point(0.0f, 0.0f);
  auto end = make_point(10.0f, 0.0f);
  EXPECT_NEAR(perpendicular_distance(pt, start, end), 0.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PerpendicularDistance_PerpendicularToMidpoint)
{
  // Point (5, 3) is 3 units above the line from (0,0) to (10,0)
  auto pt = make_point(5.0f, 3.0f);
  auto start = make_point(0.0f, 0.0f);
  auto end = make_point(10.0f, 0.0f);
  EXPECT_NEAR(perpendicular_distance(pt, start, end), 3.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PerpendicularDistance_DegenerateLine)
{
  // When start == end, distance should be Euclidean distance to the point
  auto pt = make_point(3.0f, 4.0f);
  auto start = make_point(0.0f, 0.0f);
  auto end = make_point(0.0f, 0.0f);
  EXPECT_NEAR(perpendicular_distance(pt, start, end), 5.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PerpendicularDistance_PointPastEndpoints)
{
  // Point (15, 3) is past the end of line from (0,0) to (10,0).
  // The perpendicular distance to the *infinite line* is still 3.
  auto pt = make_point(15.0f, 3.0f);
  auto start = make_point(0.0f, 0.0f);
  auto end = make_point(10.0f, 0.0f);
  EXPECT_NEAR(perpendicular_distance(pt, start, end), 3.0, 1e-6);
}

// ===========================================================================
// polygon_area tests
// ===========================================================================

TEST_F(RecordAreaAlgorithmTest, PolygonArea_UnitSquare)
{
  // Counter-clockwise unit square
  std::vector<geometry_msgs::msg::Point32> square = {
      make_point(0.0f, 0.0f),
      make_point(1.0f, 0.0f),
      make_point(1.0f, 1.0f),
      make_point(0.0f, 1.0f),
  };
  EXPECT_NEAR(polygon_area(square), 1.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PolygonArea_Triangle)
{
  // Triangle with vertices (0,0), (4,0), (0,3) — area = 0.5 * 4 * 3 = 6
  std::vector<geometry_msgs::msg::Point32> triangle = {
      make_point(0.0f, 0.0f),
      make_point(4.0f, 0.0f),
      make_point(0.0f, 3.0f),
  };
  EXPECT_NEAR(polygon_area(triangle), 6.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PolygonArea_LargerRectangle)
{
  // 5 x 3 rectangle = area 15
  std::vector<geometry_msgs::msg::Point32> rect = {
      make_point(0.0f, 0.0f),
      make_point(5.0f, 0.0f),
      make_point(5.0f, 3.0f),
      make_point(0.0f, 3.0f),
  };
  EXPECT_NEAR(polygon_area(rect), 15.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PolygonArea_LessThan3Points)
{
  // 0 points
  EXPECT_NEAR(polygon_area({}), 0.0, 1e-6);

  // 1 point
  std::vector<geometry_msgs::msg::Point32> one = {make_point(1.0f, 2.0f)};
  EXPECT_NEAR(polygon_area(one), 0.0, 1e-6);

  // 2 points
  std::vector<geometry_msgs::msg::Point32> two = {
      make_point(0.0f, 0.0f),
      make_point(1.0f, 1.0f),
  };
  EXPECT_NEAR(polygon_area(two), 0.0, 1e-6);
}

TEST_F(RecordAreaAlgorithmTest, PolygonArea_ClockwiseVsCounterclockwise)
{
  // CCW unit square
  std::vector<geometry_msgs::msg::Point32> ccw = {
      make_point(0.0f, 0.0f),
      make_point(1.0f, 0.0f),
      make_point(1.0f, 1.0f),
      make_point(0.0f, 1.0f),
  };

  // CW unit square (reversed winding)
  std::vector<geometry_msgs::msg::Point32> cw = {
      make_point(0.0f, 0.0f),
      make_point(0.0f, 1.0f),
      make_point(1.0f, 1.0f),
      make_point(1.0f, 0.0f),
  };

  EXPECT_NEAR(polygon_area(ccw), polygon_area(cw), 1e-6);
  EXPECT_NEAR(polygon_area(ccw), 1.0, 1e-6);
}

// ===========================================================================
// douglas_peucker tests
// ===========================================================================

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_StraightLine)
{
  // Collinear points along the x-axis — should simplify to just endpoints
  std::vector<geometry_msgs::msg::Point32> line;
  for (int i = 0; i <= 10; ++i)
  {
    line.push_back(make_point(static_cast<float>(i), 0.0f));
  }

  auto result = douglas_peucker(line, 0.1);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_FLOAT_EQ(result.front().x, 0.0f);
  EXPECT_FLOAT_EQ(result.back().x, 10.0f);
}

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_LShapedPath)
{
  // L-shape: (0,0) -> (5,0) -> (5,5) with intermediate collinear points
  std::vector<geometry_msgs::msg::Point32> path = {
      make_point(0.0f, 0.0f),
      make_point(1.0f, 0.0f),
      make_point(2.0f, 0.0f),
      make_point(3.0f, 0.0f),
      make_point(4.0f, 0.0f),
      make_point(5.0f, 0.0f),
      make_point(5.0f, 1.0f),
      make_point(5.0f, 2.0f),
      make_point(5.0f, 3.0f),
      make_point(5.0f, 4.0f),
      make_point(5.0f, 5.0f),
  };

  auto result = douglas_peucker(path, 0.1);
  // Should keep at least 3 points: start, corner, end
  ASSERT_GE(result.size(), 3u);

  // First and last must be preserved
  EXPECT_FLOAT_EQ(result.front().x, 0.0f);
  EXPECT_FLOAT_EQ(result.front().y, 0.0f);
  EXPECT_FLOAT_EQ(result.back().x, 5.0f);
  EXPECT_FLOAT_EQ(result.back().y, 5.0f);

  // The corner point (5,0) must be retained
  bool corner_found = false;
  for (const auto& pt : result)
  {
    if (std::abs(pt.x - 5.0f) < 1e-3f && std::abs(pt.y - 0.0f) < 1e-3f)
    {
      corner_found = true;
      break;
    }
  }
  EXPECT_TRUE(corner_found) << "Corner point (5,0) should be retained";
}

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_LessThan3Points)
{
  // 0 points
  auto r0 = douglas_peucker({}, 0.1);
  EXPECT_TRUE(r0.empty());

  // 1 point
  std::vector<geometry_msgs::msg::Point32> one = {make_point(1.0f, 2.0f)};
  auto r1 = douglas_peucker(one, 0.1);
  ASSERT_EQ(r1.size(), 1u);
  EXPECT_FLOAT_EQ(r1[0].x, 1.0f);

  // 2 points
  std::vector<geometry_msgs::msg::Point32> two = {
      make_point(0.0f, 0.0f),
      make_point(5.0f, 5.0f),
  };
  auto r2 = douglas_peucker(two, 0.1);
  ASSERT_EQ(r2.size(), 2u);
}

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_HighTolerance)
{
  // L-shape with very high tolerance — should aggressively simplify to 2 endpoints
  std::vector<geometry_msgs::msg::Point32> path = {
      make_point(0.0f, 0.0f),
      make_point(5.0f, 0.0f),
      make_point(5.0f, 5.0f),
  };

  auto result = douglas_peucker(path, 100.0);
  // With tolerance larger than the corner deviation, only endpoints are kept
  EXPECT_EQ(result.size(), 2u);
}

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_ZeroTolerance)
{
  // With zero tolerance, every point must be kept
  std::vector<geometry_msgs::msg::Point32> path;
  for (int i = 0; i <= 5; ++i)
  {
    // Slight zigzag so each point deviates from the line between its neighbors
    float y = (i % 2 == 0) ? 0.0f : 0.01f;
    path.push_back(make_point(static_cast<float>(i), y));
  }

  auto result = douglas_peucker(path, 0.0);
  EXPECT_EQ(result.size(), path.size());
}

TEST_F(RecordAreaAlgorithmTest, DouglasPeucker_CircleLikeShape)
{
  // Approximate a circle with 36 points (every 10 degrees)
  const float radius = 5.0f;
  const int n_points = 36;
  std::vector<geometry_msgs::msg::Point32> circle;
  for (int i = 0; i < n_points; ++i)
  {
    float angle =
        static_cast<float>(i) * 2.0f * static_cast<float>(M_PI) / static_cast<float>(n_points);
    circle.push_back(make_point(radius * std::cos(angle), radius * std::sin(angle)));
  }

  // Moderate simplification — should retain some points but fewer than original
  auto result = douglas_peucker(circle, 0.5);
  EXPECT_GT(result.size(), 4u) << "Circle should retain enough points for shape";
  EXPECT_LT(result.size(), circle.size()) << "Simplification should reduce point count";

  // Verify first and last points are preserved
  EXPECT_FLOAT_EQ(result.front().x, circle.front().x);
  EXPECT_FLOAT_EQ(result.front().y, circle.front().y);
  EXPECT_FLOAT_EQ(result.back().x, circle.back().x);
  EXPECT_FLOAT_EQ(result.back().y, circle.back().y);
}
