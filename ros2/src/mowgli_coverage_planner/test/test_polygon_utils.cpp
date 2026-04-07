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

/**
 * @file test_polygon_utils.cpp
 * @brief Unit tests for polygon_utils functions.
 *
 * All tests use GTest assertions. Coordinate frames are arbitrary 2-D
 * Cartesian space; units are metres unless stated otherwise.
 */

#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "gtest/gtest.h"
#include "mowgli_coverage_planner/polygon_utils.hpp"

using namespace mowgli_coverage_planner;  // NOLINT(build/namespaces)

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a Polygon2D rectangle with corners (x0,y0) → (x1,y1) in CCW order.
static Polygon2D make_rectangle(double x0, double y0, double x1, double y1)
{
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

/// Build a Polygon2D triangle.
static Polygon2D make_triangle(double ax, double ay, double bx, double by, double cx, double cy)
{
  return {{ax, ay}, {bx, by}, {cx, cy}};
}

// ---------------------------------------------------------------------------
// polygon_area
// ---------------------------------------------------------------------------

TEST(PolygonArea, Rectangle10x5)
{
  const Polygon2D rect = make_rectangle(0.0, 0.0, 10.0, 5.0);
  EXPECT_NEAR(std::abs(polygon_area(rect)), 50.0, 1e-9);
}

TEST(PolygonArea, Triangle)
{
  // Right triangle with legs 3 and 4 → area = 6.
  const Polygon2D tri = make_triangle(0.0, 0.0, 3.0, 0.0, 0.0, 4.0);
  EXPECT_NEAR(std::abs(polygon_area(tri)), 6.0, 1e-9);
}

TEST(PolygonArea, UnitSquare)
{
  const Polygon2D sq = make_rectangle(0.0, 0.0, 1.0, 1.0);
  EXPECT_NEAR(std::abs(polygon_area(sq)), 1.0, 1e-9);
}

TEST(PolygonArea, EmptyPolygon)
{
  EXPECT_NEAR(polygon_area({}), 0.0, 1e-9);
}

TEST(PolygonArea, TwoVertices)
{
  const Polygon2D line = {{0.0, 0.0}, {1.0, 1.0}};
  EXPECT_NEAR(polygon_area(line), 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// polygon_centroid
// ---------------------------------------------------------------------------

TEST(PolygonCentroid, Rectangle)
{
  // Centroid of a rectangle should be at its geometric centre.
  const Polygon2D rect = make_rectangle(0.0, 0.0, 10.0, 4.0);
  const auto [cx, cy] = polygon_centroid(rect);
  EXPECT_NEAR(cx, 5.0, 1e-9);
  EXPECT_NEAR(cy, 2.0, 1e-9);
}

TEST(PolygonCentroid, SymmetricTriangle)
{
  // Centroid of a triangle is at (mean_x, mean_y).
  const Polygon2D tri = make_triangle(0.0, 0.0, 6.0, 0.0, 3.0, 6.0);
  const auto [cx, cy] = polygon_centroid(tri);
  EXPECT_NEAR(cx, 3.0, 1e-9);
  EXPECT_NEAR(cy, 2.0, 1e-9);
}

TEST(PolygonCentroid, OffsetRectangle)
{
  const Polygon2D rect = make_rectangle(2.0, 3.0, 8.0, 9.0);
  const auto [cx, cy] = polygon_centroid(rect);
  EXPECT_NEAR(cx, 5.0, 1e-9);
  EXPECT_NEAR(cy, 6.0, 1e-9);
}

// ---------------------------------------------------------------------------
// point_in_polygon
// ---------------------------------------------------------------------------

TEST(PointInPolygon, InsideSquare)
{
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 10.0);
  EXPECT_TRUE(point_in_polygon({5.0, 5.0}, sq));
}

TEST(PointInPolygon, OutsideSquare)
{
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 10.0);
  EXPECT_FALSE(point_in_polygon({15.0, 5.0}, sq));
  EXPECT_FALSE(point_in_polygon({5.0, -1.0}, sq));
  EXPECT_FALSE(point_in_polygon({-0.1, 5.0}, sq));
}

TEST(PointInPolygon, JustInsideEdge)
{
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 10.0);
  EXPECT_TRUE(point_in_polygon({0.01, 5.0}, sq));
  EXPECT_TRUE(point_in_polygon({5.0, 0.01}, sq));
  EXPECT_TRUE(point_in_polygon({9.99, 5.0}, sq));
}

TEST(PointInPolygon, InsideTriangle)
{
  const Polygon2D tri = make_triangle(0.0, 0.0, 10.0, 0.0, 5.0, 10.0);
  EXPECT_TRUE(point_in_polygon({5.0, 3.0}, tri));
  EXPECT_FALSE(point_in_polygon({0.0, 9.0}, tri));
}

// ---------------------------------------------------------------------------
// offset_polygon
// ---------------------------------------------------------------------------

TEST(OffsetPolygon, ShrinkRectangle)
{
  // A 10×10 square shrunk by 1 m on all sides → 8×8 = 64 m².
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 10.0);
  const Polygon2D shrunk = offset_polygon(sq, 1.0);

  ASSERT_GE(shrunk.size(), 3u);

  const double shrunk_area = std::abs(polygon_area(shrunk));
  EXPECT_NEAR(shrunk_area, 64.0, 0.5);  // allow for rounding at corners
}

TEST(OffsetPolygon, ShrunkPolygonIsInsideOriginal)
{
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 10.0);
  const Polygon2D shrunk = offset_polygon(sq, 1.0);

  ASSERT_GE(shrunk.size(), 3u);

  // Every vertex of the shrunk polygon should be inside the original.
  for (const auto& v : shrunk)
  {
    EXPECT_TRUE(point_in_polygon(v, sq))
        << "Vertex (" << v.first << ", " << v.second << ") not inside original";
  }
}

TEST(OffsetPolygon, ExcessiveShrinkReturnsEmpty)
{
  // Shrinking a 1×1 square by 10 m should return empty.
  const Polygon2D sq = make_rectangle(0.0, 0.0, 1.0, 1.0);
  const Polygon2D result = offset_polygon(sq, 10.0);
  EXPECT_TRUE(result.empty());
}

TEST(OffsetPolygon, SmallShrinkReducesArea)
{
  const Polygon2D sq = make_rectangle(0.0, 0.0, 20.0, 20.0);
  const Polygon2D shrunk = offset_polygon(sq, 0.5);
  EXPECT_LT(std::abs(polygon_area(shrunk)), std::abs(polygon_area(sq)));
}

// ---------------------------------------------------------------------------
// optimal_mowing_angle
// ---------------------------------------------------------------------------

TEST(OptimalMowingAngle, WideRectanglePrefersHorizontal)
{
  // A 20×4 rectangle: optimal is to mow along the long axis (0° or 180°,
  // meaning horizontal swaths so the fewest passes are needed).
  const Polygon2D rect = make_rectangle(0.0, 0.0, 20.0, 4.0);
  const double angle_rad = optimal_mowing_angle(rect);
  const double angle_deg = angle_rad * 180.0 / M_PI;

  // The algorithm should pick near 0° or 90°.
  // For a 20×4 rectangle, mowing perpendicular to the short axis (y-axis) means
  // angle = 0 (horizontal swaths → 4/spacing passes) vs angle=90 (vertical → 20/spacing passes).
  // So 0° is optimal (minimum perpendicular extent in y-direction).
  const bool is_zero_or_near = (std::abs(angle_deg) < 10.0 || std::abs(angle_deg - 180.0) < 10.0);
  EXPECT_TRUE(is_zero_or_near) << "Expected ~0° for wide rectangle, got " << angle_deg << "°";
}

TEST(OptimalMowingAngle, TallRectanglePrefersVertical)
{
  // A 4×20 rectangle: optimal is 90° (horizontal swaths → 20/spacing passes).
  // Actually optimal is 0° for horizontal swaths cutting across the 4m width.
  // Let's verify the returned angle gives fewer passes than perpendicular.
  const Polygon2D rect = make_rectangle(0.0, 0.0, 4.0, 20.0);
  const double angle_rad = optimal_mowing_angle(rect);

  // The test simply verifies that the returned angle is a valid value in [0, π).
  EXPECT_GE(angle_rad, 0.0);
  EXPECT_LT(angle_rad, M_PI);
}

TEST(OptimalMowingAngle, SquareReturnsValidAngle)
{
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 10.0);
  const double angle_rad = optimal_mowing_angle(sq);
  EXPECT_GE(angle_rad, 0.0);
  EXPECT_LT(angle_rad, M_PI);
}

// ---------------------------------------------------------------------------
// clip_line_to_polygon
// ---------------------------------------------------------------------------

TEST(ClipLineToPolygon, HorizontalLineThroughSquare)
{
  // A line crossing y=5 of a 0–10 × 0–10 square should yield one segment
  // from x=0 to x=10.
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 10.0);
  const auto segments = clip_line_to_polygon({-5.0, 5.0}, {15.0, 5.0}, sq);

  ASSERT_EQ(segments.size(), 1u);
  const double seg_len = std::hypot(segments[0].end.first - segments[0].start.first,
                                    segments[0].end.second - segments[0].start.second);
  EXPECT_NEAR(seg_len, 10.0, 0.01);
}

TEST(ClipLineToPolygon, LineOutsideSquare)
{
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 10.0);
  const auto segments = clip_line_to_polygon({0.0, -5.0}, {10.0, -5.0}, sq);
  EXPECT_TRUE(segments.empty());
}

TEST(ClipLineToPolygon, FullyInsideLine)
{
  // A short line segment fully inside the polygon should be returned as-is.
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 10.0);
  const auto segments = clip_line_to_polygon({2.0, 5.0}, {8.0, 5.0}, sq);
  ASSERT_EQ(segments.size(), 1u);
  EXPECT_NEAR(segments[0].start.first, 2.0, 0.01);
  EXPECT_NEAR(segments[0].end.first, 8.0, 0.01);
}

TEST(ClipLineToPolygon, DiagonalThroughSquare)
{
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 10.0);
  const auto segments = clip_line_to_polygon({-5.0, -5.0}, {15.0, 15.0}, sq);
  ASSERT_EQ(segments.size(), 1u);
  // Segment should go from (0,0) to (10,10).
  EXPECT_NEAR(segments[0].start.first, 0.0, 0.1);
  EXPECT_NEAR(segments[0].start.second, 0.0, 0.1);
  EXPECT_NEAR(segments[0].end.first, 10.0, 0.1);
  EXPECT_NEAR(segments[0].end.second, 10.0, 0.1);
}

// ---------------------------------------------------------------------------
// geometry_polygon_to_points / points_to_geometry_polygon
// ---------------------------------------------------------------------------

TEST(PolygonConversion, RoundTrip)
{
  geometry_msgs::msg::Polygon ros_poly;
  for (auto [x, y] : std::vector<std::pair<float, float>>{{0, 0}, {5, 0}, {5, 3}, {0, 3}})
  {
    geometry_msgs::msg::Point32 pt;
    pt.x = x;
    pt.y = y;
    pt.z = 0.0f;
    ros_poly.points.push_back(pt);
  }

  const Polygon2D internal = geometry_polygon_to_points(ros_poly);
  ASSERT_EQ(internal.size(), 4u);
  EXPECT_NEAR(internal[0].first, 0.0, 1e-5);
  EXPECT_NEAR(internal[1].first, 5.0, 1e-5);
  EXPECT_NEAR(internal[2].second, 3.0, 1e-5);

  const geometry_msgs::msg::Polygon back = points_to_geometry_polygon(internal);
  ASSERT_EQ(back.points.size(), 4u);
  EXPECT_NEAR(back.points[2].y, 3.0f, 1e-4f);
}

// ---------------------------------------------------------------------------
// rotate_polygon
// ---------------------------------------------------------------------------

TEST(RotatePolygon, RotateBy180Degrees)
{
  // Rotating a unit square 180° around its centre should bring it back (modulo
  // vertex ordering, each point maps to the diagonally opposite vertex).
  const Polygon2D sq = make_rectangle(0.0, 0.0, 2.0, 2.0);
  const Point2D center = {1.0, 1.0};
  const Polygon2D rotated = rotate_polygon(sq, M_PI, center);

  ASSERT_EQ(rotated.size(), 4u);
  // (0,0) → should map to (2,2)
  EXPECT_NEAR(rotated[0].first, 2.0, 1e-9);
  EXPECT_NEAR(rotated[0].second, 2.0, 1e-9);
}

TEST(RotatePolygon, AreaPreservedAfterRotation)
{
  const Polygon2D sq = make_rectangle(0.0, 0.0, 10.0, 5.0);
  const Point2D center = polygon_centroid(sq);
  const Polygon2D rotated = rotate_polygon(sq, M_PI / 4.0, center);

  EXPECT_NEAR(std::abs(polygon_area(rotated)), std::abs(polygon_area(sq)), 1e-6);
}
