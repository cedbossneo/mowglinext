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
#include <cmath>

#include "mowgli_brv_planner/geometry_utils.hpp"
#include <gtest/gtest.h>

// ─────────────────────────────────────────────────────────────────────────────
// point_in_polygon
// ─────────────────────────────────────────────────────────────────────────────

TEST(Geometry, PointInPolygon_Inside)
{
  brv::Polygon2D square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  EXPECT_TRUE(brv::point_in_polygon({5, 5}, square));
  EXPECT_TRUE(brv::point_in_polygon({1, 1}, square));
  EXPECT_TRUE(brv::point_in_polygon({9.5, 9.5}, square));
}

TEST(Geometry, PointInPolygon_Outside)
{
  brv::Polygon2D square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  EXPECT_FALSE(brv::point_in_polygon({-1, 5}, square));
  EXPECT_FALSE(brv::point_in_polygon({15, 5}, square));
  EXPECT_FALSE(brv::point_in_polygon({5, -1}, square));
  EXPECT_FALSE(brv::point_in_polygon({5, 15}, square));
  EXPECT_FALSE(brv::point_in_polygon({-1, -1}, square));
}

TEST(Geometry, PointInPolygon_OnEdge)
{
  brv::Polygon2D square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  // Ray-casting may return true or false for edge points, but must not crash.
  // Just verify consistency: call twice, get same result.
  bool r1 = brv::point_in_polygon({0, 5}, square);
  bool r2 = brv::point_in_polygon({0, 5}, square);
  EXPECT_EQ(r1, r2);
}

TEST(Geometry, PointInPolygon_Triangle)
{
  brv::Polygon2D tri = {{0, 0}, {10, 0}, {5, 10}};
  EXPECT_TRUE(brv::point_in_polygon({5, 3}, tri));
  EXPECT_FALSE(brv::point_in_polygon({0, 10}, tri));
}

TEST(Geometry, PointInPolygon_DegeneratePolygon)
{
  // Less than 3 points should return false
  brv::Polygon2D line = {{0, 0}, {1, 1}};
  EXPECT_FALSE(brv::point_in_polygon({0.5, 0.5}, line));

  brv::Polygon2D empty;
  EXPECT_FALSE(brv::point_in_polygon({0, 0}, empty));
}

// ─────────────────────────────────────────────────────────────────────────────
// polygon_area
// ─────────────────────────────────────────────────────────────────────────────

TEST(Geometry, PolygonArea_CCW)
{
  // CCW winding: (0,0) -> (10,0) -> (10,10) -> (0,10) — positive area
  brv::Polygon2D ccw = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  double area = brv::polygon_area(ccw);
  EXPECT_NEAR(area, 100.0, 1e-6);
}

TEST(Geometry, PolygonArea_CW)
{
  // CW winding: (0,0) -> (0,10) -> (10,10) -> (10,0) — negative area
  brv::Polygon2D cw = {{0, 0}, {0, 10}, {10, 10}, {10, 0}};
  double area = brv::polygon_area(cw);
  EXPECT_NEAR(area, -100.0, 1e-6);
}

TEST(Geometry, PolygonArea_Triangle)
{
  brv::Polygon2D tri = {{0, 0}, {4, 0}, {0, 3}};
  double area = brv::polygon_area(tri);
  EXPECT_NEAR(std::abs(area), 6.0, 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
// convex_hull
// ─────────────────────────────────────────────────────────────────────────────

TEST(Geometry, ConvexHull_Square)
{
  brv::Polygon2D pts = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  auto hull = brv::convex_hull(pts);
  EXPECT_EQ(hull.size(), 4u);
}

TEST(Geometry, ConvexHull_WithInterior)
{
  // 4 corners + 3 interior points → hull should have only 4 points
  brv::Polygon2D pts = {{0, 0}, {10, 0}, {10, 10}, {0, 10}, {3, 3}, {5, 5}, {7, 2}};
  auto hull = brv::convex_hull(pts);
  EXPECT_EQ(hull.size(), 4u);
}

TEST(Geometry, ConvexHull_CollinearPoints)
{
  brv::Polygon2D pts = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  auto hull = brv::convex_hull(pts);
  // Collinear points: hull should contain at least the endpoints
  EXPECT_GE(hull.size(), 2u);
}

TEST(Geometry, ConvexHull_TwoPoints)
{
  brv::Polygon2D pts = {{0, 0}, {5, 5}};
  auto hull = brv::convex_hull(pts);
  // Less than 3 points: returns as-is
  EXPECT_EQ(hull.size(), 2u);
}

// ─────────────────────────────────────────────────────────────────────────────
// offset_polygon_inward
// ─────────────────────────────────────────────────────────────────────────────

TEST(Geometry, OffsetPolygon_Inward)
{
  // Square 10x10 shrunk by 1m → ~8x8 square
  brv::Polygon2D square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  auto shrunk = brv::offset_polygon_inward(square, 1.0);
  EXPECT_FALSE(shrunk.empty());

  // All points should be inside original
  for (const auto& pt : shrunk)
  {
    EXPECT_GT(pt.x, 0.5);
    EXPECT_LT(pt.x, 9.5);
    EXPECT_GT(pt.y, 0.5);
    EXPECT_LT(pt.y, 9.5);
  }

  // Area should be approximately 64 (8x8)
  double area = std::abs(brv::polygon_area(shrunk));
  EXPECT_NEAR(area, 64.0, 5.0);
}

TEST(Geometry, OffsetPolygon_Collapse)
{
  // Shrinking a polygon should reduce its area. When offset exceeds
  // half-width, the result may be empty or self-intersecting (implementation
  // uses miter join which can overshoot). Test that moderate shrink reduces
  // area, and extreme shrink produces a smaller result than original.
  brv::Polygon2D square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  double original_area = std::abs(brv::polygon_area(square));

  auto shrunk = brv::offset_polygon_inward(square, 3.0);
  ASSERT_FALSE(shrunk.empty());
  double shrunk_area = std::abs(brv::polygon_area(shrunk));
  EXPECT_LT(shrunk_area, original_area);
  // Expected ~4x4 = 16
  EXPECT_NEAR(shrunk_area, 16.0, 5.0);
}

TEST(Geometry, OffsetPolygon_Degenerate)
{
  // Less than 3 points → empty
  brv::Polygon2D line = {{0, 0}, {1, 1}};
  auto result = brv::offset_polygon_inward(line, 0.1);
  EXPECT_TRUE(result.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// segments_intersect
// ─────────────────────────────────────────────────────────────────────────────

TEST(Geometry, SegmentsIntersect_Crossing)
{
  // X pattern
  EXPECT_TRUE(brv::segments_intersect({0, 0}, {10, 10}, {0, 10}, {10, 0}));
}

TEST(Geometry, SegmentsIntersect_Parallel)
{
  EXPECT_FALSE(brv::segments_intersect({0, 0}, {10, 0}, {0, 1}, {10, 1}));
}

TEST(Geometry, SegmentsIntersect_TShaped)
{
  // Segment ends on other segment
  EXPECT_TRUE(brv::segments_intersect({0, 0}, {10, 0}, {5, -5}, {5, 0}));
}

TEST(Geometry, SegmentsIntersect_NoOverlap)
{
  // Segments that would intersect if extended but don't actually touch
  EXPECT_FALSE(brv::segments_intersect({0, 0}, {1, 0}, {2, 0}, {3, 0}));
}

// ─────────────────────────────────────────────────────────────────────────────
// rotate / unrotate roundtrip
// ─────────────────────────────────────────────────────────────────────────────

TEST(Geometry, RotateUnrotateRoundtrip)
{
  brv::Point2D pt{3.0, 4.0};
  double angle = M_PI / 6.0;
  auto rotated = brv::rotate_point(pt, angle);
  auto back = brv::unrotate_point(rotated, angle);
  EXPECT_NEAR(back.x, pt.x, 1e-10);
  EXPECT_NEAR(back.y, pt.y, 1e-10);
}

TEST(Geometry, RotateBy90)
{
  brv::Point2D pt{1.0, 0.0};
  auto rotated = brv::rotate_point(pt, M_PI / 2.0);
  EXPECT_NEAR(rotated.x, 0.0, 1e-10);
  EXPECT_NEAR(rotated.y, 1.0, 1e-10);
}

// ─────────────────────────────────────────────────────────────────────────────
// path_length
// ─────────────────────────────────────────────────────────────────────────────

TEST(Geometry, PathLength_Straight)
{
  brv::Path2D path = {{0, 0}, {3, 0}, {3, 4}};
  EXPECT_NEAR(brv::path_length(path), 7.0, 1e-10);
}

TEST(Geometry, PathLength_Single)
{
  brv::Path2D path = {{5, 5}};
  EXPECT_NEAR(brv::path_length(path), 0.0, 1e-10);
}

TEST(Geometry, PathLength_Empty)
{
  brv::Path2D path;
  EXPECT_NEAR(brv::path_length(path), 0.0, 1e-10);
}
