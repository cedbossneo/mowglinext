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
 * @file polygon_utils.hpp
 * @brief Pure geometric utility functions for polygon manipulation and coverage planning.
 *
 * All operations are performed in a 2-D Cartesian coordinate space using
 * double-precision floating-point arithmetic. No external geometry library
 * dependency — algorithms are implemented directly from first principles.
 */

#ifndef MOWGLI_COVERAGE_PLANNER__POLYGON_UTILS_HPP_
#define MOWGLI_COVERAGE_PLANNER__POLYGON_UTILS_HPP_

#include <cmath>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"

namespace mowgli_coverage_planner
{

/// Alias for a 2-D point represented as (x, y).
using Point2D = std::pair<double, double>;

/// Alias for a closed polygon whose last vertex implicitly connects back to the first.
using Polygon2D = std::vector<Point2D>;

/// Represents a clipped line segment returned by clip_line_to_polygon().
struct Segment2D
{
  Point2D start;
  Point2D end;
};

// ---------------------------------------------------------------------------
// Polygon metrics
// ---------------------------------------------------------------------------

/**
 * @brief Compute the signed area of a polygon using the Shoelace formula.
 *
 * A counter-clockwise winding yields a positive area; clockwise is negative.
 * The magnitude equals the true geometric area.
 *
 * @param polygon Closed polygon (vertices listed once; last edge closes to first).
 * @return Signed area in the same units² as the vertex coordinates.
 */
double polygon_area(const Polygon2D& polygon);

/**
 * @brief Compute the centroid (geometric centre of mass) of a simple polygon.
 *
 * Uses the standard formula derived from Green's theorem.
 *
 * @param polygon Closed polygon with at least 3 vertices.
 * @return Centroid point. Returns {0, 0} for degenerate polygons.
 */
Point2D polygon_centroid(const Polygon2D& polygon);

// ---------------------------------------------------------------------------
// Point / polygon predicates
// ---------------------------------------------------------------------------

/**
 * @brief Test whether a point lies strictly inside a polygon.
 *
 * Uses the ray-casting algorithm. Points on the boundary are treated as inside.
 *
 * @param point  Query point.
 * @param polygon Simple, closed polygon (no self-intersections assumed).
 * @return true if @p point is inside @p polygon.
 */
bool point_in_polygon(const Point2D& point, const Polygon2D& polygon);

// ---------------------------------------------------------------------------
// Polygon transformation
// ---------------------------------------------------------------------------

/**
 * @brief Offset a polygon inward (positive distance) or outward (negative distance).
 *
 * Each edge is shifted along its inward-facing normal by @p distance, and the
 * new vertex positions are computed as the intersection of adjacent offset edges.
 * Self-intersecting offset results are not handled — caller must ensure the
 * offset distance is smaller than the polygon's inradius.
 *
 * @param polygon  Input polygon (counter-clockwise winding preferred).
 * @param distance Positive value shrinks the polygon; negative expands it.
 * @return Offset polygon. Empty if the offset would collapse the polygon.
 */
Polygon2D offset_polygon(const Polygon2D& polygon, double distance);

/**
 * @brief Rotate all vertices of a polygon around a given centre point.
 *
 * @param polygon Input polygon.
 * @param angle_rad Rotation angle in radians (counter-clockwise positive).
 * @param center  Centre of rotation.
 * @return Rotated polygon.
 */
Polygon2D rotate_polygon(const Polygon2D& polygon, double angle_rad, const Point2D& center);

// ---------------------------------------------------------------------------
// Line–polygon clipping
// ---------------------------------------------------------------------------

/**
 * @brief Clip a line segment against the interior of a convex or concave polygon.
 *
 * Uses the Cyrus–Beck / Liang–Barsky parametric approach per edge to collect
 * entry and exit parameters, then reconstructs the interior portions.
 *
 * @param line_start  First endpoint of the infinite line to clip.
 * @param line_end    Second endpoint of the infinite line to clip.
 * @param polygon     Clipping polygon (must be convex for guaranteed results on a
 *                    single segment; concave polygons may yield multiple segments).
 * @return Zero or more segments lying inside @p polygon.
 */
std::vector<Segment2D> clip_line_to_polygon(const Point2D& line_start,
                                            const Point2D& line_end,
                                            const Polygon2D& polygon);

// ---------------------------------------------------------------------------
// Coverage planning helpers
// ---------------------------------------------------------------------------

/**
 * @brief Find the mowing angle (in radians) that minimises the number of swaths.
 *
 * Tests candidate angles every 5° in [0°, 180°) and selects the one that
 * maximises the projected length of the polygon along the perpendicular
 * direction (i.e. minimises the extent across the swath direction, which
 * reduces the number of passes).
 *
 * @param polygon  Area to analyse.
 * @return Optimal mowing angle in radians in [0, π).
 */
double optimal_mowing_angle(const Polygon2D& polygon);

// ---------------------------------------------------------------------------
// ROS message conversion
// ---------------------------------------------------------------------------

/**
 * @brief Convert a ROS geometry_msgs::msg::Polygon to a Polygon2D.
 *
 * @param msg ROS polygon message.
 * @return Equivalent Polygon2D.
 */
Polygon2D geometry_polygon_to_points(const geometry_msgs::msg::Polygon& msg);

/**
 * @brief Convert a Polygon2D back to a ROS geometry_msgs::msg::Polygon.
 *
 * All z-coordinates are set to 0.
 *
 * @param polygon Input polygon.
 * @return Equivalent ROS polygon message.
 */
geometry_msgs::msg::Polygon points_to_geometry_polygon(const Polygon2D& polygon);

}  // namespace mowgli_coverage_planner

#endif  // MOWGLI_COVERAGE_PLANNER__POLYGON_UTILS_HPP_
