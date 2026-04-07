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
 * @file polygon_utils.cpp
 * @brief Implementation of pure geometric polygon utilities.
 *
 * All algorithms are implemented from first principles; no external geometry
 * library dependency (no CGAL, Clipper, or Boost.Geometry).
 */

#include "mowgli_coverage_planner/polygon_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace mowgli_coverage_planner
{

// ---------------------------------------------------------------------------
// Internal helpers (translation-unit scope)
// ---------------------------------------------------------------------------

namespace
{

/// Degrees to radians conversion factor.
constexpr double kDegToRad = M_PI / 180.0;

/// Tolerance for floating-point comparisons.
constexpr double kEps = 1e-9;

/**
 * @brief Find the intersection of two lines, each defined by a point and a direction.
 *
 * Line 1: P + t * D1
 * Line 2: Q + s * D2
 *
 * @param p  Point on line 1.
 * @param d1 Direction of line 1.
 * @param q  Point on line 2.
 * @param d2 Direction of line 2.
 * @param[out] intersection  The intersection point if lines are not parallel.
 * @return true if a unique intersection exists (lines not parallel).
 */
bool line_line_intersection(
    const Point2D& p, const Point2D& d1, const Point2D& q, const Point2D& d2, Point2D& intersection)
{
  const double denom = d1.first * d2.second - d1.second * d2.first;
  if (std::abs(denom) < kEps)
  {
    return false;  // parallel
  }
  const double dx = q.first - p.first;
  const double dy = q.second - p.second;
  const double t = (dx * d2.second - dy * d2.first) / denom;
  intersection.first = p.first + t * d1.first;
  intersection.second = p.second + t * d1.second;
  return true;
}

/**
 * @brief Ensure a polygon is wound counter-clockwise.
 *
 * Reverses the vertex order when the signed area is negative (CW winding).
 */
Polygon2D ensure_ccw(Polygon2D poly)
{
  if (polygon_area(poly) < 0.0)
  {
    std::reverse(poly.begin(), poly.end());
  }
  return poly;
}

}  // namespace

// ---------------------------------------------------------------------------
// polygon_area
// ---------------------------------------------------------------------------

double polygon_area(const Polygon2D& polygon)
{
  const std::size_t n = polygon.size();
  if (n < 3)
  {
    return 0.0;
  }
  double area = 0.0;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    area += (polygon[j].first + polygon[i].first) * (polygon[j].second - polygon[i].second);
  }
  return area * 0.5;
}

// ---------------------------------------------------------------------------
// polygon_centroid
// ---------------------------------------------------------------------------

Point2D polygon_centroid(const Polygon2D& polygon)
{
  const std::size_t n = polygon.size();
  if (n == 0)
  {
    return {0.0, 0.0};
  }
  if (n == 1)
  {
    return polygon[0];
  }
  if (n == 2)
  {
    return {(polygon[0].first + polygon[1].first) * 0.5,
            (polygon[0].second + polygon[1].second) * 0.5};
  }

  double cx = 0.0;
  double cy = 0.0;
  double signed_area = 0.0;

  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double a = polygon[j].first * polygon[i].second - polygon[i].first * polygon[j].second;
    signed_area += a;
    cx += (polygon[j].first + polygon[i].first) * a;
    cy += (polygon[j].second + polygon[i].second) * a;
  }

  signed_area *= 0.5;
  if (std::abs(signed_area) < kEps)
  {
    // Degenerate — fall back to arithmetic mean.
    double sx = 0.0;
    double sy = 0.0;
    for (const auto& p : polygon)
    {
      sx += p.first;
      sy += p.second;
    }
    const double inv_n = 1.0 / static_cast<double>(n);
    return {sx * inv_n, sy * inv_n};
  }

  const double inv6a = 1.0 / (6.0 * signed_area);
  return {cx * inv6a, cy * inv6a};
}

// ---------------------------------------------------------------------------
// point_in_polygon  (ray-casting)
// ---------------------------------------------------------------------------

bool point_in_polygon(const Point2D& point, const Polygon2D& polygon)
{
  const std::size_t n = polygon.size();
  if (n < 3)
  {
    return false;
  }

  bool inside = false;
  const double px = point.first;
  const double py = point.second;

  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double xi = polygon[i].first;
    const double yi = polygon[i].second;
    const double xj = polygon[j].first;
    const double yj = polygon[j].second;

    // Check if the ray cast horizontally from (px, py) crosses this edge.
    const bool crosses_y = ((yi > py) != (yj > py));
    if (crosses_y)
    {
      // x-coordinate of the edge at y = py.
      const double x_cross = xj + (py - yj) / (yi - yj) * (xi - xj);
      if (px < x_cross)
      {
        inside = !inside;
      }
    }
  }
  return inside;
}

// ---------------------------------------------------------------------------
// offset_polygon
// ---------------------------------------------------------------------------

Polygon2D offset_polygon(const Polygon2D& polygon, double distance)
{
  const std::size_t n = polygon.size();
  if (n < 3)
  {
    return {};
  }

  // Work in CCW winding so that "inward" is well-defined.
  Polygon2D poly = ensure_ccw(polygon);

  // For each edge, compute the offset line (edge shifted by distance along
  // its inward normal).  The inward normal of a CCW edge (a→b) points to
  // the left: normal = (-dy, dx) / |edge|, then scaled by distance.
  //
  // Positive distance → inward (shrink).

  struct OffsetEdge
  {
    Point2D point;  // any point on the offset edge
    Point2D direction;  // unit direction of the edge
  };

  std::vector<OffsetEdge> offset_edges;
  offset_edges.reserve(n);

  for (std::size_t i = 0; i < n; i++)
  {
    const Point2D& a = poly[i];
    const Point2D& b = poly[(i + 1) % n];

    const double dx = b.first - a.first;
    const double dy = b.second - a.second;
    const double len = std::hypot(dx, dy);

    if (len < kEps)
    {
      continue;  // degenerate edge — skip
    }

    // Inward-facing unit normal for CCW polygon: rotate edge direction 90° CW.
    const double nx = dy / len;
    const double ny = -dx / len;

    // Shift edge origin along the inward normal.
    const Point2D shifted_origin = {a.first + nx * distance, a.second + ny * distance};
    const Point2D edge_dir = {dx / len, dy / len};

    offset_edges.push_back({shifted_origin, edge_dir});
  }

  if (offset_edges.size() < 3)
  {
    return {};
  }

  // Compute the new vertices as intersections of consecutive offset edges.
  const std::size_t m = offset_edges.size();
  Polygon2D result;
  result.reserve(m);

  for (std::size_t i = 0; i < m; i++)
  {
    const OffsetEdge& e0 = offset_edges[i];
    const OffsetEdge& e1 = offset_edges[(i + 1) % m];

    Point2D intersection;
    if (line_line_intersection(e0.point, e0.direction, e1.point, e1.direction, intersection))
    {
      result.push_back(intersection);
    }
    else
    {
      // Parallel edges — use the point on the next offset edge directly.
      result.push_back(e1.point);
    }
  }

  // Sanity-check: if the offset collapsed or inverted the polygon, return empty.
  const double original_area = std::abs(polygon_area(poly));
  const double offset_area = std::abs(polygon_area(result));
  if (offset_area < kEps || (distance > 0.0 && offset_area >= original_area))
  {
    return {};
  }

  return result;
}

// ---------------------------------------------------------------------------
// rotate_polygon
// ---------------------------------------------------------------------------

Polygon2D rotate_polygon(const Polygon2D& polygon, double angle_rad, const Point2D& center)
{
  const double cos_a = std::cos(angle_rad);
  const double sin_a = std::sin(angle_rad);

  Polygon2D result;
  result.reserve(polygon.size());

  for (const auto& v : polygon)
  {
    const double tx = v.first - center.first;
    const double ty = v.second - center.second;
    result.push_back(
        {center.first + cos_a * tx - sin_a * ty, center.second + sin_a * tx + cos_a * ty});
  }
  return result;
}

// ---------------------------------------------------------------------------
// clip_line_to_polygon
//
// Strategy: collect all t-parameter values in [0,1] at which the line
// segment crosses any polygon edge, sort them, then classify each resulting
// sub-interval by sampling its midpoint with point_in_polygon().  This is
// winding-order-agnostic and works for both convex and simple concave polygons.
// ---------------------------------------------------------------------------

std::vector<Segment2D> clip_line_to_polygon(const Point2D& line_start,
                                            const Point2D& line_end,
                                            const Polygon2D& polygon)
{
  const std::size_t n = polygon.size();
  if (n < 3)
  {
    return {};
  }

  const double dx = line_end.first - line_start.first;
  const double dy = line_end.second - line_start.second;

  // Collect all crossing parameters t ∈ [0, 1] between the line segment
  // P(t) = line_start + t*(line_end - line_start) and every polygon edge.
  std::vector<double> t_values;
  t_values.push_back(0.0);
  t_values.push_back(1.0);

  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    // Edge from polygon[j] to polygon[i].
    const double ex = polygon[i].first - polygon[j].first;
    const double ey = polygon[i].second - polygon[j].second;

    // Solve for t (line parameter) and s (edge parameter) simultaneously:
    //   line_start + t*D = polygon[j] + s*E
    // Cross-multiply to get t and s.
    const double denom = dx * ey - dy * ex;

    if (std::abs(denom) < kEps)
    {
      continue;  // line is parallel to this edge
    }

    const double ox = polygon[j].first - line_start.first;
    const double oy = polygon[j].second - line_start.second;

    const double t = (ox * ey - oy * ex) / denom;
    const double s = (ox * dy - oy * dx) / denom;

    // Only record if the crossing is within the line segment AND on the edge.
    if (t > kEps && t < 1.0 - kEps && s > -kEps && s < 1.0 + kEps)
    {
      t_values.push_back(t);
    }
  }

  // Sort all t-values.
  std::sort(t_values.begin(), t_values.end());

  // Deduplicate very close t-values to avoid zero-length segments.
  t_values.erase(std::unique(t_values.begin(),
                             t_values.end(),
                             [](double a, double b)
                             {
                               return b - a < kEps;
                             }),
                 t_values.end());

  // Classify each sub-interval: keep it if its midpoint is inside the polygon.
  std::vector<Segment2D> result;

  for (std::size_t k = 0; k + 1 < t_values.size(); ++k)
  {
    const double t0 = t_values[k];
    const double t1 = t_values[k + 1];
    const double tm = 0.5 * (t0 + t1);

    const Point2D mid = {line_start.first + tm * dx, line_start.second + tm * dy};

    if (point_in_polygon(mid, polygon))
    {
      result.push_back({{line_start.first + t0 * dx, line_start.second + t0 * dy},
                        {line_start.first + t1 * dx, line_start.second + t1 * dy}});
    }
  }

  // Merge adjacent segments (they arise when a crossing is exactly on a vertex).
  if (result.size() > 1)
  {
    std::vector<Segment2D> merged;
    merged.push_back(result[0]);
    for (std::size_t k = 1; k < result.size(); ++k)
    {
      auto& last = merged.back();
      const auto& cur = result[k];
      const double gap =
          std::hypot(cur.start.first - last.end.first, cur.start.second - last.end.second);
      if (gap < kEps * 10.0)
      {
        last.end = cur.end;
      }
      else
      {
        merged.push_back(cur);
      }
    }
    return merged;
  }

  return result;
}

// ---------------------------------------------------------------------------
// optimal_mowing_angle
// ---------------------------------------------------------------------------

double optimal_mowing_angle(const Polygon2D& polygon)
{
  if (polygon.size() < 3)
  {
    return 0.0;
  }

  // Find the bounding-box extent of the polygon when rotated by each candidate
  // angle. We want to maximise the extent along the mowing direction (which
  // reduces the number of swaths perpendicular to that direction).
  //
  // Equivalently: find the angle θ such that the projection of all vertices
  // onto the axis perpendicular to θ is minimised (fewer swaths).

  double best_angle_rad = 0.0;
  double min_perp_extent = std::numeric_limits<double>::max();

  // Test every 5° in [0°, 180°).
  for (int deg = 0; deg < 180; deg += 5)
  {
    const double angle_rad = deg * kDegToRad;

    // Unit vector perpendicular to the mowing direction.
    const double perp_x = -std::sin(angle_rad);
    const double perp_y = std::cos(angle_rad);

    double proj_min = std::numeric_limits<double>::max();
    double proj_max = std::numeric_limits<double>::lowest();

    for (const auto& v : polygon)
    {
      const double proj = perp_x * v.first + perp_y * v.second;
      proj_min = std::min(proj_min, proj);
      proj_max = std::max(proj_max, proj);
    }

    const double extent = proj_max - proj_min;
    if (extent < min_perp_extent)
    {
      min_perp_extent = extent;
      best_angle_rad = angle_rad;
    }
  }

  return best_angle_rad;
}

// ---------------------------------------------------------------------------
// ROS message conversion
// ---------------------------------------------------------------------------

Polygon2D geometry_polygon_to_points(const geometry_msgs::msg::Polygon& msg)
{
  Polygon2D result;
  result.reserve(msg.points.size());
  for (const auto& pt : msg.points)
  {
    result.push_back({static_cast<double>(pt.x), static_cast<double>(pt.y)});
  }
  return result;
}

geometry_msgs::msg::Polygon points_to_geometry_polygon(const Polygon2D& polygon)
{
  geometry_msgs::msg::Polygon msg;
  msg.points.reserve(polygon.size());
  for (const auto& pt : polygon)
  {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(pt.first);
    p.y = static_cast<float>(pt.second);
    p.z = 0.0f;
    msg.points.push_back(p);
  }
  return msg;
}

}  // namespace mowgli_coverage_planner
