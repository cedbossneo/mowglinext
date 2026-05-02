// Copyright (C) 2024 Cedric <cedric@mowgli.dev>
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

// Internal-only helpers shared between map_server_node's translation
// units (costmap_filters.cpp, progress_tracker.cpp). Not part of the
// public API of mowgli_map — do not include from outside the package.

#ifndef MOWGLI_MAP__INTERNAL_HELPERS_HPP_
#define MOWGLI_MAP__INTERNAL_HELPERS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <geometry_msgs/msg/polygon.hpp>

namespace mowgli_map
{

/// Closest point on the polygon perimeter to (px, py), plus its distance.
struct ClosestEdge
{
  double x{0.0};
  double y{0.0};
  double distance{std::numeric_limits<double>::max()};
};

inline ClosestEdge closest_edge_point(double px,
                                      double py,
                                      const geometry_msgs::msg::Polygon& polygon)
{
  ClosestEdge best;
  const auto& pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 2)
  {
    return best;
  }

  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double ax = static_cast<double>(pts[j].x);
    const double ay = static_cast<double>(pts[j].y);
    const double bx = static_cast<double>(pts[i].x);
    const double by = static_cast<double>(pts[i].y);

    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;

    double t = 0.0;
    if (len2 > 1e-12)
    {
      t = std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0, 1.0);
    }

    const double cx = ax + t * dx;
    const double cy = ay + t * dy;
    const double dist = std::hypot(px - cx, py - cy);
    if (dist < best.distance)
    {
      best = {cx, cy, dist};
    }
  }
  return best;
}

/// Minimum distance from point (px, py) to the edges of a polygon.
inline double point_to_polygon_distance(double px,
                                        double py,
                                        const geometry_msgs::msg::Polygon& polygon)
{
  return closest_edge_point(px, py, polygon).distance;
}

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__INTERNAL_HELPERS_HPP_
