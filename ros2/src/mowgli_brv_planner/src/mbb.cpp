#include "mowgli_brv_planner/mbb.hpp"

#include <cmath>
#include <limits>

#include "mowgli_brv_planner/geometry_utils.hpp"

namespace brv
{

double compute_mbb_angle(const Polygon2D& boundary)
{
  if (boundary.size() < 3)
    return 0.0;

  Polygon2D hull = convex_hull(boundary);
  int n = static_cast<int>(hull.size());
  if (n < 3)
    return 0.0;

  double min_area = std::numeric_limits<double>::max();
  double best_angle = 0.0;

  for (int i = 0; i < n; ++i)
  {
    int j = (i + 1) % n;
    double dx = hull[j].x - hull[i].x;
    double dy = hull[j].y - hull[i].y;
    double angle = std::atan2(dy, dx);

    // Rotate all hull points by -angle
    double cos_a = std::cos(-angle);
    double sin_a = std::sin(-angle);

    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();

    for (const auto& pt : hull)
    {
      double rx = cos_a * pt.x - sin_a * pt.y;
      double ry = sin_a * pt.x + cos_a * pt.y;
      x_min = std::min(x_min, rx);
      x_max = std::max(x_max, rx);
      y_min = std::min(y_min, ry);
      y_max = std::max(y_max, ry);
    }

    double w = x_max - x_min;
    double h = y_max - y_min;
    double area = w * h;

    if (area < min_area)
    {
      min_area = area;
      // Sweep along the longest edge to minimize turns
      best_angle = (w >= h) ? angle : angle + M_PI / 2.0;
    }
  }

  return best_angle;
}

}  // namespace brv
