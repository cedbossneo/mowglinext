#include "mowgli_brv_planner/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace brv
{

bool point_in_polygon(const Point2D& pt, const Polygon2D& poly)
{
  int n = static_cast<int>(poly.size());
  if (n < 3)
    return false;
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    if (((poly[i].y > pt.y) != (poly[j].y > pt.y)) &&
        (pt.x < (poly[j].x - poly[i].x) * (pt.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x))
    {
      inside = !inside;
    }
  }
  return inside;
}

double polygon_area(const Polygon2D& poly)
{
  double area = 0.0;
  int n = static_cast<int>(poly.size());
  for (int i = 0; i < n; ++i)
  {
    int j = (i + 1) % n;
    area += poly[i].x * poly[j].y;
    area -= poly[j].x * poly[i].y;
  }
  return area * 0.5;
}

Polygon2D convex_hull(const Polygon2D& pts)
{
  auto sorted = pts;
  std::sort(sorted.begin(),
            sorted.end(),
            [](const Point2D& a, const Point2D& b)
            {
              return a.x < b.x || (a.x == b.x && a.y < b.y);
            });

  // Andrew's monotone chain
  int n = static_cast<int>(sorted.size());
  if (n < 3)
    return sorted;

  auto cross = [](const Point2D& O, const Point2D& A, const Point2D& B)
  {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
  };

  Polygon2D hull(2 * n);
  int k = 0;

  // Lower hull
  for (int i = 0; i < n; ++i)
  {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], sorted[i]) <= 0)
      --k;
    hull[k++] = sorted[i];
  }

  // Upper hull
  for (int i = n - 2, t = k + 1; i >= 0; --i)
  {
    while (k >= t && cross(hull[k - 2], hull[k - 1], sorted[i]) <= 0)
      --k;
    hull[k++] = sorted[i];
  }

  hull.resize(k - 1);
  return hull;
}

Polygon2D offset_polygon_inward(const Polygon2D& poly, double dist)
{
  int n = static_cast<int>(poly.size());
  if (n < 3)
    return {};

  // Ensure CCW
  double area = polygon_area(poly);
  Polygon2D ordered = poly;
  if (area < 0)
  {
    std::reverse(ordered.begin(), ordered.end());
  }

  Polygon2D result;
  for (int i = 0; i < n; ++i)
  {
    int prev = (i + n - 1) % n;
    int next = (i + 1) % n;

    // Edge normals (inward for CCW polygon)
    double dx1 = ordered[i].x - ordered[prev].x;
    double dy1 = ordered[i].y - ordered[prev].y;
    double len1 = std::hypot(dx1, dy1);
    double nx1 = -dy1 / len1;
    double ny1 = dx1 / len1;

    double dx2 = ordered[next].x - ordered[i].x;
    double dy2 = ordered[next].y - ordered[i].y;
    double len2 = std::hypot(dx2, dy2);
    double nx2 = -dy2 / len2;
    double ny2 = dx2 / len2;

    // Bisector
    double bx = nx1 + nx2;
    double by = ny1 + ny2;
    double blen = std::hypot(bx, by);
    if (blen < 1e-10)
    {
      bx = nx1;
      by = ny1;
      blen = 1.0;
    }

    // Scale factor for miter offset
    double dot = nx1 * bx / blen + ny1 * by / blen;
    double scale = (std::abs(dot) > 1e-10) ? dist / dot : dist;
    scale = std::min(scale, dist * 3.0);  // Limit miter spike

    result.push_back({ordered[i].x + bx / blen * scale, ordered[i].y + by / blen * scale});
  }

  // Validate result has positive area
  if (std::abs(polygon_area(result)) < 1e-6)
  {
    return {};
  }
  return result;
}

std::vector<Point2D> sample_polygon_edges(const Polygon2D& poly, double spacing)
{
  std::vector<Point2D> pts;
  int n = static_cast<int>(poly.size());
  for (int i = 0; i < n; ++i)
  {
    int j = (i + 1) % n;
    double dx = poly[j].x - poly[i].x;
    double dy = poly[j].y - poly[i].y;
    double edge_len = std::hypot(dx, dy);
    int samples = std::max(1, static_cast<int>(edge_len / spacing));
    for (int s = 0; s < samples; ++s)
    {
      double t = static_cast<double>(s) / samples;
      pts.push_back({poly[i].x + t * dx, poly[i].y + t * dy});
    }
  }
  return pts;
}

Point2D rotate_point(const Point2D& pt, double angle)
{
  double c = std::cos(angle);
  double s = std::sin(angle);
  return {c * pt.x - s * pt.y, s * pt.x + c * pt.y};
}

Point2D unrotate_point(const Point2D& pt, double angle)
{
  return rotate_point(pt, -angle);
}

double path_length(const Path2D& path)
{
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
  {
    total += path[i].dist(path[i - 1]);
  }
  return total;
}

static double cross2d(const Point2D& o, const Point2D& a, const Point2D& b)
{
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

static bool on_segment(const Point2D& p, const Point2D& q, const Point2D& r)
{
  return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) &&
         q.y >= std::min(p.y, r.y);
}

bool segments_intersect(const Point2D& a1, const Point2D& a2, const Point2D& b1, const Point2D& b2)
{
  double d1 = cross2d(b1, b2, a1);
  double d2 = cross2d(b1, b2, a2);
  double d3 = cross2d(a1, a2, b1);
  double d4 = cross2d(a1, a2, b2);

  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
  {
    return true;
  }

  if (std::abs(d1) < 1e-10 && on_segment(b1, a1, b2))
    return true;
  if (std::abs(d2) < 1e-10 && on_segment(b1, a2, b2))
    return true;
  if (std::abs(d3) < 1e-10 && on_segment(a1, b1, a2))
    return true;
  if (std::abs(d4) < 1e-10 && on_segment(a1, b2, a2))
    return true;

  return false;
}

bool segment_inside_polygon(const Point2D& a, const Point2D& b, const Polygon2D& poly)
{
  // Check midpoint is inside
  Point2D mid{(a.x + b.x) / 2, (a.y + b.y) / 2};
  if (!point_in_polygon(mid, poly))
    return false;

  // Check no edge intersection
  int n = static_cast<int>(poly.size());
  for (int i = 0; i < n; ++i)
  {
    int j = (i + 1) % n;
    if (segments_intersect(a, b, poly[i], poly[j]))
    {
      return false;
    }
  }
  return true;
}

}  // namespace brv
