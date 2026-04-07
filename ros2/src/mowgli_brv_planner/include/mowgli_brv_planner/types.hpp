#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace brv
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
  Point2D() = default;
  Point2D(double x_, double y_) : x(x_), y(y_)
  {
  }
  double dist(const Point2D& o) const
  {
    return std::hypot(x - o.x, y - o.y);
  }
};

using Polygon2D = std::vector<Point2D>;
using Path2D = std::vector<Point2D>;

struct SwathInfo
{
  Point2D start;
  Point2D end;
};

struct CoverageResult
{
  Path2D full_path;  // all waypoints (coverage + transit)
  Path2D outline_path;  // headland outline
  std::vector<SwathInfo> swaths;
  double total_distance = 0.0;
  double coverage_area = 0.0;
};

// Grid cell states (Eq. 7 in paper)
enum class CellState : uint8_t
{
  UNVISITABLE = 2,
  UNVISITED = 0,
  VISITED = 1,
};

}  // namespace brv
