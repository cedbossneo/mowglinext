#include "mowgli_brv_planner/voronoi_roadmap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>

#include "mowgli_brv_planner/geometry_utils.hpp"
#include <boost/polygon/voronoi.hpp>

namespace brv
{

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

// Boost.Polygon requires integer coordinates for Voronoi.
// Scale points to integer grid with sufficient precision.
static constexpr double VORONOI_SCALE = 1000.0;  // 1mm precision

struct IntPoint
{
  int x, y;
  IntPoint(int x_, int y_) : x(x_), y(y_)
  {
  }
};

}  // namespace brv

// Boost.Polygon traits for IntPoint
namespace boost
{
namespace polygon
{
template <>
struct geometry_concept<brv::IntPoint>
{
  using type = point_concept;
};

template <>
struct point_traits<brv::IntPoint>
{
  using coordinate_type = int;
  static int get(const brv::IntPoint& p, orientation_2d orient)
  {
    return (orient == HORIZONTAL) ? p.x : p.y;
  }
};
}  // namespace polygon
}  // namespace boost

namespace brv
{

VoronoiRoadmap::VoronoiRoadmap(const Polygon2D& boundary,
                               const std::vector<Polygon2D>& obstacles,
                               double sample_spacing,
                               int knn)
    : boundary_(boundary), obstacles_(obstacles)
{
  build_from_voronoi(boundary, obstacles, sample_spacing, knn);
}

void VoronoiRoadmap::build_from_voronoi(const Polygon2D& boundary,
                                        const std::vector<Polygon2D>& obstacles,
                                        double sample_spacing,
                                        int knn)
{
  // Sample points along all polygon edges
  auto boundary_samples = sample_polygon_edges(boundary, sample_spacing);
  std::vector<Point2D> all_samples = boundary_samples;
  for (const auto& obs : obstacles)
  {
    auto obs_samples = sample_polygon_edges(obs, sample_spacing);
    all_samples.insert(all_samples.end(), obs_samples.begin(), obs_samples.end());
  }

  if (all_samples.size() < 4)
    return;

  // Convert to integer coordinates for Boost.Polygon Voronoi
  std::vector<IntPoint> int_pts;
  int_pts.reserve(all_samples.size());
  for (const auto& p : all_samples)
  {
    int_pts.emplace_back(static_cast<int>(p.x * VORONOI_SCALE),
                         static_cast<int>(p.y * VORONOI_SCALE));
  }

  // Build Voronoi diagram
  voronoi_diagram<double> vd;
  boost::polygon::construct_voronoi(int_pts.begin(), int_pts.end(), &vd);

  // Extract valid vertices (inside boundary, outside obstacles)
  std::unordered_map<std::size_t, int> vert_map;
  for (auto it = vd.vertices().begin(); it != vd.vertices().end(); ++it)
  {
    double vx = it->x() / VORONOI_SCALE;
    double vy = it->y() / VORONOI_SCALE;
    Point2D pt{vx, vy};

    if (!point_in_polygon(pt, boundary))
      continue;

    bool in_obs = false;
    for (const auto& obs : obstacles)
    {
      if (point_in_polygon(pt, obs))
      {
        in_obs = true;
        break;
      }
    }
    if (in_obs)
      continue;

    std::size_t vor_idx = it - vd.vertices().begin();
    vert_map[vor_idx] = static_cast<int>(nodes_.size());
    nodes_.push_back(pt);
  }

  if (nodes_.empty())
    return;

  // Build adjacency from Voronoi edges
  adj_.resize(nodes_.size());
  for (auto it = vd.edges().begin(); it != vd.edges().end(); ++it)
  {
    if (!it->is_primary())
      continue;
    if (it->vertex0() == nullptr || it->vertex1() == nullptr)
      continue;

    std::size_t idx0 = it->vertex0() - &vd.vertices()[0];
    std::size_t idx1 = it->vertex1() - &vd.vertices()[0];

    auto it0 = vert_map.find(idx0);
    auto it1 = vert_map.find(idx1);
    if (it0 == vert_map.end() || it1 == vert_map.end())
      continue;

    int n0 = it0->second;
    int n1 = it1->second;

    // Check edge midpoint is inside boundary and not crossing obstacles
    Point2D mid{(nodes_[n0].x + nodes_[n1].x) / 2, (nodes_[n0].y + nodes_[n1].y) / 2};
    if (!point_in_polygon(mid, boundary))
      continue;

    bool blocked = false;
    for (const auto& obs : obstacles)
    {
      if (point_in_polygon(mid, obs))
      {
        blocked = true;
        break;
      }
    }
    if (blocked)
      continue;

    double dist = nodes_[n0].dist(nodes_[n1]);
    adj_[n0].push_back({n1, dist});
    adj_[n1].push_back({n0, dist});
  }

  // Add KNN connections for better graph connectivity
  if (knn > 0 && nodes_.size() > 1)
  {
    // Simple O(n*k) nearest neighbor (good enough for typical garden sizes)
    int k = std::min(knn, static_cast<int>(nodes_.size()) - 1);
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i)
    {
      // Find k nearest
      std::vector<std::pair<double, int>> dists;
      for (int j = 0; j < static_cast<int>(nodes_.size()); ++j)
      {
        if (j == i)
          continue;
        dists.push_back({nodes_[i].dist(nodes_[j]), j});
      }
      std::partial_sort(dists.begin(),
                        dists.begin() + std::min(k, static_cast<int>(dists.size())),
                        dists.end());

      for (int ki = 0; ki < std::min(k, static_cast<int>(dists.size())); ++ki)
      {
        int j = dists[ki].second;
        double d = dists[ki].first;

        // Check collision-free
        Point2D mid{(nodes_[i].x + nodes_[j].x) / 2, (nodes_[i].y + nodes_[j].y) / 2};
        if (!point_in_polygon(mid, boundary))
          continue;

        bool blocked = false;
        for (const auto& obs : obstacles)
        {
          if (point_in_polygon(mid, obs))
          {
            blocked = true;
            break;
          }
        }
        if (blocked)
          continue;

        // Check not already connected
        bool already = false;
        for (const auto& e : adj_[i])
        {
          if (e.to == j)
          {
            already = true;
            break;
          }
        }
        if (!already)
        {
          adj_[i].push_back({j, d});
          adj_[j].push_back({i, d});
        }
      }
    }
  }
}

Path2D VoronoiRoadmap::find_path(const Point2D& start, const Point2D& end) const
{
  if (nodes_.empty())
    return {start, end};

  // Find nearest nodes to start and end
  int start_idx = 0, end_idx = 0;
  double best_sd = std::numeric_limits<double>::max();
  double best_ed = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(nodes_.size()); ++i)
  {
    double ds = start.dist(nodes_[i]);
    double de = end.dist(nodes_[i]);
    if (ds < best_sd)
    {
      best_sd = ds;
      start_idx = i;
    }
    if (de < best_ed)
    {
      best_ed = de;
      end_idx = i;
    }
  }

  if (start_idx == end_idx)
  {
    return {start, nodes_[start_idx], end};
  }

  // Dijkstra (Algorithm 3 from paper)
  int n = static_cast<int>(nodes_.size());
  std::vector<double> dist(n, std::numeric_limits<double>::max());
  std::vector<int> pred(n, -1);
  dist[start_idx] = 0.0;

  // Min-heap: (distance, node)
  using PQEntry = std::pair<double, int>;
  std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<>> pq;
  pq.push({0.0, start_idx});

  while (!pq.empty())
  {
    auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[u])
      continue;
    if (u == end_idx)
      break;

    for (const auto& edge : adj_[u])
    {
      double nd = dist[u] + edge.weight;
      if (nd < dist[edge.to])
      {
        dist[edge.to] = nd;
        pred[edge.to] = u;
        pq.push({nd, edge.to});
      }
    }
  }

  if (dist[end_idx] == std::numeric_limits<double>::max())
  {
    return {start, end};  // No path — straight line fallback
  }

  // Reconstruct
  Path2D path;
  for (int cur = end_idx; cur != -1; cur = pred[cur])
  {
    path.push_back(nodes_[cur]);
  }
  std::reverse(path.begin(), path.end());

  // Prepend start, append end
  path.insert(path.begin(), start);
  path.push_back(end);
  return path;
}

std::vector<std::pair<Point2D, Point2D>> VoronoiRoadmap::get_edges() const
{
  std::vector<std::pair<Point2D, Point2D>> edges;
  std::set<std::pair<int, int>> seen;
  for (int i = 0; i < static_cast<int>(adj_.size()); ++i)
  {
    for (const auto& e : adj_[i])
    {
      auto key = std::make_pair(std::min(i, e.to), std::max(i, e.to));
      if (seen.insert(key).second)
      {
        edges.push_back({nodes_[i], nodes_[e.to]});
      }
    }
  }
  return edges;
}

}  // namespace brv
