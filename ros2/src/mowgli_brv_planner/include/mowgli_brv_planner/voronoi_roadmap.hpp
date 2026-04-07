#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "mowgli_brv_planner/types.hpp"

namespace brv
{

class VoronoiRoadmap
{
public:
  VoronoiRoadmap(const Polygon2D& boundary,
                 const std::vector<Polygon2D>& obstacles,
                 double sample_spacing,
                 int knn);

  // Find shortest path from start to end via the roadmap.
  Path2D find_path(const Point2D& start, const Point2D& end) const;

  // Get all edges for visualization.
  std::vector<std::pair<Point2D, Point2D>> get_edges() const;

  // Get all nodes for visualization.
  const std::vector<Point2D>& nodes() const
  {
    return nodes_;
  }

private:
  void build_from_voronoi(const Polygon2D& boundary,
                          const std::vector<Polygon2D>& obstacles,
                          double sample_spacing,
                          int knn);

  // Adjacency list: node_idx -> [(neighbor_idx, distance)]
  struct Edge
  {
    int to;
    double weight;
  };
  std::vector<std::vector<Edge>> adj_;
  std::vector<Point2D> nodes_;
  Polygon2D boundary_;
  std::vector<Polygon2D> obstacles_;
};

}  // namespace brv
