#pragma once

#include <functional>
#include <string>
#include <vector>

#include "mowgli_brv_planner/types.hpp"

namespace brv
{

struct PlannerParams
{
  double tool_width = 0.18;
  int headland_passes = 2;
  double headland_width = 0.18;
  double mow_angle_deg = -1.0;  // -1 = auto via MBB
  double voronoi_sample_spacing = 0.18;
  int voronoi_knn = 10;
  double min_obstacle_area = 0.01;
};

// Progress callback: (percent, phase_name)
using ProgressCallback = std::function<void(float, const std::string&)>;

// Main B-RV planner entry point.
CoverageResult plan_coverage(const Polygon2D& boundary,
                             const std::vector<Polygon2D>& obstacles,
                             const PlannerParams& params,
                             ProgressCallback progress_cb = nullptr);

}  // namespace brv
