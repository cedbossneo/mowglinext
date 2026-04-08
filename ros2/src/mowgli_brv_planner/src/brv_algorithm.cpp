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
#include "mowgli_brv_planner/brv_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "mowgli_brv_planner/geometry_utils.hpp"
#include "mowgli_brv_planner/grid_coverage.hpp"
#include "mowgli_brv_planner/mbb.hpp"
#include "mowgli_brv_planner/voronoi_roadmap.hpp"

namespace brv
{

CoverageResult plan_coverage(const Polygon2D& boundary,
                             const std::vector<Polygon2D>& obstacles,
                             const PlannerParams& params,
                             ProgressCallback progress_cb)
{
  CoverageResult result;

  auto report = [&progress_cb](float pct, const std::string& phase)
  {
    if (progress_cb)
      progress_cb(pct, phase);
  };

  // Filter small obstacles
  std::vector<Polygon2D> filtered_obs;
  for (const auto& obs : obstacles)
  {
    if (std::abs(polygon_area(obs)) >= params.min_obstacle_area)
    {
      filtered_obs.push_back(obs);
    }
  }

  // 1. Generate headland outline
  report(5.0f, "headland");
  Polygon2D inner_boundary = boundary;
  if (params.headland_passes > 0)
  {
    for (int pass = 0; pass < params.headland_passes; ++pass)
    {
      double inset = params.headland_width * (pass + 0.5);
      Polygon2D ring = offset_polygon_inward(boundary, inset);
      if (ring.empty())
        break;
      // Add outline waypoints
      for (const auto& pt : ring)
      {
        result.outline_path.push_back(pt);
      }
      // Close the ring
      if (!ring.empty())
      {
        result.outline_path.push_back(ring.front());
      }
    }
    // Compute inner boundary for fill
    Polygon2D inset =
        offset_polygon_inward(boundary, params.headland_passes * params.headland_width);
    if (!inset.empty())
    {
      inner_boundary = inset;
    }
  }

  // 2. Determine sweep angle via MBB (Algorithm 1)
  report(10.0f, "decomposition");
  double sweep_angle = params.mow_angle_deg;
  if (sweep_angle < 0)
  {
    sweep_angle = compute_mbb_angle(inner_boundary);
  }
  else
  {
    sweep_angle = sweep_angle * M_PI / 180.0;
  }

  // 3. Build coverage grid (Section 4.3)
  report(20.0f, "swaths");
  CoverageGrid grid(inner_boundary, filtered_obs, params.tool_width, sweep_angle);

  // 4. Build Voronoi roadmap (Section 4.4)
  report(30.0f, "routing");
  VoronoiRoadmap roadmap(inner_boundary,
                         filtered_obs,
                         params.voronoi_sample_spacing,
                         params.voronoi_knn);

  // 5. Execute B-RV coverage (main loop)
  report(40.0f, "path_planning");

  int total_cells = grid.count_unvisited();
  if (total_cells == 0)
  {
    result.coverage_area = 0.0;
    return result;
  }

  // Find initial start — nearest unvisited cell to robot position,
  // or top-left corner if robot position is not provided.
  int cur_row, cur_col;
  if (!std::isnan(params.robot_x) && !std::isnan(params.robot_y))
  {
    auto [approx_row, approx_col] = grid.map_to_cell(params.robot_x, params.robot_y);
    approx_row = std::clamp(approx_row, 0, grid.rows() - 1);
    approx_col = std::clamp(approx_col, 0, grid.cols() - 1);
    auto [nr, nc] = grid.find_nearest_unvisited(approx_row, approx_col);
    if (nr < 0)
      return result;
    cur_row = nr;
    cur_col = nc;
  }
  else
  {
    auto regions = grid.get_unvisited_region_starts();
    if (regions.empty())
      return result;
    cur_row = regions[0].first;
    cur_col = regions[0].second;
  }
  Point2D cur_pos = grid.cell_to_map(cur_row, cur_col);

  int iteration = 0;
  int max_iterations = total_cells + 100;  // safety limit

  while (grid.has_unvisited() && iteration++ < max_iterations)
  {
    // Boustrophedon sweep from current position
    SweepResult sweep = boustrophedon_sweep(grid, cur_row, cur_col);

    if (!sweep.waypoints.empty())
    {
      // Record swath info
      for (size_t si = 0; si < sweep.swath_breaks.size(); ++si)
      {
        int start_idx = sweep.swath_breaks[si];
        int end_idx = (si + 1 < sweep.swath_breaks.size())
                          ? sweep.swath_breaks[si + 1] - 1
                          : static_cast<int>(sweep.waypoints.size()) - 1;
        if (start_idx <= end_idx && end_idx < static_cast<int>(sweep.waypoints.size()))
        {
          SwathInfo swath;
          swath.start = sweep.waypoints[start_idx];
          swath.end = sweep.waypoints[end_idx];
          result.swaths.push_back(swath);
        }
      }

      // Append coverage waypoints to full path
      result.full_path.insert(result.full_path.end(),
                              sweep.waypoints.begin(),
                              sweep.waypoints.end());
      cur_pos = sweep.waypoints.back();
      cur_row = sweep.end_row;
      cur_col = sweep.end_col;
    }

    // Check if more unvisited regions exist
    if (!grid.has_unvisited())
      break;

    // Find nearest unvisited region and transit via Voronoi
    auto [next_row, next_col] = grid.find_nearest_unvisited(cur_row, cur_col);
    if (next_row < 0)
      break;

    Point2D next_pos = grid.cell_to_map(next_row, next_col);

    // Use Voronoi roadmap for collision-free transit (Section 4.4)
    Path2D transit = roadmap.find_path(cur_pos, next_pos);
    if (transit.size() > 2)
    {
      // Skip first (== cur_pos) and last (== next_pos) to avoid duplicates
      for (size_t i = 1; i + 1 < transit.size(); ++i)
      {
        result.full_path.push_back(transit[i]);
      }
    }

    cur_pos = next_pos;
    cur_row = next_row;
    cur_col = next_col;

    // Report progress
    int remaining = grid.count_unvisited();
    float pct = 40.0f + 60.0f * (1.0f - static_cast<float>(remaining) / total_cells);
    report(pct, "path_planning");
  }

  // Compute metrics
  result.total_distance = path_length(result.full_path);
  int visited_cells = total_cells - grid.count_unvisited();
  result.coverage_area = visited_cells * params.tool_width * params.tool_width;

  report(100.0f, "path_planning");
  return result;
}

}  // namespace brv
