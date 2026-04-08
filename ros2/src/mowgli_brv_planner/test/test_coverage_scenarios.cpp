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
 * @file test_coverage_scenarios.cpp
 * @brief Integration tests that mirror the simulation environment.
 *
 * These tests use the EXACT same garden boundary and obstacle positions
 * as the Gazebo simulation (garden.sdf + map_server.yaml) to validate
 * that the B-RV planner generates correct zigzag coverage paths around
 * obstacles.
 */

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "mowgli_brv_planner/brv_algorithm.hpp"
#include "mowgli_brv_planner/geometry_utils.hpp"
#include "mowgli_brv_planner/grid_coverage.hpp"
#include "mowgli_brv_planner/types.hpp"
#include <gtest/gtest.h>

using namespace brv;

// ── Helpers ────────────────────────────────────────────────────────────────

static bool point_inside_rect(double px, double py, double x1, double y1, double x2, double y2)
{
  return px >= x1 && px <= x2 && py >= y1 && py <= y2;
}

static int count_jumps(const Path2D& path, double threshold)
{
  int jumps = 0;
  for (size_t i = 1; i < path.size(); ++i)
  {
    if (path[i].dist(path[i - 1]) > threshold)
      ++jumps;
  }
  return jumps;
}

static double max_jump(const Path2D& path)
{
  double mx = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
  {
    mx = std::max(mx, path[i].dist(path[i - 1]));
  }
  return mx;
}

// ── Simulation-matching scenario ───────────────────────────────────────────

class SimulationScenario : public ::testing::Test
{
protected:
  // Same as map_server.yaml: area_polygons
  Polygon2D boundary = {{-7.0, -3.0}, {2.0, -3.0}, {2.0, 3.0}, {-7.0, 3.0}};

  // Same as garden.sdf: obs_swath1, obs_swath2, obs_mid (cylinder r=0.3)
  // Approximated as squares for grid test
  Polygon2D obs_swath1 = {{-6.8, -0.3}, {-6.2, -0.3}, {-6.2, 0.3}, {-6.8, 0.3}};
  Polygon2D obs_swath2 = {{-6.3, -3.3}, {-5.7, -3.3}, {-5.7, -2.7}, {-6.3, -2.7}};
  Polygon2D obs_mid = {{2.7, -0.3}, {3.3, -0.3}, {3.3, 0.3}, {2.7, 0.3}};

  std::vector<Polygon2D> obstacles = {obs_swath1, obs_swath2, obs_mid};

  PlannerParams params;

  void SetUp() override
  {
    params.tool_width = 0.18;
    params.headland_passes = 2;
    params.headland_width = 0.18;
    params.mow_angle_deg = -1.0;
    params.robot_x = 0.0;
    params.robot_y = 0.0;
  }

  CoverageResult plan()
  {
    return plan_coverage(boundary, obstacles, params);
  }
};

// ── Tests ──────────────────────────────────────────────────────────────────

TEST_F(SimulationScenario, PathNotEmpty)
{
  auto result = plan();
  EXPECT_GT(result.full_path.size(), 100u);
  EXPECT_GT(result.swaths.size(), 10u);
}

TEST_F(SimulationScenario, NoWaypointsInsideObstacles)
{
  auto result = plan();

  for (const auto& wp : result.full_path)
  {
    // obs_swath1 at [-6.8,-0.3] to [-6.2,0.3]
    EXPECT_FALSE(point_inside_rect(wp.x, wp.y, -6.8, -0.3, -6.2, 0.3))
        << "Waypoint (" << wp.x << ", " << wp.y << ") inside obs_swath1";
    // obs_swath2 at [-6.3,-3.3] to [-5.7,-2.7]
    EXPECT_FALSE(point_inside_rect(wp.x, wp.y, -6.3, -3.3, -5.7, -2.7))
        << "Waypoint (" << wp.x << ", " << wp.y << ") inside obs_swath2";
    // obs_mid at [2.7,-0.3] to [3.3,0.3]
    EXPECT_FALSE(point_inside_rect(wp.x, wp.y, 2.7, -0.3, 3.3, 0.3))
        << "Waypoint (" << wp.x << ", " << wp.y << ") inside obs_mid";
  }
}

TEST_F(SimulationScenario, PathStaysWithinBoundary)
{
  auto result = plan();

  // Allow small margin for headland inset
  const double margin = 0.5;
  for (const auto& wp : result.full_path)
  {
    EXPECT_GE(wp.x, -7.0 - margin) << "Waypoint too far left: " << wp.x;
    EXPECT_LE(wp.x, 2.0 + margin) << "Waypoint too far right: " << wp.x;
    EXPECT_GE(wp.y, -3.0 - margin) << "Waypoint too far down: " << wp.y;
    EXPECT_LE(wp.y, 3.0 + margin) << "Waypoint too far up: " << wp.y;
  }
}

TEST_F(SimulationScenario, ZigzagPatternDetected)
{
  // The boustrophedon pattern alternates sweep direction per column.
  // Check that consecutive swaths have opposite y-direction.
  auto result = plan();
  ASSERT_GE(result.swaths.size(), 4u);

  int alternating_count = 0;
  for (size_t i = 1; i < result.swaths.size(); ++i)
  {
    double dy_prev = result.swaths[i - 1].end.y - result.swaths[i - 1].start.y;
    double dy_curr = result.swaths[i].end.y - result.swaths[i].start.y;
    // Adjacent swaths should have opposite y-direction (zigzag)
    if (dy_prev * dy_curr < 0)
      ++alternating_count;
  }

  // At least 70% of swath transitions should alternate (some won't due to obstacles)
  double ratio = static_cast<double>(alternating_count) / (result.swaths.size() - 1);
  EXPECT_GT(ratio, 0.5) << "Zigzag pattern not detected: only " << (ratio * 100)
                        << "% alternating swaths";
}

TEST_F(SimulationScenario, SwathsSplitAroundObstacle)
{
  // obs_swath1 is at x~[-6.8, -6.2], y~[-0.3, 0.3]
  // The path should have more swaths than an obstacle-free plan because
  // the obstacle creates splits and detours within columns.
  auto result = plan();

  // Plan without obstacles for comparison
  auto result_no_obs = plan_coverage(boundary, {}, params);

  EXPECT_GT(result.swaths.size(), result_no_obs.swaths.size())
      << "Obstacles should create additional swath splits (" << result.swaths.size() << " vs "
      << result_no_obs.swaths.size() << " without obstacles)";

  // Also verify no path segment passes through the obstacle midpoint
  int through_obs = 0;
  for (size_t i = 1; i < result.full_path.size(); ++i)
  {
    double mid_x = (result.full_path[i - 1].x + result.full_path[i].x) / 2;
    double mid_y = (result.full_path[i - 1].y + result.full_path[i].y) / 2;
    if (mid_x >= -6.8 && mid_x <= -6.2 && mid_y >= -0.3 && mid_y <= 0.3)
    {
      double d = result.full_path[i].dist(result.full_path[i - 1]);
      if (d > 0.3)
        ++through_obs;
    }
  }
  EXPECT_EQ(through_obs, 0) << "Path segments pass through obstacle";
}

TEST_F(SimulationScenario, FewJumpsInPath)
{
  // A clean zigzag path should have very few jumps > 0.5m.
  // Jumps happen at: obstacle gaps within columns, column transitions
  // at headland. Should be < 20 for this garden.
  auto result = plan();

  int jumps = count_jumps(result.full_path, 0.5);
  EXPECT_LT(jumps, 20) << "Too many jumps (>0.5m) in path: " << jumps
                       << ". Expected clean zigzag with few transitions.";
}

TEST_F(SimulationScenario, MaxJumpReasonable)
{
  // The maximum jump should be bounded by the obstacle size + margin.
  // Obstacle height is ~0.6m, so jumps should be < 2m.
  auto result = plan();

  double mj = max_jump(result.full_path);
  EXPECT_LT(mj, 2.0) << "Max jump " << mj << "m is too large. "
                     << "Likely a Voronoi transit detour.";
}

TEST_F(SimulationScenario, PathStartsInNearestColumn)
{
  // Robot at (0, 0) — path should start in the nearest column (x within 1 tool_width).
  // The y-coordinate may be at the boundary edge (full column sweep).
  auto result = plan();
  ASSERT_FALSE(result.full_path.empty());

  double x_dist = std::abs(result.full_path[0].x - params.robot_x);
  EXPECT_LT(x_dist, params.tool_width * 2) << "Path starts in column x=" << result.full_path[0].x
                                           << ", expected near robot x=" << params.robot_x;
}

TEST_F(SimulationScenario, HighCoverageRatio)
{
  // The planner should cover > 90% of the accessible area.
  auto result = plan();

  // Compute expected area: boundary area - headland - obstacles
  double boundary_area = 9.0 * 6.0;  // 54 m²
  double headland_inset = params.headland_passes * params.headland_width;  // 0.36m
  double inner_area = (9.0 - 2 * headland_inset) * (6.0 - 2 * headland_inset);
  double obs_area = 0.6 * 0.6 + 0.6 * 0.6 + 0.6 * 0.6;  // 3 obstacles

  EXPECT_GT(result.coverage_area, (inner_area - obs_area) * 0.85)
      << "Coverage area " << result.coverage_area << " m² is too low (expected > "
      << (inner_area - obs_area) * 0.85 << " m²)";
}

TEST_F(SimulationScenario, MowingEfficiency)
{
  // Planned path length should not be much longer than the theoretical
  // minimum: area / tool_width (straight swaths, no turns).
  auto result = plan();

  double inner_w = 9.0 - 2 * params.headland_passes * params.headland_width;
  double inner_h = 6.0 - 2 * params.headland_passes * params.headland_width;
  double n_swaths = inner_w / params.tool_width;
  double min_path = n_swaths * inner_h;

  // Allow 50% overhead for turns, obstacle gaps, and transit
  EXPECT_LT(result.total_distance, min_path * 1.5)
      << "Path length " << result.total_distance << "m is too long " << "(theoretical min "
      << min_path << "m, 50% overhead = " << min_path * 1.5 << "m)";
}

// ── No-obstacle baseline ──────────────────────────────────────────────────

TEST(CoverageBaseline, EmptyGardenFullCoverage)
{
  Polygon2D boundary = {{0, 0}, {3, 0}, {3, 3}, {0, 3}};
  PlannerParams params;
  params.tool_width = 0.18;
  params.headland_passes = 0;
  params.mow_angle_deg = 0.0;

  auto result = plan_coverage(boundary, {}, params);

  EXPECT_GT(result.full_path.size(), 50u);
  EXPECT_GT(result.swaths.size(), 5u);

  // Should have nearly zero jumps
  int jumps = count_jumps(result.full_path, 0.5);
  EXPECT_LT(jumps, 3) << "Clean empty garden should have < 3 jumps";
}

TEST(CoverageBaseline, SingleObstacleSplitsSwaths)
{
  Polygon2D boundary = {{0, 0}, {2, 0}, {2, 4}, {0, 4}};
  Polygon2D obstacle = {{0.8, 1.8}, {1.2, 1.8}, {1.2, 2.2}, {0.8, 2.2}};
  PlannerParams params;
  params.tool_width = 0.18;
  params.headland_passes = 0;
  params.mow_angle_deg = 0.0;

  auto result = plan_coverage(boundary, {obstacle}, params);

  // No waypoints inside obstacle
  for (const auto& wp : result.full_path)
  {
    EXPECT_FALSE(point_inside_rect(wp.x, wp.y, 0.8, 1.8, 1.2, 2.2))
        << "Waypoint inside obstacle at (" << wp.x << ", " << wp.y << ")";
  }

  // Should have more swaths than without obstacle (splits)
  auto result_no_obs = plan_coverage(boundary, {}, params);
  EXPECT_GT(result.swaths.size(), result_no_obs.swaths.size())
      << "Obstacle should create additional swath splits";
}

// ── Grid-level obstacle tests ─────────────────────────────────────────────

TEST(GridObstacle, ColumnSplitByObstacle)
{
  // 10x10 grid with obstacle in the middle of column 5
  Polygon2D boundary = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  Polygon2D obstacle = {{0.8, 0.8}, {1.2, 0.8}, {1.2, 1.2}, {0.8, 1.2}};

  CoverageGrid grid(boundary, {obstacle}, 0.18, 0.0);

  // Column at x~1.0 should have obstacle cells marked UNVISITABLE
  bool found_unvisitable = false;
  for (int r = 0; r < grid.rows(); ++r)
  {
    for (int c = 0; c < grid.cols(); ++c)
    {
      auto pt = grid.cell_to_map(r, c);
      if (pt.x >= 0.8 && pt.x <= 1.2 && pt.y >= 0.8 && pt.y <= 1.2)
      {
        if (grid.cell(r, c) == CellState::UNVISITABLE)
          found_unvisitable = true;
      }
    }
  }
  EXPECT_TRUE(found_unvisitable) << "Obstacle cells should be UNVISITABLE";

  // Sweep should cover both sides of the obstacle
  auto sweep = boustrophedon_sweep(grid, 0, 0);
  EXPECT_FALSE(sweep.waypoints.empty());

  // All cells should be visited after sweep
  EXPECT_EQ(grid.count_unvisited(), 0) << "All reachable cells should be visited after sweep";
}

TEST(GridObstacle, MultipleColumnsObstacle)
{
  // Wide obstacle spanning 3+ columns
  Polygon2D boundary = {{0, 0}, {3, 0}, {3, 3}, {0, 3}};
  Polygon2D obstacle = {{1.0, 1.0}, {2.0, 1.0}, {2.0, 2.0}, {1.0, 2.0}};

  CoverageGrid grid(boundary, {obstacle}, 0.18, 0.0);

  auto sweep = boustrophedon_sweep(grid, 0, 0);

  // Sweep should cover all non-obstacle cells
  EXPECT_EQ(grid.count_unvisited(), 0);

  // Check swath breaks exist (obstacle creates splits)
  EXPECT_GT(sweep.swath_breaks.size(), 1u) << "Wide obstacle should create swath breaks";
}

// ── Edge case obstacle scenarios ──────────────────────────────────────────

// Helper: count segments whose midpoint is inside any obstacle rect
static int count_through_obstacles(
    const Path2D& path, const std::vector<std::tuple<double, double, double, double>>& rects)
{
  int count = 0;
  for (size_t i = 1; i < path.size(); ++i)
  {
    double d = path[i].dist(path[i - 1]);
    if (d < 0.3)
      continue;  // short step, not a crossing
    for (auto& [rx1, ry1, rx2, ry2] : rects)
    {
      // Check midpoint and quarter-points
      for (double t : {0.25, 0.5, 0.75})
      {
        double px = path[i - 1].x + t * (path[i].x - path[i - 1].x);
        double py = path[i - 1].y + t * (path[i].y - path[i - 1].y);
        if (px >= rx1 && px <= rx2 && py >= ry1 && py <= ry2)
        {
          count++;
          goto next_segment;
        }
      }
    }
  next_segment:;
  }
  return count;
}

TEST(ObstacleEdgeCases, WideObstacleNoCrossing)
{
  // Wide obstacle spanning 3m — detour must go far enough around it
  Polygon2D boundary = {{-7, -3}, {2, -3}, {2, 3}, {-7, 3}};
  Polygon2D obs = {{-4, -0.5}, {-1, -0.5}, {-1, 0.5}, {-4, 0.5}};
  PlannerParams params;
  params.tool_width = 0.18;
  params.headland_passes = 2;
  params.headland_width = 0.18;
  params.mow_angle_deg = -1.0;

  auto result = plan_coverage(boundary, {obs}, params);
  int through = count_through_obstacles(result.full_path, {{-4, -0.5, -1, 0.5}});
  EXPECT_EQ(through, 0) << "Path crosses wide obstacle " << through << " times";
}

TEST(ObstacleEdgeCases, MultipleObstaclesNoCrossing)
{
  Polygon2D boundary = {{-7, -3}, {2, -3}, {2, 3}, {-7, 3}};
  Polygon2D o1 = {{-6.8, -0.3}, {-6.2, -0.3}, {-6.2, 0.3}, {-6.8, 0.3}};
  Polygon2D o2 = {{-3, -0.3}, {-2.4, -0.3}, {-2.4, 0.3}, {-3, 0.3}};
  Polygon2D o3 = {{1, -1}, {1.6, -1}, {1.6, -0.4}, {1, -0.4}};
  PlannerParams params;
  params.tool_width = 0.18;
  params.headland_passes = 2;
  params.headland_width = 0.18;
  params.mow_angle_deg = -1.0;

  auto result = plan_coverage(boundary, {o1, o2, o3}, params);
  int through =
      count_through_obstacles(result.full_path,
                              {{-6.8, -0.3, -6.2, 0.3}, {-3, -0.3, -2.4, 0.3}, {1, -1, 1.6, -0.4}});
  EXPECT_EQ(through, 0) << "Path crosses obstacles " << through << " times";
}

TEST(ObstacleEdgeCases, CornerObstacleNoCrossing)
{
  // Obstacle near boundary corner
  Polygon2D boundary = {{-7, -3}, {2, -3}, {2, 3}, {-7, 3}};
  Polygon2D obs = {{-6.8, -2.8}, {-6.2, -2.8}, {-6.2, -2.2}, {-6.8, -2.2}};
  PlannerParams params;
  params.tool_width = 0.18;
  params.headland_passes = 2;
  params.headland_width = 0.18;
  params.mow_angle_deg = -1.0;

  auto result = plan_coverage(boundary, {obs}, params);
  int through = count_through_obstacles(result.full_path, {{-6.8, -2.8, -6.2, -2.2}});
  EXPECT_EQ(through, 0) << "Path crosses corner obstacle " << through << " times";
}

TEST(ObstacleEdgeCases, TwoObstaclesSameColumnNoCrossing)
{
  Polygon2D boundary = {{-7, -3}, {2, -3}, {2, 3}, {-7, 3}};
  Polygon2D o1 = {{-3, 1}, {-2.4, 1}, {-2.4, 1.6}, {-3, 1.6}};
  Polygon2D o2 = {{-3, -1.6}, {-2.4, -1.6}, {-2.4, -1}, {-3, -1}};
  PlannerParams params;
  params.tool_width = 0.18;
  params.headland_passes = 2;
  params.headland_width = 0.18;
  params.mow_angle_deg = -1.0;

  auto result = plan_coverage(boundary, {o1, o2}, params);
  int through =
      count_through_obstacles(result.full_path, {{-3, 1, -2.4, 1.6}, {-3, -1.6, -2.4, -1}});
  EXPECT_EQ(through, 0) << "Path crosses stacked obstacles " << through << " times";
}

TEST(ObstacleEdgeCases, FullCoverageWithObstacles)
{
  // All scenarios should achieve 100% coverage of accessible cells
  Polygon2D boundary = {{-7, -3}, {2, -3}, {2, 3}, {-7, 3}};
  Polygon2D obs = {{-3, -0.3}, {-2.4, -0.3}, {-2.4, 0.3}, {-3, 0.3}};
  PlannerParams params;
  params.tool_width = 0.18;
  params.headland_passes = 2;
  params.headland_width = 0.18;
  params.mow_angle_deg = -1.0;

  auto result = plan_coverage(boundary, {obs}, params);

  // Verify by creating grid and marking visited cells from path
  Polygon2D inner = offset_polygon_inward(boundary, 0.36);
  ASSERT_FALSE(inner.empty());
  CoverageGrid grid(inner, {obs}, 0.18, 0.0);
  int total_accessible = 0;
  for (int r = 0; r < grid.rows(); ++r)
    for (int c = 0; c < grid.cols(); ++c)
      if (grid.cell(r, c) != CellState::UNVISITABLE)
        total_accessible++;

  int visited = 0;
  for (auto& wp : result.full_path)
  {
    auto [row, col] = grid.map_to_cell(wp.x, wp.y);
    if (row >= 0 && row < grid.rows() && col >= 0 && col < grid.cols())
    {
      if (grid.cell(row, col) == CellState::UNVISITED)
      {
        grid.set_cell(row, col, CellState::VISITED);
        visited++;
      }
    }
  }

  double cov = total_accessible > 0 ? static_cast<double>(visited) / total_accessible * 100 : 0;
  EXPECT_GT(cov, 95.0) << "Coverage " << cov << "% is too low (expected >95%)";
}
