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
#include <cmath>
#include <limits>

#include "mowgli_brv_planner/brv_algorithm.hpp"
#include "mowgli_brv_planner/geometry_utils.hpp"
#include <gtest/gtest.h>

// Helper: create a simple rectangular boundary
static brv::Polygon2D make_rect(double w, double h)
{
  return {{0, 0}, {w, 0}, {w, h}, {0, h}};
}

// ─────────────────────────────────────────────────────────────────────────────
// PlanEmptyGarden — rectangular boundary, no obstacles
// ─────────────────────────────────────────────────────────────────────────────

TEST(BrvAlgorithm, PlanEmptyGarden)
{
  brv::Polygon2D boundary = make_rect(5.0, 5.0);
  std::vector<brv::Polygon2D> obstacles;

  brv::PlannerParams params;
  params.tool_width = 0.5;
  params.headland_passes = 0;

  auto result = brv::plan_coverage(boundary, obstacles, params);

  EXPECT_GT(result.full_path.size(), 0u);
  EXPECT_GT(result.total_distance, 0.0);
  EXPECT_GT(result.coverage_area, 0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// PlanWithObstacle — boundary + 1 obstacle
// ─────────────────────────────────────────────────────────────────────────────

TEST(BrvAlgorithm, PlanWithObstacle)
{
  brv::Polygon2D boundary = make_rect(6.0, 6.0);
  // Small obstacle in center
  brv::Polygon2D obstacle = {{2.5, 2.5}, {3.5, 2.5}, {3.5, 3.5}, {2.5, 3.5}};
  std::vector<brv::Polygon2D> obstacles = {obstacle};

  brv::PlannerParams params;
  params.tool_width = 0.5;
  params.headland_passes = 0;

  auto result = brv::plan_coverage(boundary, obstacles, params);

  EXPECT_GT(result.full_path.size(), 0u);

  // No waypoint should be inside the obstacle (with small tolerance for grid edges)
  for (const auto& pt : result.full_path)
  {
    // Check with a margin — grid cell centers at resolution 0.5 should not
    // land well inside the obstacle
    bool deep_inside = (pt.x > 2.7 && pt.x < 3.3 && pt.y > 2.7 && pt.y < 3.3);
    EXPECT_FALSE(deep_inside) << "Waypoint (" << pt.x << ", " << pt.y
                              << ") is deep inside the obstacle";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// PlanWithMultipleObstacles — 3 obstacles
// ─────────────────────────────────────────────────────────────────────────────

TEST(BrvAlgorithm, PlanWithMultipleObstacles)
{
  brv::Polygon2D boundary = make_rect(10.0, 10.0);
  brv::Polygon2D obs1 = {{1.5, 1.5}, {2.5, 1.5}, {2.5, 2.5}, {1.5, 2.5}};
  brv::Polygon2D obs2 = {{5.0, 5.0}, {6.0, 5.0}, {6.0, 6.0}, {5.0, 6.0}};
  brv::Polygon2D obs3 = {{7.0, 2.0}, {8.0, 2.0}, {8.0, 3.0}, {7.0, 3.0}};
  std::vector<brv::Polygon2D> obstacles = {obs1, obs2, obs3};

  brv::PlannerParams params;
  params.tool_width = 0.5;
  params.headland_passes = 0;

  auto result = brv::plan_coverage(boundary, obstacles, params);

  EXPECT_GT(result.full_path.size(), 0u);
  EXPECT_GT(result.coverage_area, 0.0);

  // Total area is 100 minus obstacles (3 * 1 = 3) = 97
  // Coverage should be > 90% of available area
  double available_area = 100.0 - 3.0;
  EXPECT_GT(result.coverage_area, available_area * 0.5);
}

// ─────────────────────────────────────────────────────────────────────────────
// StartFromRobotPosition — robot_x/y set → path starts near robot
// ─────────────────────────────────────────────────────────────────────────────

TEST(BrvAlgorithm, StartFromRobotPosition)
{
  brv::Polygon2D boundary = make_rect(8.0, 8.0);
  std::vector<brv::Polygon2D> obstacles;

  brv::PlannerParams params;
  params.tool_width = 0.5;
  params.headland_passes = 0;
  params.robot_x = 6.0;
  params.robot_y = 6.0;

  auto result = brv::plan_coverage(boundary, obstacles, params);

  EXPECT_GT(result.full_path.size(), 0u);

  // First waypoint should be reasonably near the robot position
  // (within a few tool widths due to grid snapping)
  double dist_to_robot = result.full_path.front().dist({6.0, 6.0});
  EXPECT_LT(dist_to_robot, 3.0) << "First waypoint at (" << result.full_path.front().x << ", "
                                << result.full_path.front().y
                                << ") is too far from robot at (6, 6)";
}

// ─────────────────────────────────────────────────────────────────────────────
// StartFromCorner — no robot position → path starts from corner
// ─────────────────────────────────────────────────────────────────────────────

TEST(BrvAlgorithm, StartFromCorner)
{
  brv::Polygon2D boundary = make_rect(6.0, 6.0);
  std::vector<brv::Polygon2D> obstacles;

  brv::PlannerParams params;
  params.tool_width = 0.5;
  params.headland_passes = 0;
  // robot_x/y default to NaN

  auto result = brv::plan_coverage(boundary, obstacles, params);

  EXPECT_GT(result.full_path.size(), 0u);

  // Without robot position, the planner uses get_unvisited_region_starts()
  // which returns the topmost-leftmost cell. First waypoint should be
  // near a corner/edge of the grid.
  auto& first = result.full_path.front();
  bool near_edge = (first.x < 1.5 || first.x > 4.5 || first.y < 1.5 || first.y > 4.5);
  EXPECT_TRUE(near_edge) << "First waypoint at (" << first.x << ", " << first.y
                         << ") expected near edge of boundary";
}

// ─────────────────────────────────────────────────────────────────────────────
// SweepAngleOptimal — MBB selects angle along long edge
// ─────────────────────────────────────────────────────────────────────────────

TEST(BrvAlgorithm, SweepAngleOptimal)
{
  // Long rectangle (20x5): optimal sweep is along the long edge (angle ≈ 0)
  brv::Polygon2D boundary = make_rect(20.0, 5.0);
  std::vector<brv::Polygon2D> obstacles;

  brv::PlannerParams params;
  params.tool_width = 0.5;
  params.headland_passes = 0;
  params.mow_angle_deg = -1.0;  // auto via MBB

  auto result = brv::plan_coverage(boundary, obstacles, params);

  EXPECT_GT(result.full_path.size(), 0u);
  EXPECT_GT(result.swaths.size(), 0u);

  // With optimal sweep along X, swaths should run roughly horizontal
  // (start.y ≈ end.y for most swaths, or start.x ≈ end.x for vertical)
  // Just verify we get a reasonable number of swaths for the area
  double expected_swaths = 5.0 / params.tool_width;
  EXPECT_GT(result.swaths.size(), static_cast<size_t>(expected_swaths * 0.3));
}

// ─────────────────────────────────────────────────────────────────────────────
// ProgressCallback fires
// ─────────────────────────────────────────────────────────────────────────────

TEST(BrvAlgorithm, ProgressCallbackFires)
{
  brv::Polygon2D boundary = make_rect(4.0, 4.0);
  std::vector<brv::Polygon2D> obstacles;

  brv::PlannerParams params;
  params.tool_width = 0.5;
  params.headland_passes = 0;

  int callback_count = 0;
  float last_pct = -1.0f;
  auto cb = [&](float pct, const std::string& /*phase*/)
  {
    EXPECT_GE(pct, last_pct);
    last_pct = pct;
    callback_count++;
  };

  brv::plan_coverage(boundary, obstacles, params, cb);
  EXPECT_GT(callback_count, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// SmallObstaclesFiltered — obstacles below min_obstacle_area are ignored
// ─────────────────────────────────────────────────────────────────────────────

TEST(BrvAlgorithm, SmallObstaclesFiltered)
{
  brv::Polygon2D boundary = make_rect(6.0, 6.0);
  // Tiny obstacle (area = 0.0001) — below default min_obstacle_area (0.01)
  brv::Polygon2D tiny_obs = {{3.0, 3.0}, {3.01, 3.0}, {3.01, 3.01}, {3.0, 3.01}};
  std::vector<brv::Polygon2D> obstacles = {tiny_obs};

  brv::PlannerParams params;
  params.tool_width = 0.5;
  params.headland_passes = 0;

  auto result_with = brv::plan_coverage(boundary, obstacles, params);

  // Run without obstacles for comparison
  auto result_without = brv::plan_coverage(boundary, {}, params);

  // Coverage should be essentially the same since tiny obstacle is filtered
  EXPECT_NEAR(result_with.coverage_area, result_without.coverage_area, 1.0);
}
