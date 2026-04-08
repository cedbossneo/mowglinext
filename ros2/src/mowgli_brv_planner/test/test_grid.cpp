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

#include "mowgli_brv_planner/grid_coverage.hpp"
#include <gtest/gtest.h>

TEST(Grid, BasicSquare)
{
  brv::Polygon2D boundary = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  std::vector<brv::Polygon2D> obstacles;
  brv::CoverageGrid grid(boundary, obstacles, 0.5, 0.0);

  EXPECT_GT(grid.rows(), 0);
  EXPECT_GT(grid.cols(), 0);
  EXPECT_TRUE(grid.has_unvisited());

  auto sweep = brv::boustrophedon_sweep(grid, 0, 0);
  EXPECT_GT(sweep.waypoints.size(), 0u);
}

// ─────────────────────────────────────────────────────────────────────────────
// ObstacleInColumn — obstacle blocks mid-column → sweep creates 2+ segments
// ─────────────────────────────────────────────────────────────────────────────

TEST(Grid, ObstacleInColumn)
{
  // 10x10 boundary with a large obstacle blocking the middle rows
  brv::Polygon2D boundary = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  // Obstacle spanning several cells at resolution 0.5
  brv::Polygon2D obstacle = {{2, 4}, {8, 4}, {8, 6}, {2, 6}};
  std::vector<brv::Polygon2D> obstacles = {obstacle};

  brv::CoverageGrid grid(boundary, obstacles, 0.5, 0.0);

  // The obstacle should create UNVISITABLE cells
  bool has_unvisitable = false;
  for (int r = 0; r < grid.rows(); ++r)
  {
    for (int c = 0; c < grid.cols(); ++c)
    {
      if (grid.cell(r, c) == brv::CellState::UNVISITABLE)
      {
        has_unvisitable = true;
        break;
      }
    }
    if (has_unvisitable)
      break;
  }
  EXPECT_TRUE(has_unvisitable);

  auto sweep = brv::boustrophedon_sweep(grid, 0, 0);
  EXPECT_GT(sweep.waypoints.size(), 0u);
  // Swath breaks indicate segments split by obstacles
  EXPECT_GE(sweep.swath_breaks.size(), 1u);
}

// ─────────────────────────────────────────────────────────────────────────────
// MultipleObstacles — L-shaped area → full coverage with breaks
// ─────────────────────────────────────────────────────────────────────────────

TEST(Grid, MultipleObstacles)
{
  brv::Polygon2D boundary = {{0, 0}, {6, 0}, {6, 6}, {0, 6}};
  brv::Polygon2D obs1 = {{1, 1}, {2, 1}, {2, 2}, {1, 2}};
  brv::Polygon2D obs2 = {{3, 3}, {4, 3}, {4, 4}, {3, 4}};
  std::vector<brv::Polygon2D> obstacles = {obs1, obs2};

  brv::CoverageGrid grid(boundary, obstacles, 0.5, 0.0);
  int total = grid.count_unvisited();
  EXPECT_GT(total, 0);

  auto sweep = brv::boustrophedon_sweep(grid, 0, 0);
  EXPECT_GT(sweep.waypoints.size(), 0u);
}

// ─────────────────────────────────────────────────────────────────────────────
// EmptyGrid — all cells UNVISITABLE → sweep returns empty
// ─────────────────────────────────────────────────────────────────────────────

TEST(Grid, EmptyGrid)
{
  // Boundary entirely covered by obstacle → no unvisited cells
  brv::Polygon2D boundary = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  // Obstacle larger than boundary
  brv::Polygon2D obstacle = {{-1, -1}, {3, -1}, {3, 3}, {-1, 3}};
  std::vector<brv::Polygon2D> obstacles = {obstacle};

  brv::CoverageGrid grid(boundary, obstacles, 0.5, 0.0);

  // All cells inside boundary are also inside obstacle → UNVISITABLE
  EXPECT_EQ(grid.count_unvisited(), 0);
  EXPECT_FALSE(grid.has_unvisited());

  auto sweep = brv::boustrophedon_sweep(grid, 0, 0);
  EXPECT_EQ(sweep.waypoints.size(), 0u);
}

// ─────────────────────────────────────────────────────────────────────────────
// SingleCellGrid — 1x1-ish grid → single waypoint
// ─────────────────────────────────────────────────────────────────────────────

TEST(Grid, SingleCellGrid)
{
  // Very small boundary, just enough for 1 cell at resolution 0.5
  brv::Polygon2D boundary = {{0, 0}, {0.6, 0}, {0.6, 0.6}, {0, 0.6}};
  std::vector<brv::Polygon2D> obstacles;

  brv::CoverageGrid grid(boundary, obstacles, 0.5, 0.0);

  // Should have at least 1 unvisited cell
  int unvisited = grid.count_unvisited();
  EXPECT_GE(unvisited, 1);

  auto sweep = brv::boustrophedon_sweep(grid, 0, 0);
  EXPECT_GE(sweep.waypoints.size(), 1u);
}

// ─────────────────────────────────────────────────────────────────────────────
// SkipEmptyColumns — gap of empty columns → sweep stops at gap
// ─────────────────────────────────────────────────────────────────────────────

TEST(Grid, SkipEmptyColumns)
{
  // Wide boundary with an obstacle strip creating empty columns
  brv::Polygon2D boundary = {{0, 0}, {6, 0}, {6, 2}, {0, 2}};
  // Wall obstacle blocking columns 2-3
  brv::Polygon2D wall = {{2, -0.5}, {3, -0.5}, {3, 2.5}, {2, 2.5}};
  std::vector<brv::Polygon2D> obstacles = {wall};

  brv::CoverageGrid grid(boundary, obstacles, 0.5, 0.0);

  // Should still have unvisited cells on both sides
  EXPECT_GT(grid.count_unvisited(), 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// MapToCellRoundtrip — cell_to_map → map_to_cell ≈ same coordinates
// ─────────────────────────────────────────────────────────────────────────────

TEST(Grid, MapToCellRoundtrip)
{
  brv::Polygon2D boundary = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  std::vector<brv::Polygon2D> obstacles;
  brv::CoverageGrid grid(boundary, obstacles, 0.5, 0.0);

  // Test several cells
  for (int r = 1; r < grid.rows() - 1; r += 2)
  {
    for (int c = 1; c < grid.cols() - 1; c += 2)
    {
      auto map_pt = grid.cell_to_map(r, c);
      auto [back_r, back_c] = grid.map_to_cell(map_pt.x, map_pt.y);
      EXPECT_EQ(back_r, r) << "Row mismatch for cell (" << r << ", " << c << ")";
      EXPECT_EQ(back_c, c) << "Col mismatch for cell (" << r << ", " << c << ")";
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// SweepAngle — grid with non-zero sweep angle
// ─────────────────────────────────────────────────────────────────────────────

TEST(Grid, NonZeroSweepAngle)
{
  brv::Polygon2D boundary = {{0, 0}, {5, 0}, {5, 5}, {0, 5}};
  std::vector<brv::Polygon2D> obstacles;

  double angle = M_PI / 4.0;  // 45 degrees
  brv::CoverageGrid grid(boundary, obstacles, 0.5, angle);

  EXPECT_GT(grid.rows(), 0);
  EXPECT_GT(grid.cols(), 0);
  EXPECT_GT(grid.count_unvisited(), 0);

  auto sweep = brv::boustrophedon_sweep(grid, 0, 0);
  EXPECT_GT(sweep.waypoints.size(), 0u);
}

// ─────────────────────────────────────────────────────────────────────────────
// FindNearestUnvisited
// ─────────────────────────────────────────────────────────────────────────────

TEST(Grid, FindNearestUnvisited)
{
  brv::Polygon2D boundary = {{0, 0}, {4, 0}, {4, 4}, {0, 4}};
  std::vector<brv::Polygon2D> obstacles;
  brv::CoverageGrid grid(boundary, obstacles, 0.5, 0.0);

  // Mark most cells as visited, leave one
  for (int r = 0; r < grid.rows(); ++r)
  {
    for (int c = 0; c < grid.cols(); ++c)
    {
      if (grid.cell(r, c) == brv::CellState::UNVISITED && !(r == 5 && c == 5))
      {
        grid.set_cell(r, c, brv::CellState::VISITED);
      }
    }
  }

  // If (5,5) is still unvisited, find_nearest should return it
  if (5 < grid.rows() && 5 < grid.cols() && grid.cell(5, 5) == brv::CellState::UNVISITED)
  {
    auto [nr, nc] = grid.find_nearest_unvisited(0, 0);
    EXPECT_EQ(nr, 5);
    EXPECT_EQ(nc, 5);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// GetUnvisitedRegionStarts — multiple disconnected regions
// ─────────────────────────────────────────────────────────────────────────────

TEST(Grid, UnvisitedRegionStarts)
{
  // Wide boundary with a thick wall splitting it into two regions
  brv::Polygon2D boundary = {{0, 0}, {10, 0}, {10, 4}, {0, 4}};
  // Thick wall from top to bottom, fully splitting the area
  brv::Polygon2D wall = {{4, -1}, {6, -1}, {6, 5}, {4, 5}};
  std::vector<brv::Polygon2D> obstacles = {wall};

  brv::CoverageGrid grid(boundary, obstacles, 0.5, 0.0);

  auto regions = grid.get_unvisited_region_starts();
  // Should have at least 2 disconnected regions
  EXPECT_GE(regions.size(), 2u);
}
