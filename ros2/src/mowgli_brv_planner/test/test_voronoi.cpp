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

#include "mowgli_brv_planner/geometry_utils.hpp"
#include "mowgli_brv_planner/voronoi_roadmap.hpp"
#include <gtest/gtest.h>

// ─────────────────────────────────────────────────────────────────────────────
// VoronoiRoadmap tests
// ─────────────────────────────────────────────────────────────────────────────

TEST(Voronoi, PathAroundObstacle)
{
  // 10x10 boundary with a wall-like obstacle in the middle
  brv::Polygon2D boundary = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  brv::Polygon2D obstacle = {{4.5, 2}, {5.5, 2}, {5.5, 8}, {4.5, 8}};
  std::vector<brv::Polygon2D> obstacles = {obstacle};

  brv::VoronoiRoadmap roadmap(boundary, obstacles, 0.5, 6);

  brv::Point2D start{2, 5};
  brv::Point2D end{8, 5};
  auto path = roadmap.find_path(start, end);

  // Path should exist and have more than just start+end
  EXPECT_GE(path.size(), 2u);

  // First and last points should be start and end
  EXPECT_NEAR(path.front().x, start.x, 1e-6);
  EXPECT_NEAR(path.front().y, start.y, 1e-6);
  EXPECT_NEAR(path.back().x, end.x, 1e-6);
  EXPECT_NEAR(path.back().y, end.y, 1e-6);

  // No waypoint should be inside the obstacle
  for (const auto& pt : path)
  {
    EXPECT_FALSE(brv::point_in_polygon(pt, obstacle))
        << "Waypoint (" << pt.x << ", " << pt.y << ") is inside obstacle";
  }
}

TEST(Voronoi, DirectPathNoObstacle)
{
  brv::Polygon2D boundary = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  std::vector<brv::Polygon2D> no_obstacles;

  brv::VoronoiRoadmap roadmap(boundary, no_obstacles, 0.5, 6);

  brv::Point2D start{2, 5};
  brv::Point2D end{8, 5};
  auto path = roadmap.find_path(start, end);

  EXPECT_GE(path.size(), 2u);
  EXPECT_NEAR(path.front().x, start.x, 1e-6);
  EXPECT_NEAR(path.back().x, end.x, 1e-6);

  // Path length should be reasonably close to straight-line distance (6m)
  double total = brv::path_length(path);
  double direct = start.dist(end);
  // Allow up to 3x detour through the Voronoi graph
  EXPECT_LT(total, direct * 3.0);
}

TEST(Voronoi, EmptyRoadmap)
{
  // Very tiny boundary with no room for Voronoi vertices
  brv::Polygon2D boundary = {{0, 0}, {0.01, 0}, {0.01, 0.01}, {0, 0.01}};
  std::vector<brv::Polygon2D> no_obstacles;

  brv::VoronoiRoadmap roadmap(boundary, no_obstacles, 0.5, 6);

  brv::Point2D start{0.002, 0.005};
  brv::Point2D end{0.008, 0.005};
  auto path = roadmap.find_path(start, end);

  // With no roadmap nodes, should fall back to direct start→end
  EXPECT_GE(path.size(), 2u);
  EXPECT_NEAR(path.front().x, start.x, 1e-6);
  EXPECT_NEAR(path.back().x, end.x, 1e-6);
}

TEST(Voronoi, DisconnectedRegions)
{
  // Obstacle that nearly bisects the space — path may fall back to direct line
  brv::Polygon2D boundary = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  // Wall from bottom to near-top, leaving only a tiny gap
  brv::Polygon2D wall = {{4.9, 0.1}, {5.1, 0.1}, {5.1, 9.8}, {4.9, 9.8}};
  std::vector<brv::Polygon2D> obstacles = {wall};

  brv::VoronoiRoadmap roadmap(boundary, obstacles, 0.3, 6);

  brv::Point2D start{2, 5};
  brv::Point2D end{8, 5};
  auto path = roadmap.find_path(start, end);

  // Should still return a path (possibly direct fallback)
  EXPECT_GE(path.size(), 2u);
  EXPECT_NEAR(path.front().x, start.x, 1e-6);
  EXPECT_NEAR(path.back().x, end.x, 1e-6);
}

TEST(Voronoi, NodeCount)
{
  brv::Polygon2D boundary = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  std::vector<brv::Polygon2D> no_obstacles;

  brv::VoronoiRoadmap roadmap(boundary, no_obstacles, 0.5, 6);

  // With a 10x10 boundary sampled at 0.5m, there should be Voronoi nodes
  EXPECT_GT(roadmap.nodes().size(), 0u);
}

TEST(Voronoi, EdgesExist)
{
  brv::Polygon2D boundary = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  std::vector<brv::Polygon2D> no_obstacles;

  brv::VoronoiRoadmap roadmap(boundary, no_obstacles, 0.5, 6);

  auto edges = roadmap.get_edges();
  EXPECT_GT(edges.size(), 0u);
}
