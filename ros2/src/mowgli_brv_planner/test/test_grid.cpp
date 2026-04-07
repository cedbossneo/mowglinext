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
