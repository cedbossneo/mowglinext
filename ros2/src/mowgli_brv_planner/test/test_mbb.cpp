#include <cmath>

#include "mowgli_brv_planner/mbb.hpp"
#include <gtest/gtest.h>

TEST(MBB, SquareBoundary)
{
  brv::Polygon2D square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  double angle = brv::compute_mbb_angle(square);
  // For a square, any axis-aligned angle is optimal
  EXPECT_NEAR(std::fmod(std::abs(angle), M_PI / 2), 0.0, 0.1);
}

TEST(MBB, RectangleBoundary)
{
  // Long rectangle along X axis — sweep should be along X
  brv::Polygon2D rect = {{0, 0}, {20, 0}, {20, 5}, {0, 5}};
  double angle = brv::compute_mbb_angle(rect);
  EXPECT_NEAR(angle, 0.0, 0.15);
}
