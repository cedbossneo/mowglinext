#include <cmath>
#include <limits>

#include "mowgli_hardware/cmd_vel_validation.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::is_finite_velocity_command;

TEST(CmdVelValidation, AcceptsFiniteCommands)
{
  EXPECT_TRUE(is_finite_velocity_command(0.25, -0.8));
  EXPECT_TRUE(is_finite_velocity_command(0.0, 0.0));
  EXPECT_TRUE(is_finite_velocity_command(-0.0, 0.0));
  EXPECT_TRUE(is_finite_velocity_command(0.0, -0.0));
}

TEST(CmdVelValidation, RejectsNonFiniteLinearVelocity)
{
  EXPECT_FALSE(is_finite_velocity_command(std::numeric_limits<double>::quiet_NaN(), 0.0));
  EXPECT_FALSE(is_finite_velocity_command(std::numeric_limits<double>::infinity(), 0.0));
  EXPECT_FALSE(is_finite_velocity_command(-std::numeric_limits<double>::infinity(), 0.0));
}

TEST(CmdVelValidation, RejectsNonFiniteAngularVelocity)
{
  EXPECT_FALSE(is_finite_velocity_command(0.0, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(is_finite_velocity_command(0.0, std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(is_finite_velocity_command(0.0, -std::numeric_limits<double>::infinity()));
}

TEST(CmdVelValidation, RejectsBothNonFinite)
{
  EXPECT_FALSE(is_finite_velocity_command(std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::infinity()));
}

TEST(CmdVelValidation, RejectsValuesNonFiniteAfterFloatWireConversion)
{
  const double positive_overflow = std::numeric_limits<double>::max();
  const double negative_overflow = -std::numeric_limits<double>::max();
  EXPECT_FALSE(is_finite_velocity_command(static_cast<float>(positive_overflow), 0.0f));
  EXPECT_FALSE(is_finite_velocity_command(0.0f, static_cast<float>(negative_overflow)));
}

TEST(CmdVelValidation, AcceptsMaximumFiniteFloatWireValue)
{
  const float largest_finite_float = std::numeric_limits<float>::max();
  EXPECT_TRUE(is_finite_velocity_command(largest_finite_float, -largest_finite_float));
}
