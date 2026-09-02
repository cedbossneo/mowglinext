#pragma once

#include <cmath>

namespace mowgli_hardware
{

/// A velocity command is usable only when both values are IEEE-754 finite.
/// In particular, comparisons against limits do not reject NaN.
inline bool is_finite_velocity_command(double linear_x, double angular_z)
{
  return std::isfinite(linear_x) && std::isfinite(angular_z);
}

}  // namespace mowgli_hardware
