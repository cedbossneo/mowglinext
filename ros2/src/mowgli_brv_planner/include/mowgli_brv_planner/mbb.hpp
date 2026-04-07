#pragma once

#include "mowgli_brv_planner/types.hpp"

namespace brv
{

// Compute the optimal sweep angle using Minimum Bounding Box (rotating calipers).
// Returns the angle (radians) of the longest MBB edge.
// Reference: Algorithm 1 in Huang et al. (2021).
double compute_mbb_angle(const Polygon2D& boundary);

}  // namespace brv
