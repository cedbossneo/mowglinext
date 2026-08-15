// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace mowgli_localization
{

inline sensor_msgs::msg::LaserScan confirm_scan_returns(const sensor_msgs::msg::LaserScan& current,
                                                        const sensor_msgs::msg::LaserScan* previous,
                                                        double angle_tolerance_rad,
                                                        double range_tolerance_m)
{
  sensor_msgs::msg::LaserScan confirmed = current;
  const float inf = std::numeric_limits<float>::infinity();
  if (previous == nullptr || previous->ranges.size() != current.ranges.size() ||
      current.angle_increment == 0.0f ||
      std::abs(previous->angle_increment - current.angle_increment) > 1e-9)
  {
    for (auto& range : confirmed.ranges)
    {
      if (std::isfinite(range))
        range = inf;
    }
    return confirmed;
  }

  const int radius = std::max(
      0, static_cast<int>(std::ceil(angle_tolerance_rad / std::abs(current.angle_increment))));
  const int last = static_cast<int>(current.ranges.size()) - 1;
  for (int i = 0; i <= last; ++i)
  {
    const float range = current.ranges[static_cast<std::size_t>(i)];
    if (!std::isfinite(range))
      continue;

    bool matched = false;
    for (int j = std::max(0, i - radius); j <= std::min(last, i + radius); ++j)
    {
      const float previous_range = previous->ranges[static_cast<std::size_t>(j)];
      if (std::isfinite(previous_range) &&
          std::abs(static_cast<double>(previous_range - range)) <= range_tolerance_m)
      {
        matched = true;
        break;
      }
    }
    if (!matched)
      confirmed.ranges[static_cast<std::size_t>(i)] = inf;
  }
  return confirmed;
}

}  // namespace mowgli_localization
