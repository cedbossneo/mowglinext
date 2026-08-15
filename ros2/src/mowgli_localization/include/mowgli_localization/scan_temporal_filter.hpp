// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace mowgli_localization
{

struct ScanConfirmationState
{
  sensor_msgs::msg::LaserScan previous;
  std::vector<unsigned int> counts;
  bool valid{false};
};

inline sensor_msgs::msg::LaserScan confirm_scan_returns(const sensor_msgs::msg::LaserScan& current,
                                                        unsigned int required_frames,
                                                        double angle_tolerance_rad,
                                                        double range_tolerance_m,
                                                        ScanConfirmationState& state)
{
  if (required_frames <= 1)
    return current;

  sensor_msgs::msg::LaserScan confirmed = current;
  const float inf = std::numeric_limits<float>::infinity();
  if (!state.valid || state.previous.ranges.size() != current.ranges.size() ||
      state.counts.size() != current.ranges.size() ||
      std::abs(state.previous.angle_increment - current.angle_increment) > 1e-9)
  {
    state.previous = current;
    state.counts.assign(current.ranges.size(), 0);
    for (std::size_t i = 0; i < current.ranges.size(); ++i)
    {
      if (std::isfinite(current.ranges[i]))
      {
        state.counts[i] = 1;
        confirmed.ranges[i] = inf;
      }
    }
    state.valid = true;
    return confirmed;
  }

  const int radius = current.angle_increment == 0.0f
                         ? 0
                         : std::max(0,
                                    static_cast<int>(std::ceil(angle_tolerance_rad /
                                                               std::abs(current.angle_increment))));
  const int last = static_cast<int>(current.ranges.size()) - 1;
  std::vector<unsigned int> next_counts(current.ranges.size(), 0);
  for (int i = 0; i <= last; ++i)
  {
    const float range = current.ranges[static_cast<std::size_t>(i)];
    if (!std::isfinite(range))
      continue;

    unsigned int prior_count = 0;
    for (int j = std::max(0, i - radius); j <= std::min(last, i + radius); ++j)
    {
      const float previous_range = state.previous.ranges[static_cast<std::size_t>(j)];
      if (std::isfinite(previous_range) &&
          std::abs(static_cast<double>(previous_range - range)) <= range_tolerance_m)
      {
        prior_count = std::max(prior_count, state.counts[static_cast<std::size_t>(j)]);
      }
    }
    next_counts[static_cast<std::size_t>(i)] = std::min(required_frames, prior_count + 1);
    if (next_counts[static_cast<std::size_t>(i)] < required_frames)
      confirmed.ranges[static_cast<std::size_t>(i)] = inf;
  }
  state.previous = current;
  state.counts = std::move(next_counts);
  return confirmed;
}

}  // namespace mowgli_localization
