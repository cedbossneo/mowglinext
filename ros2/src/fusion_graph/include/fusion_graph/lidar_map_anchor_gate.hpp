// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Engage / disengage logic for the LiDAR map anchor. Pure — no ROS / GTSAM /
// Beluga — so it is unit-testable.
//
// Two regimes, decided by how stale the last RTK-Fixed receipt is:
//   MAPPING   — RTK-Fixed is fresh: the fused pose is trusted, scans are
//               inserted into the occupancy grid, no anchor factor is produced
//               (the GPS already pins the pose; a LiDAR factor would only add
//               noise, which is why scan_yield_to_rtk exists).
//   ANCHORING — RTK-Fixed is stale: the particle filter localises against the
//               grid and its (xy, covariance) becomes a unary factor.
// The transition MAPPING→ANCHORING must (re)seed the filter from the last
// trusted fused pose, so the filter starts converged instead of relocalising
// globally. The transition back simply stops producing factors.

#pragma once

#include <cstdint>

namespace fusion_graph
{

enum class LidarAnchorState : uint8_t
{
  kDisabled,
  kWaitingForMap,  // enabled, but the grid has no occupied cells yet
  kMapping,
  kAnchoring,
};

struct LidarAnchorDecision
{
  LidarAnchorState state = LidarAnchorState::kDisabled;
  bool seed_filter = false;  // true exactly once on the MAPPING→ANCHORING edge
  bool insert_scan = false;  // add this scan to the grid
  bool run_filter = false;  // run the particle filter update on this scan
};

class LidarMapAnchorGate
{
public:
  LidarMapAnchorGate(bool enabled, double engage_age_s, double insert_period_s)
      : enabled_(enabled), engage_age_s_(engage_age_s), insert_period_s_(insert_period_s)
  {
  }

  // rtk_fixed_age_s: seconds since the last accepted RTK-Fixed receipt
  // (a large value when none was ever received). map_has_structure: the grid
  // exports at least one occupied cell. now_s: monotonic time of this scan.
  LidarAnchorDecision Step(double rtk_fixed_age_s, bool map_has_structure, double now_s)
  {
    LidarAnchorDecision d;
    if (!enabled_)
    {
      state_ = LidarAnchorState::kDisabled;
      d.state = state_;
      return d;
    }
    const bool fixed_fresh = rtk_fixed_age_s <= engage_age_s_;
    if (fixed_fresh)
    {
      state_ = LidarAnchorState::kMapping;
      d.insert_scan = (now_s - last_insert_s_) >= insert_period_s_;
      if (d.insert_scan)
        last_insert_s_ = now_s;
    }
    else if (!map_has_structure)
    {
      // Nothing to localise against yet — stay honest, produce no factor.
      state_ = LidarAnchorState::kWaitingForMap;
    }
    else
    {
      d.seed_filter = (state_ != LidarAnchorState::kAnchoring);
      state_ = LidarAnchorState::kAnchoring;
      d.run_filter = true;
    }
    d.state = state_;
    return d;
  }

  LidarAnchorState state() const
  {
    return state_;
  }

private:
  bool enabled_;
  double engage_age_s_;
  double insert_period_s_;
  LidarAnchorState state_ = LidarAnchorState::kDisabled;
  double last_insert_s_ = -1e9;
};

}  // namespace fusion_graph
