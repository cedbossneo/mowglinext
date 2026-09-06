// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/lidar_map_anchor_gate.hpp"
#include <gtest/gtest.h>

using fusion_graph::LidarAnchorState;
using fusion_graph::LidarMapAnchorGate;

TEST(LidarMapAnchorGate, DisabledDoesNothing)
{
  LidarMapAnchorGate g(false, 0.3, 0.5);
  const auto d = g.Step(100.0, true, 1.0);
  EXPECT_EQ(d.state, LidarAnchorState::kDisabled);
  EXPECT_FALSE(d.insert_scan);
  EXPECT_FALSE(d.run_filter);
  EXPECT_FALSE(d.seed_filter);
}

// Fresh RTK-Fixed: the pose is trusted, so scans build the map and no anchor
// factor is produced — the GPS already pins the pose.
TEST(LidarMapAnchorGate, FreshFixedMapsAndNeverAnchors)
{
  LidarMapAnchorGate g(true, 0.3, 0.5);
  const auto d = g.Step(0.1, true, 1.0);
  EXPECT_EQ(d.state, LidarAnchorState::kMapping);
  EXPECT_TRUE(d.insert_scan);
  EXPECT_FALSE(d.run_filter);
  EXPECT_FALSE(d.seed_filter);
}

// Insertion is rate-limited: a 10 Hz LiDAR need not write every scan.
TEST(LidarMapAnchorGate, InsertionIsRateLimited)
{
  LidarMapAnchorGate g(true, 0.3, 0.5);
  EXPECT_TRUE(g.Step(0.0, true, 1.0).insert_scan);
  EXPECT_FALSE(g.Step(0.0, true, 1.2).insert_scan);
  EXPECT_FALSE(g.Step(0.0, true, 1.4).insert_scan);
  EXPECT_TRUE(g.Step(0.0, true, 1.5).insert_scan);
}

// Stale Fixed with nothing in the map yet: stay honest, no factor.
TEST(LidarMapAnchorGate, StaleWithoutMapWaits)
{
  LidarMapAnchorGate g(true, 0.3, 0.5);
  const auto d = g.Step(5.0, false, 1.0);
  EXPECT_EQ(d.state, LidarAnchorState::kWaitingForMap);
  EXPECT_FALSE(d.run_filter);
  EXPECT_FALSE(d.seed_filter);
}

// The MAPPING→ANCHORING edge seeds the filter exactly once; later stale
// scans run the filter without re-seeding (that would throw away convergence).
TEST(LidarMapAnchorGate, EngageSeedsOnceThenTracks)
{
  LidarMapAnchorGate g(true, 0.3, 0.5);
  g.Step(0.0, true, 1.0);
  const auto first = g.Step(1.0, true, 2.0);
  EXPECT_EQ(first.state, LidarAnchorState::kAnchoring);
  EXPECT_TRUE(first.seed_filter);
  EXPECT_TRUE(first.run_filter);
  const auto second = g.Step(2.0, true, 3.0);
  EXPECT_FALSE(second.seed_filter);
  EXPECT_TRUE(second.run_filter);
}

// RTK coming back to Fixed returns to mapping; a later loss re-seeds, because
// the filter was not tracking meanwhile and the fused pose is the fresher truth.
TEST(LidarMapAnchorGate, ReturnToFixedDisengagesAndNextLossReseeds)
{
  LidarMapAnchorGate g(true, 0.3, 0.5);
  g.Step(1.0, true, 1.0);
  EXPECT_EQ(g.state(), LidarAnchorState::kAnchoring);
  const auto back = g.Step(0.0, true, 2.0);
  EXPECT_EQ(back.state, LidarAnchorState::kMapping);
  EXPECT_FALSE(back.run_filter);
  const auto again = g.Step(1.0, true, 3.0);
  EXPECT_TRUE(again.seed_filter);
}
