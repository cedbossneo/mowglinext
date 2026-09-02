// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "heartbeat_emergency_policy.hpp"
#include <gtest/gtest.h>

TEST(HeartbeatEmergencyPolicy, NoFlagsLeaveEmergencyUnchanged)
{
  EXPECT_EQ(decide_heartbeat_emergency(false, false, false, false).action,
            HeartbeatEmergencyAction::Unchanged);
}

TEST(HeartbeatEmergencyPolicy, StopAssertsEmergency)
{
  EXPECT_EQ(decide_heartbeat_emergency(true, false, false, false).action,
            HeartbeatEmergencyAction::Assert);
}

TEST(HeartbeatEmergencyPolicy, ReleaseOnlyWithoutPhysicalEmergencyReleases)
{
  EXPECT_EQ(decide_heartbeat_emergency(false, true, false, false).action,
            HeartbeatEmergencyAction::Release);
}

TEST(HeartbeatEmergencyPolicy, ReleaseOnlyWithPhysicalEmergencyIsRejected)
{
  EXPECT_EQ(decide_heartbeat_emergency(false, true, true, false).action,
            HeartbeatEmergencyAction::Unchanged);
}

TEST(HeartbeatEmergencyPolicy, StopDominatesReleaseWithoutPhysicalEmergency)
{
  EXPECT_EQ(decide_heartbeat_emergency(true, true, false, false).action,
            HeartbeatEmergencyAction::Assert);
}

TEST(HeartbeatEmergencyPolicy, StopDominatesReleaseWithPhysicalEmergency)
{
  EXPECT_EQ(decide_heartbeat_emergency(true, true, true, false).action,
            HeartbeatEmergencyAction::Assert);
}

TEST(HeartbeatEmergencyPolicy, ReleaseWorksAfterPreviousStop)
{
  EXPECT_EQ(decide_heartbeat_emergency(true, false, false, false).action,
            HeartbeatEmergencyAction::Assert);
  EXPECT_EQ(decide_heartbeat_emergency(false, true, false, false).action,
            HeartbeatEmergencyAction::Release);
}

TEST(HeartbeatEmergencyPolicy, WatchdogAutoClearCannotBypassExplicitStop)
{
  const auto decision = decide_heartbeat_emergency(true, true, false, true);
  EXPECT_EQ(decision.action, HeartbeatEmergencyAction::Assert);
  EXPECT_TRUE(decision.clear_heartbeat_only_latch);
}
