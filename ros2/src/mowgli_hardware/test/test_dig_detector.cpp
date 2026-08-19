// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the wheel-slip "digging" detector and its bounded
// reverse-escape budget. Pure logic, no ROS — mirrors test_ftc_stall.cpp.

#include <gtest/gtest.h>

#include "mowgli_hardware/dig_detector.hpp"

namespace mh = mowgli_hardware;

namespace
{
// Drive `seconds` of perfectly-tracking motion (map keeps up with wheels).
mh::DigAction RunGoodTraction(const mh::DigDetectorCfg& cfg,
                              mh::DigDetectorState& st,
                              double seconds,
                              double dt = 0.1)
{
  mh::DigAction last = mh::DigAction::kNone;
  for (double t = 0.0; t < seconds; t += dt)
  {
    // 0.3 m/s commanded, 0.03 m per 0.1 s tick on BOTH wheel and map.
    last = mh::DigDecide(cfg, st, 0.3, 0.03, 0.03, 0.003, dt).action;
  }
  return last;
}

// Drive `seconds` of digging: wheels turn, the fused map pose barely moves.
mh::DigAction RunDigging(const mh::DigDetectorCfg& cfg,
                         mh::DigDetectorState& st,
                         double seconds,
                         double pos_sigma = 0.003,
                         double dt = 0.1)
{
  mh::DigAction last = mh::DigAction::kNone;
  for (double t = 0.0; t < seconds; t += dt)
  {
    last = mh::DigDecide(cfg, st, 0.3, 0.03, 0.001, pos_sigma, dt).action;
    if (last == mh::DigAction::kDig)
    {
      break;
    }
  }
  return last;
}
}  // namespace

// ── Detection ───────────────────────────────────────────────────────────────

TEST(DigDetector, GoodTractionNeverTrips)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  EXPECT_EQ(RunGoodTraction(cfg, st, 10.0), mh::DigAction::kNone);
}

TEST(DigDetector, SpinningWheelsWithNoMapProgressTrips)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  EXPECT_EQ(RunDigging(cfg, st, 10.0), mh::DigAction::kDig);
}

TEST(DigDetector, DoesNotTripBeforeTheWindowElapses)
{
  mh::DigDetectorCfg cfg;  // window_s = 1.2
  mh::DigDetectorState st;

  // 0.5 s of digging is not yet enough evidence.
  EXPECT_EQ(RunDigging(cfg, st, 0.5), mh::DigAction::kNone);
}

TEST(DigDetector, UntrustworthyFusedPoseSuppressesDetection)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // Same digging signature, but RTK-Float: sigma way above max_pos_sigma.
  // Must NOT trip — the map pose can't distinguish slip from GPS noise.
  EXPECT_EQ(RunDigging(cfg, st, 10.0, 0.9), mh::DigAction::kNone);
}

TEST(DigDetector, StationaryRobotNeverTrips)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // Not commanded to move: no wheel travel, no map travel.
  for (int i = 0; i < 100; ++i)
  {
    EXPECT_EQ(mh::DigDecide(cfg, st, 0.0, 0.0, 0.0, 0.003, 0.1).action, mh::DigAction::kNone);
  }
}

TEST(DigDetector, SlowLegitimateCreepIsNotADig)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // Docking creep: commanded slowly, wheels AND map agree at 0.005 m/tick.
  mh::DigAction last = mh::DigAction::kNone;
  for (int i = 0; i < 100; ++i)
  {
    last = mh::DigDecide(cfg, st, 0.06, 0.005, 0.005, 0.003, 0.1).action;
  }
  EXPECT_EQ(last, mh::DigAction::kNone);
}

TEST(DigDetector, BlockedChassisWithNoWheelTravelIsNotOurCase)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // Hard stall: wheels don't turn either. Firmware anti-dig owns this;
  // we require min_wheel_dist of claimed travel before calling it a dig.
  mh::DigAction last = mh::DigAction::kNone;
  for (int i = 0; i < 100; ++i)
  {
    last = mh::DigDecide(cfg, st, 0.3, 0.0, 0.0, 0.003, 0.1).action;
  }
  EXPECT_EQ(last, mh::DigAction::kNone);
}

TEST(DigDetector, TractionReturningResetsTheWindow)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  RunDigging(cfg, st, 0.5);          // partial evidence
  RunGoodTraction(cfg, st, 2.0);     // traction returns -> must clear
  EXPECT_EQ(RunDigging(cfg, st, 0.5), mh::DigAction::kNone);
}

TEST(DigDetector, DisabledNeverTrips)
{
  mh::DigDetectorCfg cfg;
  cfg.enabled = false;
  mh::DigDetectorState st;

  EXPECT_EQ(RunDigging(cfg, st, 10.0), mh::DigAction::kNone);
}

// ── Bounded reverse escape ──────────────────────────────────────────────────

TEST(DigEscape, ReversesWhileBudgetRemains)
{
  mh::DigEscapeCfg cfg;
  mh::DigEscapeState st;

  const double v = mh::DigEscapeStep(cfg, st, 0.1);
  EXPECT_LT(v, 0.0) << "escape must command reverse";
  EXPECT_NEAR(v, -cfg.reverse_speed, 1e-9);
}

TEST(DigEscape, StopsAfterDistanceBudgetSpent)
{
  mh::DigEscapeCfg cfg;  // reverse_dist default 0.30 m
  mh::DigEscapeState st;

  double v = 0.0;
  for (int i = 0; i < 200; ++i)
  {
    v = mh::DigEscapeStep(cfg, st, 0.1);
  }
  EXPECT_EQ(v, 0.0);
  EXPECT_TRUE(mh::DigEscapeDone(cfg, st));
  EXPECT_LE(st.travelled, cfg.reverse_dist + 1e-9) << "must not overshoot the budget";
}

TEST(DigEscape, StopsAfterTimeoutEvenWithoutDistance)
{
  mh::DigEscapeCfg cfg;
  mh::DigEscapeState st;

  // dt large enough to blow the timeout before the distance budget.
  mh::DigEscapeStep(cfg, st, cfg.timeout_s + 0.1);
  EXPECT_TRUE(mh::DigEscapeDone(cfg, st));
  EXPECT_EQ(mh::DigEscapeStep(cfg, st, 0.1), 0.0);
}

TEST(DigEscape, IgnoresNonPositiveDt)
{
  mh::DigEscapeCfg cfg;
  mh::DigEscapeState st;

  mh::DigEscapeStep(cfg, st, -0.5);
  EXPECT_EQ(st.travelled, 0.0);
  EXPECT_EQ(st.elapsed, 0.0);
}

TEST(DigEscape, ResetRestoresFullBudget)
{
  mh::DigEscapeCfg cfg;
  mh::DigEscapeState st;

  for (int i = 0; i < 200; ++i)
  {
    mh::DigEscapeStep(cfg, st, 0.1);
  }
  ASSERT_TRUE(mh::DigEscapeDone(cfg, st));

  st = mh::DigEscapeState{};
  EXPECT_FALSE(mh::DigEscapeDone(cfg, st));
  EXPECT_LT(mh::DigEscapeStep(cfg, st, 0.1), 0.0);
}

// Regression: the verdict must CARRY the evidence, because DigDecide resets
// its window on every verdict. Reading the state after a kDig reports zeros,
// which is what the dig event and the operator-facing log would have shown.
TEST(DigDetector, VerdictCarriesEvidenceAfterWindowReset)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  mh::DigVerdict v;
  for (int i = 0; i < 100; ++i)
  {
    v = mh::DigDecide(cfg, st, 0.3, 0.03, 0.001, 0.003, 0.1);
    if (v.action == mh::DigAction::kDig)
    {
      break;
    }
  }

  ASSERT_EQ(v.action, mh::DigAction::kDig);
  EXPECT_GT(v.wheel_dist, cfg.min_wheel_dist) << "evidence must survive the reset";
  EXPECT_LT(v.map_dist, v.wheel_dist * cfg.progress_fraction);
  EXPECT_EQ(st.wheel_dist, 0.0) << "window itself is reset for the next pass";
}
