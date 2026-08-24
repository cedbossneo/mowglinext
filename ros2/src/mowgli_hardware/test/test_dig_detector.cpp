// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the wheel-slip "digging" detector and its bounded
// reverse-escape budget. Pure logic, no ROS — mirrors test_ftc_stall.cpp.

#include <limits>

#include "mowgli_hardware/dig_detector.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

namespace
{
// Drive `seconds` of perfectly-tracking motion (map keeps up with wheels).
mh::DigAction RunGoodTraction(const mh::DigDetectorCfg& cfg,
                              mh::DigDetectorState& st,
                              double seconds,
                              double dt = 0.1,
                              double yaw_rate = 0.0)
{
  mh::DigAction last = mh::DigAction::kNone;
  for (double t = 0.0; t < seconds; t += dt)
  {
    // 0.3 m/s commanded, 0.03 m per 0.1 s tick on BOTH wheel and map.
    last = mh::DigDecide(cfg, st, 0.3, 0.03, 0.03, 0.003, yaw_rate, dt).action;
  }
  return last;
}

// Drive `seconds` of digging: wheels turn, the fused map pose barely moves.
mh::DigAction RunDigging(const mh::DigDetectorCfg& cfg,
                         mh::DigDetectorState& st,
                         double seconds,
                         double pos_sigma = 0.003,
                         double dt = 0.1,
                         double yaw_rate = 0.0)
{
  mh::DigAction last = mh::DigAction::kNone;
  for (double t = 0.0; t < seconds; t += dt)
  {
    last = mh::DigDecide(cfg, st, 0.3, 0.03, 0.001, pos_sigma, yaw_rate, dt).action;
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
    EXPECT_EQ(mh::DigDecide(cfg, st, 0.0, 0.0, 0.0, 0.003, 0.0, 0.1).action, mh::DigAction::kNone);
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
    last = mh::DigDecide(cfg, st, 0.06, 0.005, 0.005, 0.003, 0.0, 0.1).action;
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
    last = mh::DigDecide(cfg, st, 0.3, 0.0, 0.0, 0.003, 0.0, 0.1).action;
  }
  EXPECT_EQ(last, mh::DigAction::kNone);
}

TEST(DigDetector, TractionReturningResetsTheWindow)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  RunDigging(cfg, st, 0.5);  // partial evidence
  RunGoodTraction(cfg, st, 2.0);  // traction returns -> must clear
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
    v = mh::DigDecide(cfg, st, 0.3, 0.03, 0.001, 0.003, 0.0, 0.1);
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

// ─────────────────────────────────────────────────────────────────────────────
// Turn exclusion
// ─────────────────────────────────────────────────────────────────────────────

TEST(DigDetector, TurningSuppressesDetection)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // A swath U-turn at connector_turn_radius 0.18 m and 0.2 m/s is ~1.1 rad/s.
  // The wheel-vs-map ratio collapses there on a perfectly healthy robot
  // (arc vs chord, plus the graph's own estimate degrading), so it must not
  // be read as a dig.
  EXPECT_EQ(RunDigging(cfg, st, 10.0, 0.003, 0.1, 1.1), mh::DigAction::kNone);
}

TEST(DigDetector, StaleGyroSuppressesDetection)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // The bridge passes infinity when the gyro is stale: the turn exclusion
  // cannot be evaluated, so stand down rather than guess.
  EXPECT_EQ(RunDigging(cfg, st, 10.0, 0.003, 0.1, std::numeric_limits<double>::infinity()),
            mh::DigAction::kNone);

  mh::DigDetectorState st_nan;
  EXPECT_EQ(RunDigging(cfg, st_nan, 10.0, 0.003, 0.1, std::numeric_limits<double>::quiet_NaN()),
            mh::DigAction::kNone);
}

TEST(DigDetector, GentleCurveStillDetects)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // Coverage straights carry a little yaw correction. Below max_yaw_rate the
  // detector must stay armed, or a dig on a slightly curving swath is missed.
  EXPECT_EQ(RunDigging(cfg, st, 10.0, 0.003, 0.1, 0.15), mh::DigAction::kDig);
}

// ─────────────────────────────────────────────────────────────────────────────
// Regression: the dig recorded on the robot, 2026-08-24 09:48:23 UTC
// ─────────────────────────────────────────────────────────────────────────────

// Left encoder ran 889 ticks (3.17 m) while the right ran 50 (0.18 m) over
// 9 s; the chassis neither translated nor rotated (FTC's error vector was
// frozen at lat=0.551 lon=0.835 ang=-41.2 deg for the whole window, and the
// gyro read ~0). Mean wheel travel is therefore ~1.68 m of phantom advance.
// Nothing fired at the time. This pins that it now does.
TEST(DigDetector, FieldRecordedOneWheelSlipIsDetected)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // 1.68 m of claimed travel over 9 s => 0.0187 m per 0.1 s tick, chassis
  // static, gyro ~0 because only ONE wheel was spinning.
  constexpr double kWheelStep = 0.0187;
  constexpr double kMapStep = 0.0;
  constexpr double kGyro = 0.01;  // chassis not rotating
  constexpr double kSigma = 0.09;  // measured baseline major axis, RTK-Fixed

  mh::DigAction last = mh::DigAction::kNone;
  for (double t = 0.0; t < 9.0; t += 0.1)
  {
    last = mh::DigDecide(cfg, st, 0.2, kWheelStep, kMapStep, kSigma, kGyro, 0.1).action;
    if (last == mh::DigAction::kDig)
    {
      break;
    }
  }
  EXPECT_EQ(last, mh::DigAction::kDig) << "the 2026-08-24 one-wheel dig must trip the detector";
}

// The same event, gated by the OLD 0.10 m sigma threshold, did not fire —
// the measured baseline major axis (0.05-0.14 m) straddles it. Pins why the
// threshold moved.
TEST(DigDetector, OldSigmaThresholdWouldHaveMissedTheFieldDig)
{
  mh::DigDetectorCfg cfg;
  cfg.max_pos_sigma = 0.10;  // the pre-fix value
  mh::DigDetectorState st;

  EXPECT_EQ(RunDigging(cfg, st, 9.0, 0.14, 0.1, 0.01), mh::DigAction::kNone)
      << "documents the miss the new threshold fixes";
}

// ─────────────────────────────────────────────────────────────────────────────
// Trust-signal selection (DigTrustSigma)
// ─────────────────────────────────────────────────────────────────────────────

TEST(DigTrustSigma, RtkFixedWithValidAccuracyIsTrusted)
{
  EXPECT_NEAR(mh::DigTrustSigma(true, true, true, 0.014), 0.014, 1e-12);
}

TEST(DigTrustSigma, AnythingMissingSuppresses)
{
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(mh::DigTrustSigma(false, true, true, 0.014), inf) << "stale /gps/status";
  EXPECT_EQ(mh::DigTrustSigma(true, false, true, 0.014), inf) << "not RTK-Fixed";
  EXPECT_EQ(mh::DigTrustSigma(true, true, false, 0.014), inf) << "no accuracy in message";
  EXPECT_EQ(mh::DigTrustSigma(true, true, true, -1.0), inf) << "negative accuracy";
}

// RTK-Float reports decimetres to metres; the detector must stand down there.
TEST(DigTrustSigma, FloatAccuracyExceedsTheGate)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;
  const double float_acc = mh::DigTrustSigma(true, true, true, 0.45);
  EXPECT_GT(float_acc, cfg.max_pos_sigma);
  EXPECT_EQ(RunDigging(cfg, st, 9.0, float_acc, 0.1, 0.0), mh::DigAction::kNone);
}

// The measurement that moved the gate off the graph marginal. Sampled over 90 s
// of live RTK-Fixed mowing on 2026-08-24, fusion_graph's published position
// sigma had p50 0.574 m while the receiver reported 0.014 m and FTC tracked to
// 8-16 mm. Feeding the graph value blocks; feeding the receiver value detects.
TEST(DigTrustSigma, GraphMarginalWouldBlockWhereReceiverAccuracyDoesNot)
{
  mh::DigDetectorCfg cfg;

  mh::DigDetectorState st_graph;
  EXPECT_EQ(RunDigging(cfg, st_graph, 9.0, 0.574, 0.1, 0.01), mh::DigAction::kNone)
      << "graph marginal p50 — this is why the gate moved off it";

  mh::DigDetectorState st_gnss;
  EXPECT_EQ(RunDigging(cfg, st_gnss, 9.0, 0.014, 0.1, 0.01), mh::DigAction::kDig)
      << "receiver accuracy at the same instant";
}
