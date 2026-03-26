// Copyright (c) 2024, Cedric - Mowgli ROS2 port.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for mowgli_nav2_plugins::FailureDetector.

#include <gtest/gtest.h>

#include "mowgli_nav2_plugins/oscillation_detector.hpp"

namespace mowgli_nav2_plugins
{

// ── Fixture ───────────────────────────────────────────────────────────────────

class OscillationDetectorTest : public ::testing::Test
{
protected:
  FailureDetector detector_;

  /// Push n identical (v, omega) samples into the detector.
  void pushSamples(
    int n,
    double v, double omega,
    double v_eps = 0.1, double omega_eps = 0.1)
  {
    for (int i = 0; i < n; ++i) {
      detector_.update(v, omega, 1.0, 1.0, 1.0, v_eps, omega_eps);
    }
  }

  /// Alternate omega between +omega_val and -omega_val for n pairs.
  void pushAlternating(
    int n,
    double v, double omega_val,
    double v_eps = 0.1, double omega_eps = 0.1)
  {
    for (int i = 0; i < n; ++i) {
      const double sign = (i % 2 == 0) ? 1.0 : -1.0;
      detector_.update(v, sign * omega_val, 1.0, 1.0, 1.0, v_eps, omega_eps);
    }
  }
};

// ── Test: initial state ───────────────────────────────────────────────────────

TEST_F(OscillationDetectorTest, InitialStateIsNotOscillating)
{
  EXPECT_FALSE(detector_.isOscillating());
}

// ── Test: clear resets state ──────────────────────────────────────────────────

TEST_F(OscillationDetectorTest, ClearResetsOscillationFlag)
{
  detector_.setBufferLength(10);

  // Produce oscillation pattern: low v, alternating omega.
  pushAlternating(10, 0.0, 0.9);

  // Whether oscillating or not, clear must reset the flag.
  detector_.clear();
  EXPECT_FALSE(detector_.isOscillating());
}

// ── Test: zero-capacity buffer never oscillates ───────────────────────────────

TEST_F(OscillationDetectorTest, ZeroCapacityNeverOscillates)
{
  detector_.setBufferLength(0);
  pushAlternating(20, 0.0, 0.9);
  EXPECT_FALSE(detector_.isOscillating());
}

// ── Test: buffer shorter than two samples never reaches half-full threshold ───

TEST_F(OscillationDetectorTest, SingleSampleBufferNeverOscillates)
{
  detector_.setBufferLength(1);
  // Half of 1 is 0 (integer), so detect() should still return false because
  // there can never be > 1 zero crossing with a single sample.
  pushAlternating(10, 0.0, 0.9);
  EXPECT_FALSE(detector_.isOscillating());
}

// ── Test: steady forward motion does not trigger oscillation ──────────────────

TEST_F(OscillationDetectorTest, SteadyForwardMotionIsNotOscillating)
{
  detector_.setBufferLength(20);

  // Normalised v=0.8, omega=0.0 -> |v_mean|=0.8 > v_eps => not oscillating.
  pushSamples(20, 0.8, 0.0, 0.1, 0.1);

  EXPECT_FALSE(detector_.isOscillating());
}

// ── Test: zero linear + alternating angular triggers oscillation ──────────────

TEST_F(OscillationDetectorTest, AlternatingAngularZeroLinearIsOscillating)
{
  // Buffer large enough to fill quickly.
  detector_.setBufferLength(10);

  // v=0 (normalised), omega alternates between +0.9 and -0.9.
  // Mean v = 0, mean omega = 0, zero crossings > 1 => should oscillate.
  pushAlternating(10, 0.0, 0.9, 0.1, 0.1);

  EXPECT_TRUE(detector_.isOscillating());
}

// ── Test: large v_eps prevents oscillation detection ─────────────────────────

TEST_F(OscillationDetectorTest, LargeEpsThresholdDoesNotOscillate)
{
  detector_.setBufferLength(10);

  // Same alternating pattern but v_eps and omega_eps are very large (> 1.0)
  // so the means will always be below the threshold.
  pushAlternating(10, 0.0, 0.9, 2.0 /*v_eps*/, 2.0 /*omega_eps*/);

  EXPECT_FALSE(detector_.isOscillating());
}

// ── Test: high sustained omega but no zero crossings => not oscillating ───────

TEST_F(OscillationDetectorTest, HighConstantOmegaWithoutCrossingsIsNotOscillating)
{
  detector_.setBufferLength(10);

  // omega stays positive => |omega_mean| > omega_eps, and zero crossings == 0.
  pushSamples(10, 0.0, 0.9, 0.1, 0.1);

  // Mean omega = 0.9 > eps=0.1, so detect() condition fails on omega_mean.
  EXPECT_FALSE(detector_.isOscillating());
}

// ── Test: capacity accessor ───────────────────────────────────────────────────

TEST_F(OscillationDetectorTest, CapacityReflectsSetBufferLength)
{
  detector_.setBufferLength(42);
  EXPECT_EQ(detector_.capacity(), 42u);

  detector_.setBufferLength(0);
  EXPECT_EQ(detector_.capacity(), 0u);
}

// ── Test: buffer does not grow beyond capacity ────────────────────────────────

TEST_F(OscillationDetectorTest, BufferDoesNotGrowBeyondCapacity)
{
  // Indirect test: if the buffer grew unbounded a steady input pushed after
  // an oscillating run would keep oscillating due to stale data.
  detector_.setBufferLength(6);

  // Phase 1: produce oscillation.
  pushAlternating(6, 0.0, 0.9);
  EXPECT_TRUE(detector_.isOscillating());

  // Phase 2: push steady forward motion to flush old oscillating samples.
  // After 6 fresh samples the old ones should be evicted.
  pushSamples(6, 0.8, 0.0);
  EXPECT_FALSE(detector_.isOscillating());
}

// ── Test: setBufferLength trims existing data ─────────────────────────────────

TEST_F(OscillationDetectorTest, ShrinkingBufferTrimsSamples)
{
  detector_.setBufferLength(20);
  // Fill with steady motion (not oscillating).
  pushSamples(20, 0.8, 0.0);
  EXPECT_FALSE(detector_.isOscillating());

  // Shrink to 4. The remaining samples are all steady forward -> still fine.
  detector_.setBufferLength(4);
  EXPECT_EQ(detector_.capacity(), 4u);
  // One more update to trigger detect().
  detector_.update(0.8, 0.0, 1.0, 1.0, 1.0, 0.1, 0.1);
  EXPECT_FALSE(detector_.isOscillating());
}

}  // namespace mowgli_nav2_plugins

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
