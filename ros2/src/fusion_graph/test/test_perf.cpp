// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Performance benchmarks for fusion_graph_core. Single-threaded
// micro-benchmarks targeting the per-Tick hot path that dominates
// fusion_graph_node CPU when the graph passes ~3 k nodes.
//
// Each test prints a one-line summary so regressions in CI logs are
// grep-able. They also assert loose ceilings so a major regression
// (e.g. accidentally O(N²) relinearization) fails the test.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <random>
#include <vector>

#include "fusion_graph/graph_manager.hpp"
#include "fusion_graph/scan_matcher.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

// Synthetic LiDAR scan: closed asymmetric L-shape, used as both source
// and target with small relative motion. Mirrors the live geometry
// closely enough for ICP cost estimates.
std::vector<Eigen::Vector2d> SyntheticScan(int n, const gtsam::Pose2& T)
{
  std::vector<Eigen::Vector2d> pts;
  pts.reserve(n);
  for (int i = 0; i < n / 2; ++i)
  {
    const double t = -2.0 + 4.0 * static_cast<double>(i) / static_cast<double>(n / 2);
    pts.emplace_back(t, 2.0);
  }
  for (int i = 0; i < n / 2; ++i)
  {
    const double t = -2.0 + 3.0 * static_cast<double>(i) / static_cast<double>(n / 2);
    pts.emplace_back(2.0, t);
  }
  for (auto& p : pts)
  {
    const double c = std::cos(T.theta());
    const double s = std::sin(T.theta());
    Eigen::Vector2d q(T.x() + c * p.x() - s * p.y(), T.y() + s * p.x() + c * p.y());
    p = q;
  }
  return pts;
}

template <typename F>
double TimeMillis(F&& fn)
{
  const auto t0 = std::chrono::steady_clock::now();
  fn();
  const auto t1 = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// Baseline: pure node creation through Tick, no scans, no LC.
// Measures iSAM2 update + calculateEstimate cost as N grows.
// Expected: ~constant per-tick once tree is warm. Sub-linear growth
// with N is OK; super-linear is a red flag.
// ─────────────────────────────────────────────────────────────────────
TEST(Perf, BareTickThroughput)
{
  fg::GraphParams gp;
  gp.cov_update_every_n = 10;
  gp.isam2_relinearize_skip = 5;
  fg::GraphManager gm(gp);
  gm.Initialize(gtsam::Pose2(), 0.0);

  constexpr int kN = 2000;
  std::vector<double> per_tick_ms;
  per_tick_ms.reserve(kN);

  for (int i = 0; i < kN; ++i)
  {
    // Simulate 0.1 s of straight-line motion @ 1 m/s + tight RTK GPS.
    gm.AddWheelTwist(1.0, 0.0, 0.0, 0.1);
    gm.AddGyroDelta(0.0, 0.1);
    gm.QueueGnss(0.1 * (i + 1), 0.0, 0.005);
    const double now_s = 0.1 * (i + 1);
    double dt = TimeMillis(
        [&]
        {
          gm.Tick(now_s);
        });
    per_tick_ms.push_back(dt);
  }

  std::sort(per_tick_ms.begin(), per_tick_ms.end());
  const double med = per_tick_ms[kN / 2];
  const double p95 = per_tick_ms[kN * 95 / 100];
  const double last100_avg =
      std::accumulate(per_tick_ms.end() - 100, per_tick_ms.end(), 0.0) / 100.0;
  std::printf(
      "[Perf] BareTickThroughput N=%d  median=%.2f ms  p95=%.2f ms  "
      "last100_avg=%.2f ms\n",
      kN,
      med,
      p95,
      last100_avg);

  // Loose ceiling: median tick under 30 ms even on slow ARM. If we
  // regress to >30 ms median the 10 Hz publish target is unreachable.
  EXPECT_LT(med, 30.0);
}

// ─────────────────────────────────────────────────────────────────────
// Realistic load: every node has a scan attached + a scan-match
// between-factor + occasional loop closure. Mirrors the live mowing
// session that pegged CPU at 100%.
// ─────────────────────────────────────────────────────────────────────
TEST(Perf, MowingSessionWithScansAndLC)
{
  fg::GraphParams gp;
  gp.cov_update_every_n = 10;
  gp.isam2_relinearize_skip = 5;
  fg::GraphManager gm(gp);
  fg::ScanMatcher matcher;
  gm.Initialize(gtsam::Pose2(), 0.0);

  constexpr int kN = 1500;
  constexpr int kScanPts = 200;

  // Pre-generate scans on a closed 30-m square track so mid-session
  // nodes spatially overlap with early nodes — this is what triggers
  // loop closures in the live workload.
  std::vector<std::vector<Eigen::Vector2d>> all_scans(kN);
  for (int i = 0; i < kN; ++i)
  {
    all_scans[i] = SyntheticScan(kScanPts, gtsam::Pose2());
  }

  std::vector<double> per_tick_ms;
  per_tick_ms.reserve(kN);

  for (int i = 0; i < kN; ++i)
  {
    // Square loop trajectory: 30 m forward, then back.
    const double phase = (2.0 * M_PI * i) / 600.0;
    const double vx = std::cos(phase);
    const double vy = 0.5 * std::sin(phase);
    gm.AddWheelTwist(vx, vy, 0.0, 0.1);
    gm.AddGyroDelta(0.0, 0.1);

    // GPS at every node (RTK Fixed σ ~5 mm).
    const double cumx = 0.05 * std::sin(phase * 2);
    const double cumy = 0.05 * std::cos(phase * 2);
    gm.QueueGnss(cumx, cumy, 0.005);

    // Pretend ICP gave a tight between with rmse 5 cm.
    if (i > 0)
    {
      gm.QueueScanBetween(gtsam::Pose2(0.1, 0.0, 0.0), 0.05, 0.005);
    }

    const double now_s = 0.1 * (i + 1);
    double dt = TimeMillis(
        [&]
        {
          auto out = gm.Tick(now_s);
          if (out)
          {
            gm.AttachScan(out->node_index, all_scans[i]);
            // Periodic rebase mimics the node's 30 s maintenance timer.
            if (i > 0 && (i % 500) == 0)
            {
              gm.RebaseISAM2();
            }
            // LC against an early node every 50 nodes once we're past
            // the warmup.
            if (i > 200 && i % 50 == 0)
            {
              auto cands = gm.FindLoopClosureCandidates(out->node_index, 5.0, 5.0, 1);
              if (!cands.empty())
              {
                auto cand_scan = gm.GetScan(cands[0]);
                auto cand_pose = gm.GetPose(cands[0]);
                if (!cand_scan.empty() && cand_pose)
                {
                  auto res = matcher.Match(cand_scan, all_scans[i], cand_pose->between(out->pose));
                  if (res.ok)
                  {
                    gm.AddLoopClosure(cands[0], out->node_index, res.delta, 0.05, 0.02);
                  }
                }
              }
            }
          }
        });
    per_tick_ms.push_back(dt);
  }

  std::sort(per_tick_ms.begin(), per_tick_ms.end());
  const double med = per_tick_ms[kN / 2];
  const double p95 = per_tick_ms[kN * 95 / 100];
  const double last100_avg =
      std::accumulate(per_tick_ms.end() - 100, per_tick_ms.end(), 0.0) / 100.0;
  const auto stats = gm.Stats();

  std::printf(
      "[Perf] MowingSession N=%d  nodes=%lu  scans=%lu  LC=%lu  "
      "median=%.2f ms  p95=%.2f ms  last100_avg=%.2f ms\n",
      kN,
      (unsigned long)stats.total_nodes,
      (unsigned long)stats.scans_attached,
      (unsigned long)stats.loop_closures,
      med,
      p95,
      last100_avg);

  // A 10 Hz publisher needs median << 100 ms. We aim for <50 ms
  // sustained (50% headroom for OnTimer overhead, ICP, etc).
  EXPECT_LT(last100_avg, 50.0);
}

// ─────────────────────────────────────────────────────────────────────
// ScanMatcher cost on the live-shape input. Sets the floor for how
// fast we can run scan-matching in the OnTimer hot path.
// ─────────────────────────────────────────────────────────────────────
TEST(Perf, ScanMatcherSingle)
{
  fg::ScanMatcher matcher;
  auto src = SyntheticScan(360, gtsam::Pose2());  // live LiDAR ~360 pts
  auto tgt = SyntheticScan(360, gtsam::Pose2(0.03, 0.02, 0.02));

  constexpr int kN = 200;
  std::vector<double> per_match_ms;
  per_match_ms.reserve(kN);
  for (int i = 0; i < kN; ++i)
  {
    double dt = TimeMillis(
        [&]
        {
          (void)matcher.Match(src, tgt, gtsam::Pose2());
        });
    per_match_ms.push_back(dt);
  }
  std::sort(per_match_ms.begin(), per_match_ms.end());
  const double med = per_match_ms[kN / 2];
  const double p95 = per_match_ms[kN * 95 / 100];
  std::printf("[Perf] ScanMatcher N=%d  median=%.2f ms  p95=%.2f ms\n", kN, med, p95);
  // ARM target: ICP under 20 ms median to stay <20% CPU at 10 Hz.
  EXPECT_LT(med, 20.0);
}
