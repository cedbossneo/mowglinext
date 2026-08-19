// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Wheel-slip "digging" detector + bounded reverse escape. Pure logic, no ROS,
// so it is unit-testable standalone (see test_dig_detector.cpp) — same shape
// as mowgli_nav2_plugins/ftc_stall.hpp.
//
// ── Why this exists ─────────────────────────────────────────────────────────
// Every pre-existing "am I stuck?" check on this robot measures THE WHEELS,
// so none of them can see a dig — the failure where the wheels turn freely
// while the chassis goes nowhere and the tyres carve a hole in the lawn:
//
//   * Firmware anti-dig (cpp_main.cpp ANTIDIG_*) compares encoder ticks to
//     the travel the command implies. A SPINNING wheel produces the full
//     tick count, so the fraction test never trips. It catches a BLOCKED
//     wheel, not a slipping one — and that remains its job (it is the
//     un-bypassable backstop and stays exactly as it is).
//   * FTC's stall cap (ftc_stall.hpp) reads measured forward speed from
//     /wheel_odom (nav2_params_base.yaml controller.odom_topic), i.e. the
//     same encoders, which cheerfully confirm the phantom motion. So the
//     controller ramps to speed_fast and floors it — that IS what digs.
//   * fusion_graph's slip veto (graph_params.hpp slip_*) is ROTATIONAL only
//     (gyro yaw vs wheel yaw). A straight-line dig has gyro ~= 0 AND wheel
//     yaw ~= 0, so nothing is vetoed and the graph dead-reckons the robot
//     forward THROUGH the hole it is digging.
//
// The one signal that is independent of the wheels is the GNSS-anchored
// fused pose (/odometry/filtered_map). Over a short window, "encoders claim
// we travelled X, the map says we moved much less than X" is an unambiguous
// dig signature — no other failure mode produces it. That comparison is what
// this header makes.
//
// ── Trust gating ────────────────────────────────────────────────────────────
// The comparison is only as good as the fused pose. Under RTK-Fixed the
// position sigma is ~3 mm and a 0.3 m discrepancy is overwhelming evidence;
// under RTK-Float it can be metres and the same discrepancy proves nothing.
// So a sigma above max_pos_sigma SUPPRESSES detection (and resets the
// window) rather than guessing. This deliberately makes the detector
// self-disabling exactly when it would otherwise false-fire — the firmware
// backstop still covers the blocked-wheel case throughout.
//
// Only ever REDUCES commanded motion, and the escape only ever commands
// REVERSE, bounded in both distance and time.

#pragma once

#include <algorithm>
#include <cmath>

namespace mowgli_hardware
{

// ─────────────────────────────────────────────────────────────────────────────
// Detection
// ─────────────────────────────────────────────────────────────────────────────

struct DigDetectorCfg
{
  bool enabled = true;
  /// Sustained evidence required before latching a dig [s].
  double window_s = 1.2;
  /// Below this commanded speed the robot isn't being told to travel [m/s].
  double min_cmd_speed = 0.05;
  /// The encoders must claim at least this much travel in the window before
  /// we call it a dig [m]. Without it, a hard stall (wheels not turning) —
  /// which is the FIRMWARE's case, not ours — would look identical.
  double min_wheel_dist = 0.15;
  /// Latch when map travel is below this fraction of encoder travel.
  double progress_fraction = 0.35;
  /// Max fused-pose 1-sigma position uncertainty to trust the comparison [m].
  double max_pos_sigma = 0.10;
};

enum class DigAction
{
  kNone,
  kDig
};

/// Verdict plus the evidence that produced it. The evidence is returned
/// (rather than left in the state) because DigDecide resets its window on
/// every verdict — without this, a caller reading the state after a kDig
/// would report zeros in its logs and in the DigEvent it publishes.
struct DigVerdict
{
  DigAction action = DigAction::kNone;
  double wheel_dist = 0.0;  ///< encoder-claimed travel over the window [m]
  double map_dist = 0.0;  ///< fused-pose travel over the same window [m]
};

/// Caller-owned window accumulators (mirrors ftc_stall's in-place state).
struct DigDetectorState
{
  double window_time = 0.0;
  double wheel_dist = 0.0;
  double map_dist = 0.0;
};

inline void DigResetWindow(DigDetectorState& st)
{
  st = DigDetectorState{};
}

/// Feed one control tick.
///
/// @param cmd_speed  commanded forward speed this tick [m/s]
/// @param wheel_step encoder-derived distance travelled since last tick [m]
/// @param map_step   fused-pose (GNSS-anchored) distance since last tick [m]
/// @param pos_sigma  fused-pose 1-sigma position uncertainty [m]
/// @param dt         tick duration [s]
inline DigVerdict DigDecide(const DigDetectorCfg& cfg,
                            DigDetectorState& st,
                            double cmd_speed,
                            double wheel_step,
                            double map_step,
                            double pos_sigma,
                            double dt)
{
  if (!cfg.enabled || dt <= 0.0)
  {
    return {};
  }

  // Can't trust the only wheel-independent signal we have -> stand down.
  if (!(pos_sigma <= cfg.max_pos_sigma))  // NaN-safe: NaN sigma suppresses too
  {
    DigResetWindow(st);
    return {};
  }

  // Not commanded to travel -> nothing to compare against.
  if (std::abs(cmd_speed) < cfg.min_cmd_speed)
  {
    DigResetWindow(st);
    return {};
  }

  st.window_time += dt;
  st.wheel_dist += std::abs(wheel_step);
  st.map_dist += std::abs(map_step);

  if (st.window_time < cfg.window_s)
  {
    return {};  // not enough evidence yet
  }

  const bool wheels_claim_travel = st.wheel_dist >= cfg.min_wheel_dist;
  const bool map_disagrees = st.map_dist < cfg.progress_fraction * st.wheel_dist;

  const DigVerdict verdict{(wheels_claim_travel && map_disagrees) ? DigAction::kDig
                                                                  : DigAction::kNone,
                           st.wheel_dist,
                           st.map_dist};

  DigResetWindow(st);  // start a fresh window either way
  return verdict;
}

// ─────────────────────────────────────────────────────────────────────────────
// Bounded reverse escape
// ─────────────────────────────────────────────────────────────────────────────
//
// After a dig latches, the robot backs straight out of the hole it just made.
// The budget is bounded BOTH ways on purpose: distance is the intent, but the
// distance is integrated from the COMMANDED speed rather than the encoders,
// because the encoders are precisely the signal we just decided we cannot
// trust. If the tyres are still slipping, the timeout is what ends the
// manoeuvre. Reverse is chosen (not a turn) because it retraces ground the
// robot already occupied a moment ago and so is the least likely direction to
// find a new obstacle — the same reasoning as FTC's reverse-escape.

struct DigEscapeCfg
{
  double reverse_speed = 0.12;  ///< magnitude of the reverse command [m/s]
  double reverse_dist = 0.30;  ///< distance budget [m]
  double timeout_s = 4.0;  ///< hard time bound [s]
};

struct DigEscapeState
{
  double travelled = 0.0;  ///< commanded-integral distance so far [m]
  double elapsed = 0.0;  ///< time spent escaping [s]
};

inline bool DigEscapeDone(const DigEscapeCfg& cfg, const DigEscapeState& st)
{
  return st.travelled >= cfg.reverse_dist || st.elapsed >= cfg.timeout_s;
}

/// Advance the escape by one tick. Returns the forward velocity to command:
/// negative while escaping, exactly 0.0 once the budget is spent.
inline double DigEscapeStep(const DigEscapeCfg& cfg, DigEscapeState& st, double dt)
{
  if (dt <= 0.0 || DigEscapeDone(cfg, st))
  {
    return 0.0;
  }

  st.elapsed += dt;
  st.travelled = std::min(cfg.reverse_dist, st.travelled + std::abs(cfg.reverse_speed) * dt);

  return -std::abs(cfg.reverse_speed);
}

}  // namespace mowgli_hardware
