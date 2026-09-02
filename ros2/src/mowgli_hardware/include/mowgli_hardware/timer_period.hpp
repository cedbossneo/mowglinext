// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace mowgli_hardware
{

using TimerDuration = std::chrono::nanoseconds;

inline void require_finite_positive(double value, const char* name)
{
  if (!std::isfinite(value) || value <= 0.0)
  {
    throw std::invalid_argument(std::string{name} + " must be finite and positive");
  }
}

inline void require_finite_nonnegative_timeout(double value, const char* name)
{
  if (!std::isfinite(value) || value < 0.0)
  {
    throw std::invalid_argument(std::string{name} + " must be finite and non-negative");
  }
}

/** Lowest positive rate whose reciprocal is representable as TimerDuration. */
inline constexpr double minimum_timer_rate_hz()
{
  return 1.0e9 / static_cast<double>(TimerDuration::max().count());
}

/**
 * Convert a timer cadence in Hz to a positive, representable wall-timer
 * duration.  Nanoseconds preserve sub-millisecond cadence.  Rates faster than
 * one tick per nanosecond intentionally saturate at one nanosecond rather
 * than producing a zero-duration busy loop.
 */
inline TimerDuration timer_period_from_rate_hz(double rate_hz)
{
  if (!std::isfinite(rate_hz) || rate_hz < minimum_timer_rate_hz())
  {
    throw std::invalid_argument(
        "timer rate must be finite, positive, and have a representable period");
  }

  const long double period_ns = 1.0e9L / static_cast<long double>(rate_hz);
  if (period_ns <= 1.0L)
  {
    return TimerDuration{1};
  }

  // The lower-rate bound above makes the cast representable.  Truncation is
  // deliberate: it preserves the historic exact periods for integral-ms
  // defaults while allowing sub-millisecond rates.
  return TimerDuration{static_cast<TimerDuration::rep>(period_ns)};
}

}  // namespace mowgli_hardware
