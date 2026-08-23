// Copyright (C) 2026 MowgliNext contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#ifndef MOWGLI_MAP__MOW_PROGRESS_HPP_
#define MOWGLI_MAP__MOW_PROGRESS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace mowgli_map
{

enum class MowProgressInhibitReason
{
  kActive,
  kBladeNotRequested,
  kTelemetryMissingOrStale,
  kBladeInactive,
  kRpmTooLow,
};

/// Decide whether a pose may be added to the "actually mowed" overlay.
///
/// This deliberately requires both command intent and fresh blade-controller
/// evidence.  A missing or stale telemetry timestamp is never treated as a
/// positive observation.
inline MowProgressInhibitReason GetMowProgressInhibitReason(bool blade_requested,
                                                            bool telemetry_fresh,
                                                            bool blade_active,
                                                            double blade_rpm,
                                                            double min_blade_rpm)
{
  if (!blade_requested)
  {
    return MowProgressInhibitReason::kBladeNotRequested;
  }
  if (!telemetry_fresh)
  {
    return MowProgressInhibitReason::kTelemetryMissingOrStale;
  }
  if (!blade_active)
  {
    return MowProgressInhibitReason::kBladeInactive;
  }
  if (!std::isfinite(blade_rpm) || blade_rpm < min_blade_rpm)
  {
    return MowProgressInhibitReason::kRpmTooLow;
  }
  return MowProgressInhibitReason::kActive;
}

inline const char* ToString(MowProgressInhibitReason reason)
{
  switch (reason)
  {
    case MowProgressInhibitReason::kActive:
      return "verified blade activity";
    case MowProgressInhibitReason::kBladeNotRequested:
      return "blade not requested";
    case MowProgressInhibitReason::kTelemetryMissingOrStale:
      return "blade telemetry missing or stale";
    case MowProgressInhibitReason::kBladeInactive:
      return "blade controller reports inactive";
    case MowProgressInhibitReason::kRpmTooLow:
      return "blade RPM below threshold";
  }
  return "unknown";
}

/// Number of evenly spaced disc centres needed to cover a segment without a
/// gap larger than one grid cell. The endpoint is included by callers.
inline std::size_t SweepStepCount(double distance, double resolution)
{
  return std::max<std::size_t>(1,
                               static_cast<std::size_t>(
                                   std::ceil(distance / std::max(resolution, 1e-9))));
}

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__MOW_PROGRESS_HPP_
