// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// Transit-failure classification (issue #487)
//
// Every blade-off NavigateToPose transit that aborts used to be treated
// identically by FollowStrip: "transit failed — skipping". That collapsed two
// completely different conditions into one:
//
//   * the GOAL is unreachable (obstacle in the way, no valid path, timeout) —
//     the swath genuinely cannot be mowed on this pass, skipping is right;
//   * the ROBOT'S OWN POSE is a lethal cell (START_OCCUPIED) — no plan can ever
//     be produced from where it stands, so EVERY sub-path start fails in a row
//     and the whole area is forfeited even though the field is perfectly
//     mowable. Observed 2026-08-24 (attempt 1 cut zero grass): the robot
//     undocked into the inflated keepout around a 0.25 m obstacle circle and
//     SmacPlanner2D answered "Start occupied" to all 26 planning calls.
//
// Where the code comes from (verified against the nav2 version in use, Kilted):
//   * nav2_msgs/action/NavigateToPose.action has `uint16 error_code` +
//     `string error_msg` in its RESULT (only NONE/UNKNOWN/
//     FAILED_TO_LOAD_BEHAVIOR_TREE/TF_ERROR/TIMEOUT are declared on that
//     action itself).
//   * nav2_behavior_tree's BtActionServer::populateErrorCode copies the
//     highest-priority (lowest-numbered) non-zero `<prefix>_error_code`
//     blackboard entry into that result, for every prefix in
//     `error_code_name_prefixes` (default list includes "compute_path").
//   * trees/navigate_to_pose.xml wires ComputePathToPose with
//     error_code_id="{compute_path_error_code}", so the planner's
//     ComputePathToPose::Result::START_OCCUPIED (205) reaches the
//     NavigateToPose result verbatim.
//
// The message fallback exists because that propagation is configuration-
// dependent (a robot whose bt_navigator drops "compute_path" from
// error_code_name_prefixes, or a tree that omits error_code_id, reports
// UNKNOWN). nav2_smac_planner throws nav2_core::StartOccupied("Start occupied")
// and planner_server puts that text in error_msg either way, so matching it is
// a strictly additional signal — never the only one.
// ---------------------------------------------------------------------------

/// What made a blade-off NavigateToPose transit fail.
enum class TransitFailure : uint8_t
{
  /// No result was available to classify (result never arrived, or the goal was
  /// rejected before it ever ran).
  kUnknown = 0,
  /// The planner refused because the ROBOT'S CURRENT CELL is lethal. Retrying
  /// from the same pose is futile — every goal fails identically.
  kStartOccupied,
  /// The GOAL cell is lethal. Specific to this swath start; other swaths may
  /// still be reachable.
  kGoalOccupied,
  /// Planner searched and found nothing (obstacle field, walled-off region).
  kNoValidPath,
  /// Planning or navigation timed out.
  kTimeout,
  /// TF was unavailable/late.
  kTfError,
  /// A real error code that is none of the above.
  kOther,
};

/// Human-readable tag for logs. Stable strings — field logs are grepped for
/// these.
inline const char* transitFailureName(TransitFailure kind)
{
  switch (kind)
  {
    case TransitFailure::kStartOccupied:
      return "START_OCCUPIED";
    case TransitFailure::kGoalOccupied:
      return "GOAL_OCCUPIED";
    case TransitFailure::kNoValidPath:
      return "NO_VALID_PATH";
    case TransitFailure::kTimeout:
      return "TIMEOUT";
    case TransitFailure::kTfError:
      return "TF_ERROR";
    case TransitFailure::kOther:
      return "OTHER";
    case TransitFailure::kUnknown:
    default:
      return "UNKNOWN";
  }
}

namespace detail
{

/// ASCII-lowercase copy — the planner's message casing is not part of any API
/// contract, so the substring fallback must not depend on it.
inline std::string asciiLower(const std::string& s)
{
  std::string out = s;
  std::transform(out.begin(),
                 out.end(),
                 out.begin(),
                 [](unsigned char c)
                 {
                   return static_cast<char>(std::tolower(c));
                 });
  return out;
}

}  // namespace detail

/// Classify a NavigateToPose result. `error_code` / `error_msg` are the fields
/// of nav2_msgs::action::NavigateToPose::Result. Pure — unit-testable without
/// ROS running.
///
/// The numeric code wins when it is one nav2 actually defines; the message text
/// is only consulted when the code carries no information (0 = NONE, or the
/// NavigateToPose-level UNKNOWN placeholder), which is exactly the case where
/// the sub-action code failed to propagate.
inline TransitFailure classifyTransitFailure(uint16_t error_code, const std::string& error_msg)
{
  using ComputePath = nav2_msgs::action::ComputePathToPose::Result;
  using Navigate = nav2_msgs::action::NavigateToPose::Result;

  switch (error_code)
  {
    case ComputePath::START_OCCUPIED:
      return TransitFailure::kStartOccupied;
    case ComputePath::GOAL_OCCUPIED:
      return TransitFailure::kGoalOccupied;
    case ComputePath::NO_VALID_PATH:
      return TransitFailure::kNoValidPath;
    case ComputePath::TIMEOUT:
      return TransitFailure::kTimeout;
    case ComputePath::TF_ERROR:
      return TransitFailure::kTfError;
    default:
      break;
  }
  if (error_code == Navigate::TIMEOUT)
  {
    return TransitFailure::kTimeout;
  }
  if (error_code == Navigate::TF_ERROR)
  {
    return TransitFailure::kTfError;
  }

  // No usable code — fall back to the planner's own words. Only the
  // start-occupied phrase is matched: it is the one condition whose handling
  // differs, and nav2_smac_planner's exception text ("Start occupied") is
  // stable across the supported versions.
  const bool code_is_uninformative = (error_code == ComputePath::NONE) ||
                                     (error_code == Navigate::UNKNOWN) ||
                                     (error_code == ComputePath::UNKNOWN);
  if (code_is_uninformative && !error_msg.empty())
  {
    const std::string lowered = detail::asciiLower(error_msg);
    if (lowered.find("start occupied") != std::string::npos)
    {
      return TransitFailure::kStartOccupied;
    }
  }

  if (error_code == ComputePath::NONE)
  {
    return TransitFailure::kUnknown;
  }
  return TransitFailure::kOther;
}

/// True when the failure means "the robot is standing somewhere the planner
/// refuses to plan from" — i.e. nothing about the SWATH is wrong. FollowStrip
/// uses this to tell a forfeit-the-field condition apart from a genuinely
/// unmowable area.
inline bool isStartPoseBlocked(TransitFailure kind)
{
  return kind == TransitFailure::kStartOccupied;
}

}  // namespace mowgli_behavior
