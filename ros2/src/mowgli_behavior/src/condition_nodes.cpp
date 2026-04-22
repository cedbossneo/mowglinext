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

#include "mowgli_behavior/condition_nodes.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "tf2/exceptions.h"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// IsEmergency
// ---------------------------------------------------------------------------

BT::NodeStatus IsEmergency::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  std::lock_guard<std::mutex> lock(ctx->context_mutex);

  // Fail-safe: if emergency data is stale (>2 s since last message),
  // treat as emergency so the robot stops.
  const auto age = std::chrono::steady_clock::now() - ctx->last_emergency_time;
  if (age > std::chrono::seconds(2))
  {
    return BT::NodeStatus::SUCCESS;  // stale data → assume emergency
  }

  return ctx->latest_emergency.active_emergency ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsCharging
// ---------------------------------------------------------------------------

BT::NodeStatus IsCharging::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  std::lock_guard<std::mutex> lock(ctx->context_mutex);
  return ctx->latest_power.charger_enabled ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsBatteryLow
// ---------------------------------------------------------------------------

BT::NodeStatus IsBatteryLow::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  std::lock_guard<std::mutex> lock(ctx->context_mutex);

  float threshold = 22.0f;
  if (auto res = getInput<float>("threshold"))
  {
    threshold = res.value();
  }

  return ctx->battery_percent < threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsRainDetected
// ---------------------------------------------------------------------------

BT::NodeStatus IsRainDetected::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  return ctx->latest_status.rain_detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// NeedsDocking
// ---------------------------------------------------------------------------

BT::NodeStatus NeedsDocking::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  std::lock_guard<std::mutex> lock(ctx->context_mutex);

  float threshold = 20.0f;
  if (auto res = getInput<float>("threshold"))
  {
    threshold = res.value();
  }

  return ctx->battery_percent <= threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsBatteryAbove
// ---------------------------------------------------------------------------

BT::NodeStatus IsBatteryAbove::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  std::lock_guard<std::mutex> lock(ctx->context_mutex);

  float threshold = 95.0f;
  if (auto res = getInput<float>("threshold"))
  {
    threshold = res.value();
  }

  return ctx->battery_percent >= threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsCommand
// ---------------------------------------------------------------------------

BT::NodeStatus IsCommand::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  auto res = getInput<uint8_t>("command");
  if (!res)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "IsCommand: missing required port 'command': %s",
                 res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  return ctx->current_command == res.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsGPSFixed
// ---------------------------------------------------------------------------

BT::NodeStatus IsGPSFixed::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  std::lock_guard<std::mutex> lock(ctx->context_mutex);
  return ctx->gps_is_fixed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// ReplanNeeded
// ---------------------------------------------------------------------------

BT::NodeStatus ReplanNeeded::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  return ctx->replan_needed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsBoundaryViolation
// ---------------------------------------------------------------------------

BT::NodeStatus IsBoundaryViolation::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  return ctx->boundary_violation ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsLethalBoundaryViolation
// ---------------------------------------------------------------------------

BT::NodeStatus IsLethalBoundaryViolation::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  return ctx->lethal_boundary_violation ? BT::NodeStatus::SUCCESS
                                        : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsNewRain
// ---------------------------------------------------------------------------

BT::NodeStatus IsNewRain::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  // Only trigger if it's raining NOW and it was NOT raining when mowing started.
  bool raining_now = ctx->latest_status.rain_detected;
  return (raining_now && !ctx->raining_at_mow_start) ? BT::NodeStatus::SUCCESS
                                                     : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsResumeUndockAllowed
// ---------------------------------------------------------------------------

BT::NodeStatus IsResumeUndockAllowed::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  int max_attempts = 3;
  if (auto res = getInput<int>("max_attempts"))
  {
    max_attempts = res.value();
  }

  if (ctx->resume_undock_failures >= max_attempts)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "IsResumeUndockAllowed: %d/%d resume-undock failures, aborting session",
                ctx->resume_undock_failures,
                max_attempts);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// IsChargingProgressing
// ---------------------------------------------------------------------------

BT::NodeStatus IsChargingProgressing::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  std::lock_guard<std::mutex> lock(ctx->context_mutex);

  // Once a charger failure is detected, keep returning FAILURE until the
  // next charging session resets the node via a fresh baseline.
  if (charger_failed_)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto now = std::chrono::steady_clock::now();
  const float current_battery = ctx->battery_percent;

  if (!baseline_set_)
  {
    baseline_battery_ = current_battery;
    baseline_time_ = now;
    baseline_set_ = true;
    charger_failed_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed = std::chrono::duration<double>(now - baseline_time_).count();

  if (elapsed < check_interval_sec_)
  {
    // Not enough time has passed yet — assume charging is OK.
    return BT::NodeStatus::SUCCESS;
  }

  // 30 minutes have passed — check progress.
  const float increase = current_battery - baseline_battery_;

  if (increase >= min_increase_)
  {
    // Good progress — reset baseline for the next window.
    baseline_battery_ = current_battery;
    baseline_time_ = now;
    return BT::NodeStatus::SUCCESS;
  }

  // No meaningful charge increase in 30 minutes — charger problem.
  // Set charger_failed_ so subsequent ticks fail immediately, allowing
  // RetryUntilSuccessful to exhaust quickly instead of waiting hours.
  RCLCPP_WARN(ctx->node->get_logger(),
              "IsChargingProgressing: battery only changed %.1f%% in 30 min "
              "(%.1f%% -> %.1f%%), charger may be broken",
              increase,
              baseline_battery_,
              current_battery);
  charger_failed_ = true;
  baseline_set_ = false;  // Reset for next charging session.
  return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// PreFlightCheck
// ---------------------------------------------------------------------------

BT::NodeStatus PreFlightCheck::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  float min_battery = 20.0f;
  int min_gps_fix_type = 2;
  double tf_timeout = 0.5;
  getInput<float>("min_battery", min_battery);
  getInput<int>("min_gps_fix_type", min_gps_fix_type);
  getInput<double>("tf_timeout_sec", tf_timeout);

  std::vector<std::string> failures;

  // ── 1. Emergency ─────────────────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    if (ctx->latest_emergency.active_emergency || ctx->latest_emergency.latched_emergency)
    {
      failures.emplace_back("emergency=active");
    }
  }

  // ── 2. Battery ───────────────────────────────────────────────────────────
  float battery;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    battery = ctx->battery_percent;
  }
  if (battery < min_battery)
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "battery=%.1f%% (need >=%.1f%%)", battery, min_battery);
    failures.emplace_back(buf);
  }

  // ── 3. GPS fix type ──────────────────────────────────────────────────────
  uint8_t fix_type;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    fix_type = ctx->gps_fix_type;
  }
  if (static_cast<int>(fix_type) < min_gps_fix_type)
  {
    const char* names[] = {"no-fix", "auto", "DGPS", "?", "RTK-fix", "RTK-float"};
    const char* current = (fix_type < 6) ? names[fix_type] : "?";
    char buf[80];
    snprintf(buf, sizeof(buf), "gps_fix=%s (%u, need >=%d)",
             current, fix_type, min_gps_fix_type);
    failures.emplace_back(buf);
  }

  // ── 4. TF chain: map → base_footprint resolvable ─────────────────────────
  try
  {
    ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout));
  }
  catch (const tf2::TransformException& ex)
  {
    failures.emplace_back(std::string("tf(map->base_footprint)=") + ex.what());
  }

  // ── 5. Mowing area defined ───────────────────────────────────────────────
  if (!coverage_client_)
  {
    coverage_client_ =
        ctx->helper_node->create_client<mowgli_interfaces::srv::GetCoverageStatus>(
            "/map_server_node/get_coverage_status");
  }
  if (!coverage_client_->service_is_ready())
  {
    failures.emplace_back("coverage-service-unavailable");
  }
  else
  {
    auto req = std::make_shared<mowgli_interfaces::srv::GetCoverageStatus::Request>();
    req->area_index = 0;
    auto future = coverage_client_->async_send_request(req);
    auto start = std::chrono::steady_clock::now();
    bool ready = false;
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(1))
    {
      if (future.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready)
      {
        ready = true;
        break;
      }
    }
    if (!ready)
    {
      failures.emplace_back("coverage-query-timeout");
    }
    else
    {
      auto resp = future.get();
      if (!resp || !resp->success)
      {
        failures.emplace_back("no-mowing-area-defined");
      }
    }
  }

  // ── Verdict ──────────────────────────────────────────────────────────────
  if (failures.empty())
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "PreFlightCheck PASS: battery=%.1f%% fix=%u area-ok tf-ok",
                battery, fix_type);
    return BT::NodeStatus::SUCCESS;
  }

  std::string all;
  for (size_t i = 0; i < failures.size(); ++i)
  {
    if (i) all += ", ";
    all += failures[i];
  }
  RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 3000,
                       "PreFlightCheck FAIL: %s", all.c_str());
  return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// Nav2Active
// ---------------------------------------------------------------------------

BT::NodeStatus Nav2Active::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  double timeout_sec = 0.5;
  if (auto res = getInput<double>("timeout_sec"))
  {
    timeout_sec = res.value();
  }

  if (!client_)
  {
    client_ = ctx->helper_node->create_client<std_srvs::srv::Trigger>(
        "/lifecycle_manager_navigation/is_active");
  }

  // Don't block if lifecycle_manager_navigation hasn't come up yet — treat
  // that the same as "not active". The retry loop above this node is
  // responsible for waiting, not us.
  if (!client_->service_is_ready())
  {
    RCLCPP_DEBUG(ctx->node->get_logger(),
                 "Nav2Active: is_active service not available yet");
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client_->async_send_request(req);

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout_sec);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (future.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready)
    {
      auto resp = future.get();
      if (resp && resp->success)
      {
        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 3000,
                           "Nav2Active: lifecycle_manager reports not-active "
                           "(msg=%s)",
                           resp ? resp->message.c_str() : "(null)");
      return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 3000,
                       "Nav2Active: is_active call timed out after %.2fs",
                       timeout_sec);
  return BT::NodeStatus::FAILURE;
}

}  // namespace mowgli_behavior
