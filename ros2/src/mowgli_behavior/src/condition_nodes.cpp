#include "mowgli_behavior/condition_nodes.hpp"

#include <chrono>
#include <memory>
#include <mutex>

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
// IsChargingProgressing
// ---------------------------------------------------------------------------

BT::NodeStatus IsChargingProgressing::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  std::lock_guard<std::mutex> lock(ctx->context_mutex);

  const auto now = std::chrono::steady_clock::now();
  const float current_battery = ctx->battery_percent;

  if (!baseline_set_) {
    baseline_battery_ = current_battery;
    baseline_time_ = now;
    baseline_set_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed =
      std::chrono::duration<double>(now - baseline_time_).count();

  if (elapsed < check_interval_sec_) {
    // Not enough time has passed yet — assume charging is OK.
    return BT::NodeStatus::SUCCESS;
  }

  // 30 minutes have passed — check progress.
  const float increase = current_battery - baseline_battery_;

  if (increase >= min_increase_) {
    // Good progress — reset baseline for the next window.
    baseline_battery_ = current_battery;
    baseline_time_ = now;
    return BT::NodeStatus::SUCCESS;
  }

  // No meaningful charge increase in 30 minutes — charger problem.
  RCLCPP_WARN(ctx->node->get_logger(),
    "IsChargingProgressing: battery only changed %.1f%% in 30 min "
    "(%.1f%% -> %.1f%%), charger may be broken",
    increase, baseline_battery_, current_battery);
  baseline_set_ = false;  // Reset for next charging session.
  return BT::NodeStatus::FAILURE;
}

}  // namespace mowgli_behavior
