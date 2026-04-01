#include "mowgli_behavior/condition_nodes.hpp"

#include <memory>

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// IsEmergency
// ---------------------------------------------------------------------------

BT::NodeStatus IsEmergency::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  return ctx->latest_emergency.active_emergency ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsCharging
// ---------------------------------------------------------------------------

BT::NodeStatus IsCharging::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  return ctx->latest_power.charger_enabled ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// IsBatteryLow
// ---------------------------------------------------------------------------

BT::NodeStatus IsBatteryLow::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

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

}  // namespace mowgli_behavior
