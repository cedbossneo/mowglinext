#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "mowgli_behavior/bt_context.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// IsEmergency
// ---------------------------------------------------------------------------

/// Returns SUCCESS when an active emergency is flagged in the hardware bridge.
class IsEmergency : public BT::ConditionNode
{
public:
  IsEmergency(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsCharging
// ---------------------------------------------------------------------------

/// Returns SUCCESS when the charger relay is enabled (robot is on the dock).
class IsCharging : public BT::ConditionNode
{
public:
  IsCharging(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsBatteryLow
// ---------------------------------------------------------------------------

/// Returns SUCCESS when battery_percent falls below the given threshold.
///
/// Input ports:
///   threshold (float, default "22.0") – low-battery warning level in percent.
class IsBatteryLow : public BT::ConditionNode
{
public:
  IsBatteryLow(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<float>("threshold", 22.0f, "Low-battery threshold in percent")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsRainDetected
// ---------------------------------------------------------------------------

/// Returns SUCCESS when the rain sensor reports rain.
class IsRainDetected : public BT::ConditionNode
{
public:
  IsRainDetected(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// NeedsDocking
// ---------------------------------------------------------------------------

/// Returns SUCCESS when battery_percent is at or below the docking threshold.
///
/// Input ports:
///   threshold (float, default "20.0") – return-to-dock battery level in percent.
class NeedsDocking : public BT::ConditionNode
{
public:
  NeedsDocking(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<float>("threshold", 20.0f, "Docking threshold in percent")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsBatteryAbove
// ---------------------------------------------------------------------------

/// Returns SUCCESS when battery_percent is at or above the given threshold.
/// Used to wait until the battery is sufficiently charged before resuming.
///
/// Input ports:
///   threshold (float, default "95.0") – battery percent to consider "charged".
class IsBatteryAbove : public BT::ConditionNode
{
public:
  IsBatteryAbove(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<float>("threshold", 95.0f, "Battery percent threshold")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsCommand
// ---------------------------------------------------------------------------

/// Returns SUCCESS when context.current_command equals the requested command.
///
/// Input ports:
///   command (uint8_t) – expected command value (see HighLevelControl.srv).
class IsCommand : public BT::ConditionNode
{
public:
  IsCommand(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<uint8_t>("command", "Expected HighLevelControl command value")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsGPSFixed
// ---------------------------------------------------------------------------

/// Returns SUCCESS when GPS has RTK fixed quality (high precision).
class IsGPSFixed : public BT::ConditionNode
{
public:
  IsGPSFixed(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// ReplanNeeded
// ---------------------------------------------------------------------------

/// Returns SUCCESS when the obstacle map has changed and a coverage replan
/// is required.  The flag is set by the main node from the
/// /map_server_node/replan_needed topic.
class ReplanNeeded : public BT::ConditionNode
{
public:
  ReplanNeeded(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsBoundaryViolation
// ---------------------------------------------------------------------------

/// Returns SUCCESS when the robot is detected outside the allowed mowing area.
class IsBoundaryViolation : public BT::ConditionNode
{
public:
  IsBoundaryViolation(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsNewRain
// ---------------------------------------------------------------------------

/// Returns SUCCESS when rain is currently detected AND it was NOT raining
/// when mowing started (i.e., rain is new since mow start).
class IsNewRain : public BT::ConditionNode
{
public:
  IsNewRain(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

}  // namespace mowgli_behavior
