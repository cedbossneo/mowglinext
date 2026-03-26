#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/power.hpp"

namespace mowgli_behavior {

/// Shared context passed to all BehaviorTree nodes via the blackboard.
///
/// The main node keeps this struct alive and updates it from ROS2 topic
/// callbacks before each tree tick.  BT nodes retrieve a shared_ptr to
/// this struct with:
///
///   auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
struct BTContext {
  /// ROS2 node used by action/service nodes to create clients.
  rclcpp::Node::SharedPtr node;

  // -----------------------------------------------------------------------
  // Latest sensor state (updated by topic subscribers in the main node)
  // -----------------------------------------------------------------------

  mowgli_interfaces::msg::Status    latest_status;
  mowgli_interfaces::msg::Emergency latest_emergency;
  mowgli_interfaces::msg::Power     latest_power;

  // -----------------------------------------------------------------------
  // Command state (set by HighLevelControl service handler)
  // -----------------------------------------------------------------------

  /// Last command received via the ~/high_level_control service.
  /// Constants match HighLevelControl.srv (COMMAND_START=1, COMMAND_HOME=2,
  /// COMMAND_S1=3, COMMAND_S2=4, COMMAND_RESET_EMERGENCY=254, …).
  uint8_t current_command{0};

  // -----------------------------------------------------------------------
  // Derived / convenience fields (computed from latest_* messages)
  // -----------------------------------------------------------------------

  float battery_percent{100.0f};
  float gps_quality{0.0f};

  // -----------------------------------------------------------------------
  // Docking point (set from parameter or service call)
  // -----------------------------------------------------------------------

  double dock_x{0.0};
  double dock_y{0.0};
  double dock_yaw{0.0};
};

}  // namespace mowgli_behavior
