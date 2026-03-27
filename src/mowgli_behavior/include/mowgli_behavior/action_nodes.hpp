#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "mowgli_interfaces/srv/plan_coverage_path.hpp"

namespace mowgli_behavior {

// ---------------------------------------------------------------------------
// SetMowerEnabled
// ---------------------------------------------------------------------------

/// Calls the /hardware_bridge/mower_control service to enable or disable the
/// cutting blade motor.
///
/// Input ports:
///   enabled (bool) – true to start the blade, false to stop it.
class SetMowerEnabled : public BT::SyncActionNode {
public:
  SetMowerEnabled(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>("enabled", "Enable (true) or disable (false) the mow motor")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// StopMoving
// ---------------------------------------------------------------------------

/// Publishes a zero-velocity Twist to /cmd_vel to halt the robot immediately.
class StopMoving : public BT::SyncActionNode {
public:
  StopMoving(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

// ---------------------------------------------------------------------------
// PublishHighLevelStatus
// ---------------------------------------------------------------------------

/// Publishes a HighLevelStatus message to ~/high_level_status.
///
/// Input ports:
///   state      (uint8_t) – HIGH_LEVEL_STATE_* constant.
///   state_name (string)  – Human-readable sub_state_name string.
class PublishHighLevelStatus : public BT::SyncActionNode {
public:
  PublishHighLevelStatus(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<uint8_t>("state",      "HIGH_LEVEL_STATE_* value"),
      BT::InputPort<std::string>("state_name", "Human-readable state label")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr pub_;
};

// ---------------------------------------------------------------------------
// WaitForDuration
// ---------------------------------------------------------------------------

/// Stateful action that returns RUNNING until the requested duration has
/// elapsed, then returns SUCCESS.
///
/// Input ports:
///   duration_sec (double, default "1.0") – wait duration in seconds.
class WaitForDuration : public BT::StatefulActionNode {
public:
  WaitForDuration(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("duration_sec", 1.0, "Duration to wait in seconds")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void           onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::duration<double>         duration_;
};

// ---------------------------------------------------------------------------
// NavigateToPose
// ---------------------------------------------------------------------------

/// Stateful action that sends a goal to the Nav2 NavigateToPose action server
/// and waits for it to complete.
///
/// Input ports:
///   goal (string) – target pose encoded as "x;y;yaw" (metres / radians,
///                   frame_id = "map").
class NavigateToPose : public BT::StatefulActionNode {
public:
  using Nav2Goal   = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Goal>;

  NavigateToPose(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("goal", "Target pose as 'x;y;yaw'")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void           onHalted() override;

private:
  rclcpp_action::Client<Nav2Goal>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr>  goal_handle_future_;
  GoalHandle::SharedPtr                      goal_handle_;

  /// Lazily creates the action client once and reuses it across ticks.
  void ensureActionClient(const rclcpp::Node::SharedPtr& node);
};

// ---------------------------------------------------------------------------
// PlanCoveragePath
// ---------------------------------------------------------------------------

/// Calls the coverage planner service asynchronously and waits for the result.
///
/// Input ports:
///   area_index (uint32_t, default "0") – index of the mowing area to plan.
class PlanCoveragePath : public BT::StatefulActionNode {
public:
  PlanCoveragePath(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index to plan"),
      BT::InputPort<std::string>("boundary", "", "Mowing boundary points (optional)"),
      BT::OutputPort<std::string>("first_waypoint", "First waypoint as 'x;y;yaw'")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void           onHalted() override;

private:
  rclcpp::Client<mowgli_interfaces::srv::PlanCoveragePath>::SharedPtr client_;
  rclcpp::Client<mowgli_interfaces::srv::PlanCoveragePath>::SharedFuture future_;
  uint32_t area_index_{0};
};

// ---------------------------------------------------------------------------
// FollowCoveragePath
// ---------------------------------------------------------------------------

/// Subscribes to the coverage path topic, sends it to Nav2 FollowPath action,
/// and returns SUCCESS when the robot has traversed the entire path.
///
/// Input ports:
///   path_topic (string, default "/coverage_planner_node/coverage_path")
class FollowCoveragePath : public BT::StatefulActionNode {
public:
  using FollowPathAction = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPathAction>;

  FollowCoveragePath(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("path_topic",
        "/coverage_planner_node/coverage_path",
        "Topic with the coverage path to follow")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void           onHalted() override;

private:
  rclcpp_action::Client<FollowPathAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path latest_path_;
  bool path_received_{false};
};

// ---------------------------------------------------------------------------
// ClearCommand
// ---------------------------------------------------------------------------

/// Resets the current_command in BTContext to 0 (no command), preventing the
/// BT from re-entering a completed sequence.
class ClearCommand : public BT::SyncActionNode {
public:
  ClearCommand(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// Free registration helper
// ---------------------------------------------------------------------------

/// Registers all mowgli_behavior BT nodes with the given factory.
void registerAllNodes(BT::BehaviorTreeFactory& factory);

}  // namespace mowgli_behavior
