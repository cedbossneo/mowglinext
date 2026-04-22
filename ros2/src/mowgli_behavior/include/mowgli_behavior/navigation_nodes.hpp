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

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// StopMoving
// ---------------------------------------------------------------------------

/// Publishes zero velocity to /cmd_vel_emergency (twist_mux priority 100) to
/// halt the robot immediately. Goes through twist_mux so it respects the
/// priority ladder rather than racing with other publishers on /cmd_vel.
class StopMoving : public BT::SyncActionNode
{
public:
  StopMoving(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

// ---------------------------------------------------------------------------
// ClearCostmap
// ---------------------------------------------------------------------------

/// Calls the Nav2 clear_entirely service on both the global and local costmaps.
///
/// This is a synchronous fire-and-forget node: it sends both service requests
/// without waiting for responses (the node is already being spun by the main
/// executor), then returns SUCCESS immediately.  Useful after obstacle removal
/// to let the planner see a clean costmap before retrying coverage.
class ClearCostmap : public BT::SyncActionNode
{
public:
  ClearCostmap(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr local_client_;
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
class NavigateToPose : public BT::StatefulActionNode
{
public:
  using Nav2Goal = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Goal>;

  NavigateToPose(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("goal", "Target pose as 'x;y;yaw'")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Goal>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;

  /// Lazily creates the action client once and reuses it across ticks.
  void ensureActionClient(const rclcpp::Node::SharedPtr& node);
};

// ---------------------------------------------------------------------------
// BackUp
// ---------------------------------------------------------------------------

/// Calls the Nav2 /backup action to reverse the robot by a given distance.
/// Used for undocking (reverse away from charger) and recovery (back away
/// from unseen obstacles).
///
/// Input ports:
///   backup_dist  (double, default "0.5") – distance to reverse in metres.
///   backup_speed (double, default "0.15") – reverse speed in m/s.
class BackUp : public BT::StatefulActionNode
{
public:
  using BackUpAction = nav2_msgs::action::BackUp;
  using GoalHandle = rclcpp_action::ClientGoalHandle<BackUpAction>;

  BackUp(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("backup_dist", 0.5, "Distance to reverse (m)"),
            BT::InputPort<double>("backup_speed", 0.15, "Reverse speed (m/s)")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using WrappedResult = rclcpp_action::ClientGoalHandle<BackUpAction>::WrappedResult;

  rclcpp_action::Client<BackUpAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<WrappedResult> result_future_;
  bool result_requested_{false};
};

// ---------------------------------------------------------------------------
// SetNavMode
// ---------------------------------------------------------------------------

/// Dynamically adjusts Nav2 controller speed and costmap inflation based on
/// GPS quality.  "precise" = full speed, "degraded" = half speed + wider
/// inflation.
///
/// Input ports:
///   mode (string) – "precise" or "degraded"
class SetNavMode : public BT::SyncActionNode
{
public:
  SetNavMode(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("mode", "precise", "Navigation mode: precise or degraded")};
  }

  BT::NodeStatus tick() override;
};

}  // namespace mowgli_behavior
