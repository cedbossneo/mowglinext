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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/get_next_strip.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// GetNextStrip — fetch next unmowed strip from map_server
// ---------------------------------------------------------------------------

class GetNextStrip : public BT::StatefulActionNode
{
public:
  GetNextStrip(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Client<mowgli_interfaces::srv::GetNextStrip>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// FollowStrip — follow a strip path with FTCController, blade ON
// ---------------------------------------------------------------------------

class FollowStrip : public BT::StatefulActionNode
{
public:
  using Nav2FollowPath = nav2_msgs::action::FollowPath;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Nav2FollowPath>;

  FollowStrip(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void setBladeEnabled(bool enabled);

  rclcpp_action::Client<Nav2FollowPath>::SharedPtr follow_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;
};

// ---------------------------------------------------------------------------
// TransitToStrip — navigate to strip start using Nav2 navigate_to_pose
// ---------------------------------------------------------------------------

class TransitToStrip : public BT::StatefulActionNode
{
public:
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  TransitToStrip(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
};

}  // namespace mowgli_behavior
