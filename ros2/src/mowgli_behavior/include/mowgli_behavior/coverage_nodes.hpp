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
#include <future>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/get_coverage_status.hpp"
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

  // Blade spinup delay — wait before sending path goal
  static constexpr double kBladeSpinupDelaySec = 1.5;
  std::chrono::steady_clock::time_point blade_start_time_;
  bool goal_sent_ = false;
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

// ---------------------------------------------------------------------------
// DetourAroundObstacle — when FollowStrip aborts on a lookahead-collision,
// drive a short side-step path through the global planner so the robot
// physically gets out from in front of the obstacle (a person standing in
// the strip). The next strip iteration replans from the new pose; the
// `mow_progress` layer prevents re-cutting already-mowed cells.
//
// The detour is a NavigateToPose at (current_pose ⊕ forward·x̂_body
// + lateral·ŷ_body), routed via SmacPlanner over the local costmap which
// has the obstacle layer enabled — so the planner naturally curves around
// the obstruction rather than charging through it.
// ---------------------------------------------------------------------------

class DetourAroundObstacle : public BT::StatefulActionNode
{
public:
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  DetourAroundObstacle(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("forward_m", 0.8,
                              "Forward offset from current pose, body frame"),
        BT::InputPort<double>("lateral_m", 0.6,
                              "Lateral offset (positive = left), body frame"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
};

// ---------------------------------------------------------------------------
// GetNextUnmowedArea — find next area with remaining strips
// ---------------------------------------------------------------------------

class GetNextUnmowedArea : public BT::StatefulActionNode
{
public:
  GetNextUnmowedArea(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<uint32_t>("max_areas", 20u, "Maximum number of areas to check"),
        BT::OutputPort<uint32_t>("area_index", "Index of the next unmowed area"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// Process a completed service response. Returns SUCCESS if an unmowed area
  /// was found, FAILURE if all areas are done / no areas defined, or RUNNING
  /// if more areas need to be checked (launches next async call internally).
  BT::NodeStatus processResponse();

  rclcpp::Client<mowgli_interfaces::srv::GetCoverageStatus>::SharedPtr client_;
  std::optional<rclcpp::Client<mowgli_interfaces::srv::GetCoverageStatus>::FutureAndRequestId>
      pending_future_;
  std::chrono::steady_clock::time_point call_start_;
  uint32_t current_area_idx_{0};
  uint32_t max_areas_{20};
  uint32_t areas_queried_{0};
  uint32_t areas_complete_{0};
};

}  // namespace mowgli_behavior
