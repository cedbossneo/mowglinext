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
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/emergency_stop.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// SetMowerEnabled
// ---------------------------------------------------------------------------

/// Calls the /hardware_bridge/mower_control service to enable or disable the
/// cutting blade motor.
///
/// Input ports:
///   enabled (bool) – true to start the blade, false to stop it.
class SetMowerEnabled : public BT::SyncActionNode
{
public:
  SetMowerEnabled(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("enabled", "Enable (true) or disable (false) the mow motor")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// WaitForDuration
// ---------------------------------------------------------------------------

/// Stateful action that returns RUNNING until the requested duration has
/// elapsed, then returns SUCCESS.
///
/// Input ports:
///   duration_sec (double, default "1.0") – wait duration in seconds.
class WaitForDuration : public BT::StatefulActionNode
{
public:
  WaitForDuration(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("duration_sec", 1.0, "Duration to wait in seconds")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::duration<double> duration_;
};

// ---------------------------------------------------------------------------
// SaveSlamMap
// ---------------------------------------------------------------------------

/// Calls /slam_toolbox/serialize_map to persist the current pose graph to
/// disk.  Intended to run after mowing completes so the map survives container
/// restarts.
///
/// Input ports:
///   map_path (string, default "/ros2_ws/maps/garden_map") – destination path
///            without extension.  slam_toolbox appends .posegraph / .data.
///
/// Returns SUCCESS when the service call succeeds, FAILURE otherwise.  The
/// node is synchronous: it blocks for up to 5 s waiting for the service
/// response before declaring failure.
class SaveSlamMap : public BT::StatefulActionNode
{
public:
  using SerializeSrv = slam_toolbox::srv::SerializePoseGraph;

  SaveSlamMap(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>("map_path",
                                   "/ros2_ws/maps/garden_map",
                                   "Destination path without extension for the pose graph files")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Client<SerializeSrv>::SharedPtr client_;
  std::shared_future<SerializeSrv::Response::SharedPtr> response_future_;
};

// ---------------------------------------------------------------------------
// SaveObstacles
// ---------------------------------------------------------------------------

/// Calls /obstacle_tracker/save_obstacles to persist the obstacle map to disk.
class SaveObstacles : public BT::SyncActionNode
{
public:
  SaveObstacles(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// ResetEmergency
// ---------------------------------------------------------------------------

/// Calls the /hardware_bridge/emergency_stop service with emergency=0 to
/// release a latched emergency state in the firmware.  Used to auto-clear
/// emergencies when the robot is placed back on the dock.
///
/// Returns SUCCESS when the release request is sent (fire-and-forget to the
/// firmware, which is the safety authority and may refuse the release if a
/// physical trigger is still asserted).
class ResetEmergency : public BT::SyncActionNode
{
public:
  ResetEmergency(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<mowgli_interfaces::srv::EmergencyStop>::SharedPtr client_;
};

}  // namespace mowgli_behavior
