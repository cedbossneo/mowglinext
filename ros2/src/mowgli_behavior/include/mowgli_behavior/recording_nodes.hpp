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
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/add_mowing_area.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class RecordAreaAlgorithmTest;

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// RecordArea — record robot trajectory and save as mowing area polygon
// ---------------------------------------------------------------------------

/// Records the robot's position at ~1 Hz while the user drives along the
/// boundary. On finish (COMMAND_RECORD_FINISH=5), simplifies the trajectory
/// using Douglas-Peucker algorithm and saves it as a mowing area via the
/// map_server_node/add_area service.
///
/// On cancel (COMMAND_RECORD_CANCEL=6), discards the recording.
///
/// Returns RUNNING while recording, SUCCESS on finish, FAILURE on cancel or error.
///
/// Input ports:
///   simplification_tolerance (double, default "0.2") — Douglas-Peucker tolerance in metres.
///   min_vertices (uint32_t, default "3") — minimum polygon vertices after simplification.
///   min_area (double, default "1.0") — minimum polygon area in square metres.
///   record_rate_hz (double, default "1.0") — position recording frequency.
class RecordArea : public BT::StatefulActionNode
{
  friend class ::RecordAreaAlgorithmTest;  // test access to private static methods

public:
  RecordArea(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("simplification_tolerance", 0.2, "Douglas-Peucker tolerance (m)"),
        BT::InputPort<uint32_t>("min_vertices", 3u, "Minimum polygon vertices"),
        BT::InputPort<double>("min_area", 1.0, "Minimum polygon area (m^2)"),
        BT::InputPort<double>("record_rate_hz", 1.0, "Recording frequency (Hz)"),
        BT::InputPort<bool>("is_exclusion_zone", false, "Record as exclusion zone"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// Record current robot position into trajectory_.
  void record_position();

  /// Apply Douglas-Peucker simplification to the trajectory.
  static std::vector<geometry_msgs::msg::Point32> douglas_peucker(
      const std::vector<geometry_msgs::msg::Point32>& points, double tolerance);

  /// Recursive Douglas-Peucker helper.
  static void dp_recursive(const std::vector<geometry_msgs::msg::Point32>& points,
                           double tolerance,
                           size_t start,
                           size_t end,
                           std::vector<bool>& keep);

  /// Perpendicular distance from a point to a line segment.
  static double perpendicular_distance(const geometry_msgs::msg::Point32& pt,
                                       const geometry_msgs::msg::Point32& line_start,
                                       const geometry_msgs::msg::Point32& line_end);

  /// Compute polygon area using the shoelace formula.
  static double polygon_area(const std::vector<geometry_msgs::msg::Point32>& points);

  /// Save the simplified polygon as a mowing area.
  bool save_area(const std::vector<geometry_msgs::msg::Point32>& points, bool is_exclusion_zone);

  /// Recorded trajectory points in map frame.
  std::vector<geometry_msgs::msg::Point32> trajectory_;

  /// Timestamp of last recorded position.
  std::chrono::steady_clock::time_point last_record_time_;

  /// Recording interval (computed from record_rate_hz).
  std::chrono::milliseconds record_interval_{1000};

  /// Publisher for live trajectory preview.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

  /// Service client for adding the area.
  rclcpp::Client<mowgli_interfaces::srv::AddMowingArea>::SharedPtr add_area_client_;

  /// Area counter for auto-naming.
  static int area_counter_;
};

}  // namespace mowgli_behavior
