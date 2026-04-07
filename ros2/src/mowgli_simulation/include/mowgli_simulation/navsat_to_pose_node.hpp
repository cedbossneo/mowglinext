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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file navsat_to_pose_node.hpp
 * @brief Converts NavSatFix (from Gazebo) to PoseWithCovarianceStamped for the EKF.
 *
 * Simulation-only node.  On the real robot the firmware publishes AbsolutePose
 * which is handled by gps_pose_converter_node.  In simulation, Gazebo publishes
 * sensor_msgs/NavSatFix on /gps/fix — this node converts it to
 * geometry_msgs/PoseWithCovarianceStamped on /gps/pose using the first fix as
 * the local ENU datum.
 *
 * Subscribed topics:
 *   /gps/fix   sensor_msgs/msg/NavSatFix
 *
 * Published topics:
 *   /gps/pose  geometry_msgs/msg/PoseWithCovarianceStamped
 *
 * Parameters:
 *   datum_lat  (double, 0.0)  – override datum latitude  (0 = use first fix)
 *   datum_lon  (double, 0.0)  – override datum longitude (0 = use first fix)
 *   xy_covariance (double, 0.001) – fixed covariance for RTK-fixed sim GPS
 */

#pragma once

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace mowgli_simulation
{

class NavSatToPoseNode : public rclcpp::Node
{
public:
  explicit NavSatToPoseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~NavSatToPoseNode() override = default;

private:
  void on_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

  // Parameters
  double datum_lat_{0.0};
  double datum_lon_{0.0};
  double xy_covariance_{0.001};

  bool datum_set_{false};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

}  // namespace mowgli_simulation
