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
 * @file wheel_odometry_node.hpp
 * @brief Differential-drive wheel odometry node.
 *
 * Subscribes to raw wheel tick counts from the STM32 firmware and integrates
 * them into a nav_msgs/msg/Odometry message suitable for consumption by the
 * FusionCore UKF.
 *
 * Subscribed topics:
 *   /wheel_ticks   mowgli_interfaces/msg/WheelTick
 *
 * Published topics:
 *   /wheel_odom    nav_msgs/msg/Odometry
 *
 * Optionally published TF:
 *   odom → base_footprint  (disabled by default; FusionCore owns this transform)
 *
 * Parameters:
 *   wheel_distance   (double, default 0.35 m)   – lateral distance between
 *                    the two driven wheels (track width).
 *   ticks_per_meter  (double, default 300.0)   – encoder ticks per metre of
 *                    wheel travel.
 *   publish_tf       (bool,   default false)     – when true the node also
 *                    broadcasts the odom → base_footprint TF.
 */

#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mowgli_interfaces/msg/wheel_tick.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace mowgli_localization
{

class WheelOdometryNode : public rclcpp::Node
{
public:
  explicit WheelOdometryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~WheelOdometryNode() override = default;

private:
  // ---------------------------------------------------------------------------
  // Initialisation helpers
  // ---------------------------------------------------------------------------
  void declare_parameters();
  void create_publishers();
  void create_subscribers();

  // ---------------------------------------------------------------------------
  // Callback
  // ---------------------------------------------------------------------------
  void on_wheel_tick(mowgli_interfaces::msg::WheelTick::ConstSharedPtr msg);

  // ---------------------------------------------------------------------------
  // Internal helpers
  // ---------------------------------------------------------------------------

  /**
   * @brief Convert a yaw angle to a geometry_msgs quaternion (z-rotation only).
   * @param yaw Angle in radians.
   */
  static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  double wheel_distance_{0.35};
  double ticks_per_meter_{300.0};
  bool publish_tf_{false};

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  bool first_tick_{true};

  /// Cumulative tick counts from the previous message (RL = left, RR = right).
  uint32_t prev_ticks_left_{0};
  uint32_t prev_ticks_right_{0};

  /// Wall-clock time of the previous tick message; used to compute the
  /// real dt for the published twist instead of a hardcoded nominal.
  rclcpp::Time last_tick_time_;

  /// Accumulated pose in the odom frame.
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};  ///< Heading in radians, positive = counter-clockwise.

  // ---------------------------------------------------------------------------
  // ROS handles
  // ---------------------------------------------------------------------------
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<mowgli_interfaces::msg::WheelTick>::SharedPtr wheel_tick_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace mowgli_localization
