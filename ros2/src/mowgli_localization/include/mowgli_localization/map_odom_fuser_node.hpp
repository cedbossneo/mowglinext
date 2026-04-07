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
/**
 * @file map_odom_fuser_node.hpp
 * @brief Corrects SLAM's map->odom heading using ekf_map as absolute reference.
 *
 * Instead of replacing SLAM's heading entirely (which kills rotation tracking),
 * this node computes a smoothed heading OFFSET between ekf_map and SLAM, then
 * applies that offset to SLAM's heading.  This preserves SLAM's real-time
 * rotation tracking while correcting absolute heading drift.
 *
 * When robot rotates: SLAM yaw changes, ekf_map yaw changes similarly,
 *   offset stays constant -> corrected heading tracks rotation correctly.
 * When SLAM drifts (undock): SLAM yaw jumps, ekf_map stays stable,
 *   offset absorbs the jump -> corrected heading stays correct.
 */

#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace mowgli_localization
{

class MapOdomFuserNode : public rclcpp::Node
{
public:
  explicit MapOdomFuserNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MapOdomFuserNode() override = default;

private:
  void timer_callback();
  void ekf_map_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// Normalize angle to [-pi, pi]
  static double normalize_angle(double a);

  // Parameters
  double publish_rate_{50.0};
  double offset_alpha_{0.01};  // EMA smoothing for heading offset
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // State: SLAM transform (from TF)
  double slam_x_{0.0};
  double slam_y_{0.0};
  double slam_yaw_{0.0};
  rclcpp::Time last_slam_stamp_;
  bool has_slam_{false};

  // State: ekf_map heading
  double ekf_yaw_{0.0};
  bool has_ekf_{false};

  // State: smoothed heading offset (ekf_yaw - slam_yaw)
  double yaw_offset_{0.0};
  bool offset_initialized_{false};

  // ROS handles
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mowgli_localization
