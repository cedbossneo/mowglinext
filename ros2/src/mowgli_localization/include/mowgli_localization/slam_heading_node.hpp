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
 * @file slam_heading_node.hpp
 * @brief Extracts heading from SLAM's pose topic for EKF fusion.
 *
 * slam_toolbox publishes its scan-matched pose on /slam_toolbox/pose.
 * This node subscribes and republishes the yaw component as a heading
 * measurement for ekf_map. This avoids a circular dependency since
 * ekf_map now publishes map→odom TF (GPS-anchored).
 */

#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_localization
{

class SlamHeadingNode : public rclcpp::Node
{
public:
  explicit SlamHeadingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SlamHeadingNode() override = default;

private:
  void on_slam_pose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  // Parameters
  double yaw_variance_{0.05};
  double stale_timeout_{5.0};
  double stale_yaw_variance_{1e6};

  // ROS handles
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr heading_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub_;
};

}  // namespace mowgli_localization
