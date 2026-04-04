// Copyright 2026 Mowgli Project
//
// Licensed under the GNU General Public License, version 3 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.html
/**
 * @file slam_heading_node.hpp
 * @brief Extracts heading from SLAM's map→odom TF and publishes it as a
 *        PoseWithCovarianceStamped for EKF fusion.
 *
 * slam_toolbox maintains a map→odom transform via scan matching. This node
 * periodically looks up that transform and publishes the yaw component as
 * a heading measurement. The EKF can fuse this to get absolute heading
 * from LiDAR map matching — works regardless of magnetometer, GPS, or
 * robot motion.
 *
 * The yaw covariance reflects SLAM confidence: tighter when SLAM is
 * actively matching scans, looser when it hasn't updated recently.
 */

#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mowgli_localization
{

class SlamHeadingNode : public rclcpp::Node
{
public:
  explicit SlamHeadingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SlamHeadingNode() override = default;

private:
  void timer_callback();

  // Parameters
  double publish_rate_{5.0};
  double yaw_variance_{0.05};  // rad² — SLAM heading is usually quite good
  double stale_timeout_{5.0};  // seconds before considering SLAM stale
  double stale_yaw_variance_{1e6};  // variance when SLAM is stale

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  rclcpp::Time last_valid_tf_time_;
  bool has_valid_tf_{false};

  // ROS handles
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr heading_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mowgli_localization
