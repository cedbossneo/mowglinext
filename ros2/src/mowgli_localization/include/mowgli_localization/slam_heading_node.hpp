// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
/**
 * @file slam_heading_node.hpp
 * @brief Extracts full pose from SLAM's TF for EKF fusion.
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

  double publish_rate_{5.0};
  double yaw_variance_{0.05};
  double position_variance_{0.1};
  double stale_timeout_{5.0};
  double stale_variance_{1e6};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr heading_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mowgli_localization
