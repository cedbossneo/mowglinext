// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
/**
 * @file slam_heading_node.cpp
 * @brief Extracts heading from SLAM's /pose topic for EKF fusion.
 *
 * slam_toolbox publishes PoseWithCovarianceStamped on /pose whenever a
 * scan is processed (independent of transform_publish_period). This node
 * extracts the yaw and republishes it for ekf_map to fuse as absolute
 * heading from LiDAR scan matching.
 *
 * Note: /pose only publishes when the robot has moved enough
 * (minimum_travel_distance). When stationary, no heading is published
 * and ekf_map relies on GPS velocity heading + IMU gyro.
 */

#include "mowgli_localization/slam_heading_node.hpp"

#include <cmath>

namespace mowgli_localization
{

SlamHeadingNode::SlamHeadingNode(const rclcpp::NodeOptions& options) : Node("slam_heading", options)
{
  yaw_variance_ = declare_parameter<double>("yaw_variance", 0.05);
  stale_timeout_ = declare_parameter<double>("stale_timeout", 5.0);
  stale_yaw_variance_ = declare_parameter<double>("stale_yaw_variance", 1e6);

  heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/slam/heading", rclcpp::QoS(10));

  // Subscribe to SLAM's /pose topic. Published by slam_toolbox whenever a
  // scan is processed, independent of transform_publish_period (works even
  // when set to 0.0). Only fires when robot moves minimum_travel_distance.
  slam_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose",
      rclcpp::QoS(10),
      std::bind(&SlamHeadingNode::on_slam_pose, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "SlamHeadingNode started (subscribing to /pose, yaw_var=%.3f)",
              yaw_variance_);
}

void SlamHeadingNode::on_slam_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  const rclcpp::Time msg_stamp(msg->header.stamp);
  const double age = (now() - msg_stamp).seconds();
  const auto& q = msg->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  double variance = yaw_variance_;
  if (age > stale_timeout_)
  {
    variance = stale_yaw_variance_;
  }
  else if (age > stale_timeout_ * 0.5)
  {
    const double alpha = (age - stale_timeout_ * 0.5) / (stale_timeout_ * 0.5);
    variance = yaw_variance_ + alpha * (stale_yaw_variance_ - yaw_variance_);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp = now();
  out.header.frame_id = "map";

  out.pose.pose.orientation.w = std::cos(yaw / 2.0);
  out.pose.pose.orientation.z = std::sin(yaw / 2.0);

  out.pose.covariance[0] = 1e6;
  out.pose.covariance[7] = 1e6;
  out.pose.covariance[14] = 1e6;
  out.pose.covariance[21] = 1e6;
  out.pose.covariance[28] = 1e6;
  out.pose.covariance[35] = variance;

  heading_pub_->publish(out);
}

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::SlamHeadingNode>());
  rclcpp::shutdown();
  return 0;
}
