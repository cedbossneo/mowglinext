// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
/**
 * @file slam_heading_node.cpp
 * @brief Extracts full pose (x, y, yaw) from SLAM's map→odom TF for EKF fusion.
 *
 * slam_toolbox publishes map→odom TF via scan matching. This node looks up
 * map→base_footprint (which chains SLAM's map→odom with ekf_odom's
 * odom→base_footprint) and publishes the full robot pose with covariance
 * for ekf_map to fuse.
 *
 * When GPS is good, ekf_map trusts GPS more (lower covariance).
 * When GPS degrades, SLAM pose dominates (moderate covariance).
 * This provides both heading AND position from LiDAR scan matching.
 */

#include "mowgli_localization/slam_heading_node.hpp"

#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_localization
{

SlamHeadingNode::SlamHeadingNode(const rclcpp::NodeOptions& options) : Node("slam_heading", options)
{
  publish_rate_ = declare_parameter<double>("publish_rate", 5.0);
  yaw_variance_ = declare_parameter<double>("yaw_variance", 0.05);
  position_variance_ = declare_parameter<double>("position_variance", 0.1);
  stale_timeout_ = declare_parameter<double>("stale_timeout", 5.0);
  stale_variance_ = declare_parameter<double>("stale_variance", 1e6);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/slam/pose", rclcpp::QoS(10));

  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                             std::bind(&SlamHeadingNode::timer_callback, this));

  RCLCPP_INFO(get_logger(),
              "SlamHeadingNode started (map→base_footprint TF, rate=%.1f Hz, "
              "pos_var=%.3f, yaw_var=%.3f)",
              publish_rate_,
              position_variance_,
              yaw_variance_);
}

void SlamHeadingNode::timer_callback()
{
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    // Look up full robot pose in map frame (chains SLAM + ekf_odom)
    tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
  }
  catch (const tf2::TransformException&)
  {
    return;
  }

  const rclcpp::Time tf_stamp(tf.header.stamp);
  const double age = (now() - tf_stamp).seconds();
  const double yaw = tf2::getYaw(tf.transform.rotation);

  // Scale covariance based on TF freshness
  double pos_var = position_variance_;
  double yaw_var = yaw_variance_;
  if (age > stale_timeout_)
  {
    pos_var = stale_variance_;
    yaw_var = stale_variance_;
  }
  else if (age > stale_timeout_ * 0.5)
  {
    const double alpha = (age - stale_timeout_ * 0.5) / (stale_timeout_ * 0.5);
    pos_var = position_variance_ + alpha * (stale_variance_ - position_variance_);
    yaw_var = yaw_variance_ + alpha * (stale_variance_ - yaw_variance_);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  // Full pose: position + orientation
  msg.pose.pose.position.x = tf.transform.translation.x;
  msg.pose.pose.position.y = tf.transform.translation.y;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = tf.transform.rotation;

  // Covariance: position moderate, yaw tight, rest huge
  msg.pose.covariance[0] = pos_var;   // x
  msg.pose.covariance[7] = pos_var;   // y
  msg.pose.covariance[14] = 1e6;      // z
  msg.pose.covariance[21] = 1e6;      // roll
  msg.pose.covariance[28] = 1e6;      // pitch
  msg.pose.covariance[35] = yaw_var;  // yaw

  heading_pub_->publish(msg);
}

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::SlamHeadingNode>());
  rclcpp::shutdown();
  return 0;
}
