// Copyright 2026 Mowgli Project
//
// Licensed under the GNU General Public License, version 3 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.html
/**
 * @file slam_heading_node.cpp
 * @brief Extracts heading from SLAM's map→odom TF for EKF fusion.
 *
 * slam_toolbox computes a map→odom transform via LiDAR scan matching.
 * This node periodically looks up that transform, extracts the yaw
 * component, and publishes it as a PoseWithCovarianceStamped on
 * /slam/heading for the EKF to fuse.
 *
 * This provides an absolute heading reference from LiDAR features
 * (walls, trees, fences) that works:
 *   - Without a magnetometer
 *   - While stationary (unlike GPS velocity heading)
 *   - Indoors or under canopy (unlike GPS)
 *
 * The position in the published message is set to (0,0,0) with huge
 * covariance — only the yaw is meaningful. The EKF fuses yaw only
 * via the pose_config setting.
 */

#include "mowgli_localization/slam_heading_node.hpp"

#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_localization
{

SlamHeadingNode::SlamHeadingNode(const rclcpp::NodeOptions & options)
: Node("slam_heading", options)
{
  publish_rate_ = declare_parameter<double>("publish_rate", 5.0);
  yaw_variance_ = declare_parameter<double>("yaw_variance", 0.05);
  stale_timeout_ = declare_parameter<double>("stale_timeout", 5.0);
  stale_yaw_variance_ = declare_parameter<double>("stale_yaw_variance", 1e6);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/slam/heading", rclcpp::QoS(10));

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_),
    std::bind(&SlamHeadingNode::timer_callback, this));

  RCLCPP_INFO(get_logger(),
    "SlamHeadingNode started (rate=%.1f Hz, yaw_var=%.3f)",
    publish_rate_, yaw_variance_);
}

void SlamHeadingNode::timer_callback()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    // Look up SLAM's map→odom transform.
    // SLAM is the authority for this transform (ekf_map has publish_tf=false).
    tf = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    // SLAM TF not available — don't publish anything.
    // The EKF will rely on GPS heading and IMU.
    return;
  }

  const rclcpp::Time tf_stamp(tf.header.stamp);
  const double age = (now() - tf_stamp).seconds();

  // Extract yaw from the map→odom quaternion
  const double yaw = tf2::getYaw(tf.transform.rotation);

  // Determine yaw variance based on TF freshness
  double variance = yaw_variance_;
  if (age > stale_timeout_) {
    // SLAM hasn't updated recently — increase variance so other sources dominate
    variance = stale_yaw_variance_;
  } else if (age > stale_timeout_ * 0.5) {
    // Ramp up variance as TF ages
    const double alpha = (age - stale_timeout_ * 0.5) / (stale_timeout_ * 0.5);
    variance = yaw_variance_ + alpha * (stale_yaw_variance_ - yaw_variance_);
  }

  // Publish heading as PoseWithCovarianceStamped
  // Position is zeroed with huge covariance — only yaw matters.
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  // Position: don't care (huge covariance)
  msg.pose.pose.position.x = 0.0;
  msg.pose.pose.position.y = 0.0;
  msg.pose.pose.position.z = 0.0;

  // Orientation: SLAM-derived yaw
  msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
  msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;

  // Covariance: huge for position, meaningful for yaw
  msg.pose.covariance[0] = 1e6;        // x
  msg.pose.covariance[7] = 1e6;        // y
  msg.pose.covariance[14] = 1e6;       // z
  msg.pose.covariance[21] = 1e6;       // roll
  msg.pose.covariance[28] = 1e6;       // pitch
  msg.pose.covariance[35] = variance;  // yaw

  heading_pub_->publish(msg);
}

}  // namespace mowgli_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::SlamHeadingNode>());
  rclcpp::shutdown();
  return 0;
}
