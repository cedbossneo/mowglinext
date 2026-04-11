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
 * @file slam_heading_node.cpp
 * @brief Extracts heading from SLAM's pose for EKF fusion.
 *
 * slam_toolbox computes a pose via LiDAR scan matching and publishes
 * it on /slam_toolbox/pose. This node subscribes to that topic,
 * extracts the yaw component, and republishes it as a
 * PoseWithCovarianceStamped on /slam/heading for the EKF to fuse.
 *
 * This avoids a circular dependency: ekf_map publishes map→odom TF
 * (GPS-anchored), so we cannot read map→odom to get SLAM heading.
 * Instead we read SLAM's internal pose directly.
 *
 * This provides an absolute heading reference from LiDAR features
 * (walls, trees, fences) that works:
 *   - Without a magnetometer
 *   - While stationary (unlike GPS velocity heading)
 *   - Under canopy (unlike GPS)
 */

#include "mowgli_localization/slam_heading_node.hpp"

#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_localization
{

SlamHeadingNode::SlamHeadingNode(const rclcpp::NodeOptions &options) : Node("slam_heading", options)
{
  yaw_variance_ = declare_parameter<double>("yaw_variance", 0.05);
  stale_timeout_ = declare_parameter<double>("stale_timeout", 5.0);
  stale_yaw_variance_ = declare_parameter<double>("stale_yaw_variance", 1e6);

  heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/heading",
                                                                                 rclcpp::QoS(10));

  // Subscribe to SLAM's internal pose (avoids TF circular dependency)
  slam_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/slam_toolbox/pose",
      rclcpp::QoS(10),
      std::bind(&SlamHeadingNode::on_slam_pose, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "SlamHeadingNode started (subscribing to /slam_toolbox/pose, yaw_var=%.3f)",
              yaw_variance_);
}

void SlamHeadingNode::on_slam_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  const rclcpp::Time msg_stamp(msg->header.stamp);
  const double age = (now() - msg_stamp).seconds();

  // Extract yaw from SLAM's pose
  const double yaw = tf2::getYaw(msg->pose.pose.orientation);

  // Determine yaw variance based on message freshness
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

  // Publish heading as PoseWithCovarianceStamped
  // Position is zeroed with huge covariance — only yaw matters.
  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp = now();
  out.header.frame_id = "map";

  out.pose.pose.position.x = 0.0;
  out.pose.pose.position.y = 0.0;
  out.pose.pose.position.z = 0.0;

  out.pose.pose.orientation.w = std::cos(yaw / 2.0);
  out.pose.pose.orientation.z = std::sin(yaw / 2.0);
  out.pose.pose.orientation.x = 0.0;
  out.pose.pose.orientation.y = 0.0;

  out.pose.covariance[0] = 1e6;  // x
  out.pose.covariance[7] = 1e6;  // y
  out.pose.covariance[14] = 1e6;  // z
  out.pose.covariance[21] = 1e6;  // roll
  out.pose.covariance[28] = 1e6;  // pitch
  out.pose.covariance[35] = variance;  // yaw

  heading_pub_->publish(out);
}

}  // namespace mowgli_localization

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::SlamHeadingNode>());
  rclcpp::shutdown();
  return 0;
}
