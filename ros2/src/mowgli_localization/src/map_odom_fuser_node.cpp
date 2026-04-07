// Copyright 2026 Mowgli Project
//
// Licensed under the GNU General Public License, version 3 (the "License");
/**
 * @file map_odom_fuser_node.cpp
 * @brief Corrects SLAM's map->odom heading via smoothed offset from ekf_map.
 *
 * How it works:
 *   1. Read SLAM's map->odom TF (position + heading from scan matching)
 *   2. Read ekf_map's heading from /odometry/filtered_map
 *   3. Compute heading offset = ekf_yaw - slam_yaw (smoothed with EMA)
 *   4. Publish corrected TF: SLAM position + (SLAM heading + offset)
 *
 * The offset absorbs slow drift and sudden jumps (like the undock flip)
 * while preserving SLAM's real-time rotation tracking.  The EMA smoothing
 * (alpha=0.01) means the offset adapts over ~2-3 seconds.
 */

#include "mowgli_localization/map_odom_fuser_node.hpp"

#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_localization
{

MapOdomFuserNode::MapOdomFuserNode(const rclcpp::NodeOptions &options)
    : Node("map_odom_fuser", options)
{
  publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
  offset_alpha_ = declare_parameter<double>("offset_alpha", 0.01);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  ekf_map_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered_map",
      rclcpp::QoS(10),
      std::bind(&MapOdomFuserNode::ekf_map_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                             std::bind(&MapOdomFuserNode::timer_callback, this));

  RCLCPP_INFO(get_logger(),
              "MapOdomFuserNode started (rate=%.0f Hz, offset_alpha=%.3f)",
              publish_rate_,
              offset_alpha_);
}

double MapOdomFuserNode::normalize_angle(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

void MapOdomFuserNode::ekf_map_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  ekf_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
  has_ekf_ = true;

  // Update heading offset when we have both sources
  if (has_slam_)
  {
    const double raw_offset = normalize_angle(ekf_yaw_ - slam_yaw_);

    if (!offset_initialized_)
    {
      yaw_offset_ = raw_offset;
      offset_initialized_ = true;
    }
    else
    {
      // EMA smoothing on the unit circle (avoids wraparound issues)
      const double sin_err = std::sin(raw_offset) - std::sin(yaw_offset_);
      const double cos_err = std::cos(raw_offset) - std::cos(yaw_offset_);
      const double new_sin = std::sin(yaw_offset_) + offset_alpha_ * sin_err;
      const double new_cos = std::cos(yaw_offset_) + offset_alpha_ * cos_err;
      yaw_offset_ = std::atan2(new_sin, new_cos);
    }
  }
}

void MapOdomFuserNode::timer_callback()
{
  // Read SLAM's map->odom TF
  try
  {
    auto tf = tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);

    const rclcpp::Time tf_stamp(tf.header.stamp);
    if (!has_slam_ || tf_stamp != last_slam_stamp_)
    {
      slam_x_ = tf.transform.translation.x;
      slam_y_ = tf.transform.translation.y;
      slam_yaw_ = tf2::getYaw(tf.transform.rotation);
      last_slam_stamp_ = tf_stamp;
      has_slam_ = true;
    }
  }
  catch (const tf2::TransformException &)
  {
    if (!has_slam_)
    {
      return;
    }
  }

  // Compute corrected heading: SLAM heading + smoothed offset
  const double corrected_yaw = normalize_angle(slam_yaw_ + yaw_offset_);

  // Publish corrected map->odom TF
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now();
  t.header.frame_id = map_frame_;
  t.child_frame_id = odom_frame_;

  t.transform.translation.x = slam_x_;
  t.transform.translation.y = slam_y_;
  t.transform.translation.z = 0.0;

  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = std::sin(corrected_yaw / 2.0);
  t.transform.rotation.w = std::cos(corrected_yaw / 2.0);

  tf_broadcaster_->sendTransform(t);
}

}  // namespace mowgli_localization

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::MapOdomFuserNode>());
  rclcpp::shutdown();
  return 0;
}
