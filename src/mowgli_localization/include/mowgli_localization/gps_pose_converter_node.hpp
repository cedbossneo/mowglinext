// SPDX-License-Identifier: GPL-3.0
/**
 * @file gps_pose_converter_node.hpp
 * @brief GPS pose converter — always publishes, covariance reflects quality.
 *
 * Converts mowgli_interfaces/msg/AbsolutePose into
 * geometry_msgs/msg/PoseWithCovarianceStamped for robot_localization's EKF.
 *
 * Every fix is published; the covariance is scaled by fix quality so the EKF
 * naturally blends GPS with SLAM:
 *   RTK fixed  → tight covariance → GPS dominates
 *   RTK float  → moderate         → GPS + SLAM blend
 *   No RTK     → large            → SLAM dominates
 */

#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_localization
{

class GpsPoseConverterNode : public rclcpp::Node
{
public:
  explicit GpsPoseConverterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GpsPoseConverterNode() override = default;

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
  void on_absolute_pose(mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg);

  // ---------------------------------------------------------------------------
  // Internal helpers
  // ---------------------------------------------------------------------------

  /**
   * @brief Compute xy variance from fix quality and reported accuracy.
   *
   * Always returns a positive value — every fix is published.
   * The multiplier scales with fix degradation so the EKF weights
   * GPS less when quality is poor, letting SLAM take over.
   */
  double compute_xy_variance(const mowgli_interfaces::msg::AbsolutePose& msg) const;

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  // Reserved: datum parameters removed — not yet implemented.
  double min_accuracy_threshold_{0.5};

  // ---------------------------------------------------------------------------
  // ROS handles
  // ---------------------------------------------------------------------------
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr abs_pose_sub_;
};

}  // namespace mowgli_localization
