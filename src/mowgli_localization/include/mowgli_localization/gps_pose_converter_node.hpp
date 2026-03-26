// SPDX-License-Identifier: GPL-3.0
/**
 * @file gps_pose_converter_node.hpp
 * @brief GPS pose converter node.
 *
 * Converts mowgli_interfaces/msg/AbsolutePose (produced by the xbot_positioning
 * subsystem or its ROS2 equivalent) into a
 * geometry_msgs/msg/PoseWithCovarianceStamped that robot_localization's EKF
 * can consume directly on /gps/pose.
 *
 * The AbsolutePose message already carries a geometry_msgs/PoseWithCovariance
 * in a local flat-earth frame, so no geodetic projection is required here.
 * This node's job is to:
 *   1. Gate on position quality (flags + position_accuracy).
 *   2. Scale the output covariance according to fix type so the EKF trusts
 *      RTK-fixed positions more than float, and rejects degraded fixes.
 *   3. Stamp the message with current ROS time and forward it.
 *
 * Subscribed topics:
 *   /gps/absolute_pose   mowgli_interfaces/msg/AbsolutePose
 *
 * Published topics:
 *   /gps/pose            geometry_msgs/msg/PoseWithCovarianceStamped
 *
 * Parameters:
 *   use_first_fix_as_datum (bool,   default true)  – reserved for future
 *     geodetic datum logic; currently the AbsolutePose is already projected.
 *   datum_lat              (double, default 0.0)   – geodetic datum latitude.
 *   datum_lon              (double, default 0.0)   – geodetic datum longitude.
 *   min_accuracy_threshold (double, default 0.5 m) – positions with
 *     position_accuracy worse than this value are discarded.
 */

#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mowgli_interfaces/msg/absolute_pose.hpp"

namespace mowgli_localization
{

class GpsPoseConverterNode : public rclcpp::Node
{
public:
  explicit GpsPoseConverterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
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
  void on_absolute_pose(const mowgli_interfaces::msg::AbsolutePose::SharedPtr msg);

  // ---------------------------------------------------------------------------
  // Internal helpers
  // ---------------------------------------------------------------------------

  /**
   * @brief Compute the xy position variance to use in the covariance matrix.
   *
   * The variance is derived from position_accuracy (1-sigma in metres) and
   * a multiplier that reflects the current fix quality:
   *   RTK fixed   → multiplier 1.0  (trust the sensor accuracy fully)
   *   RTK float   → multiplier 4.0  (twice the standard deviation)
   *   Dead reckon → multiplier 16.0 (very loose)
   *   No fix      → discard message (returns negative sentinel)
   *
   * @param msg  The incoming AbsolutePose message.
   * @return     Variance in m² to place on the diagonal, or -1.0 if the fix
   *             quality is too poor to publish.
   */
  double compute_xy_variance(const mowgli_interfaces::msg::AbsolutePose & msg) const;

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  bool   use_first_fix_as_datum_{true};
  double datum_lat_{0.0};
  double datum_lon_{0.0};
  double min_accuracy_threshold_{0.5};

  // ---------------------------------------------------------------------------
  // ROS handles
  // ---------------------------------------------------------------------------
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr abs_pose_sub_;
};

}  // namespace mowgli_localization
