// SPDX-License-Identifier: GPL-3.0
/**
 * @file navsat_to_absolute_pose_node.hpp
 * @brief Converts sensor_msgs/NavSatFix → mowgli_interfaces/AbsolutePose.
 *
 * Bridges the standard ROS2 ublox_gps driver output into the Mowgli
 * AbsolutePose message that gps_pose_converter_node expects.
 *
 * Performs WGS84 → local ENU projection using a configurable datum origin.
 * RTK fix quality is mapped from NavSatFix::status to AbsolutePose flags.
 *
 * Subscribed topics:
 *   /gps/fix   sensor_msgs/msg/NavSatFix
 *
 * Published topics:
 *   /gps/absolute_pose    mowgli_interfaces/msg/AbsolutePose
 */

#pragma once

#include <cmath>

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace mowgli_localization
{

class NavSatToAbsolutePoseNode : public rclcpp::Node
{
public:
  explicit NavSatToAbsolutePoseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~NavSatToAbsolutePoseNode() override = default;

private:
  void declare_parameters();
  void create_publishers();
  void create_subscribers();

  void on_navsat_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

  /**
   * @brief Project WGS84 lat/lon to local ENU (x=east, y=north) relative to datum.
   *
   * Uses an equirectangular approximation which is accurate to ~1 cm within
   * 10 km of the datum origin — more than sufficient for a garden mower.
   */
  void wgs84_to_enu(double lat, double lon, double& east, double& north) const;

  // Parameters
  double datum_lat_{0.0};
  double datum_lon_{0.0};
  double cos_datum_lat_{1.0};  ///< Precomputed cos(datum_lat) for projection

  // ROS handles
  rclcpp::Publisher<mowgli_interfaces::msg::AbsolutePose>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
};

}  // namespace mowgli_localization
