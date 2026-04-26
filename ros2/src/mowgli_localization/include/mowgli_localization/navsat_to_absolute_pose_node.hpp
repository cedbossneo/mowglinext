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
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
  void create_services();

  void on_navsat_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void on_set_datum(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

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

  // Latest GPS fix for the set_datum service.
  sensor_msgs::msg::NavSatFix last_fix_;
  bool has_fix_{false};

  // ROS handles
  rclcpp::Publisher<mowgli_interfaces::msg::AbsolutePose>::SharedPtr pose_pub_;
  /// Standard-msg twin of the AbsolutePose topic. robot_localization's EKF
  /// pose0 input expects PoseWithCovarianceStamped; AbsolutePose is a
  /// Mowgli-specific type and not subscribable by the EKF.
  ///
  /// UNLIKE pose_pub_ this publishes BASE FRAME (base_footprint) position —
  /// not antenna position. Lever arm is subtracted using the latest
  /// odom→base_footprint TF for the yaw at GPS-fix time. Required so the
  /// EKF tracks the robot body and not the 30-cm antenna circle traced
  /// during pure rotation.
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_datum_srv_;

  /// TF listener to resolve base_footprint↔gps_link (static from URDF,
  /// gives the lever arm) and odom↔base_footprint (dynamic from ekf_odom,
  /// gives current yaw). First-successful-lookup latches the lever arm;
  /// yaw is looked up fresh each fix. We intentionally use the ODOM-frame
  /// yaw (wheels+gyro) rather than the map-frame yaw because ekf_map
  /// consumes this pose_cov topic and using its yaw would feed back into
  /// the lever arm correction, amplifying rotational noise.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool lever_arm_known_{false};
  double lever_arm_x_{0.0};
  double lever_arm_y_{0.0};
};

}  // namespace mowgli_localization
