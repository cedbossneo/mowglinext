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
 * @file navsat_to_pose_node.cpp
 * @brief Converts Gazebo NavSatFix → PoseWithCovarianceStamped for the EKF.
 *
 * Uses the first received fix (or configured datum) as the local ENU origin.
 * Subsequent fixes are projected to local x/y metres using equirectangular
 * approximation — accurate enough for a garden-scale simulation.
 */

#include "mowgli_simulation/navsat_to_pose_node.hpp"

#include <cmath>

namespace mowgli_simulation
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kEarthRadiusM = 6371000.0;
}  // namespace

NavSatToPoseNode::NavSatToPoseNode(const rclcpp::NodeOptions& options)
    : Node("navsat_to_pose", options)
{
  datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
  datum_lon_ = declare_parameter<double>("datum_lon", 0.0);
  xy_covariance_ = declare_parameter<double>("xy_covariance", 0.001);

  // If both datum values are non-zero, use them immediately.
  if (std::abs(datum_lat_) > 1e-9 || std::abs(datum_lon_) > 1e-9)
  {
    datum_set_ = true;
    RCLCPP_INFO(get_logger(),
                "NavSatToPoseNode using configured datum: lat=%.8f lon=%.8f",
                datum_lat_,
                datum_lon_);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "NavSatToPoseNode will use first fix as datum");
  }

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/mowgli/gps/pose",
                                                                              rclcpp::QoS(10));

  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
      {
        on_fix(msg);
      });

  RCLCPP_INFO(get_logger(), "NavSatToPoseNode started — xy_covariance=%.4f", xy_covariance_);
}

void NavSatToPoseNode::on_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  // Discard invalid fixes.
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
  {
    return;
  }

  // Set datum from first valid fix if not configured.
  if (!datum_set_)
  {
    datum_lat_ = msg->latitude;
    datum_lon_ = msg->longitude;
    datum_set_ = true;
    RCLCPP_INFO(get_logger(),
                "Datum set from first fix: lat=%.8f lon=%.8f",
                datum_lat_,
                datum_lon_);
  }

  // Equirectangular projection to local ENU (x=east, y=north).
  const double dlat = (msg->latitude - datum_lat_) * kDegToRad;
  const double dlon = (msg->longitude - datum_lon_) * kDegToRad;
  const double cos_lat = std::cos(datum_lat_ * kDegToRad);

  const double x = dlon * cos_lat * kEarthRadiusM;  // east
  const double y = dlat * kEarthRadiusM;  // north

  // Build PoseWithCovarianceStamped in map frame.
  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp = now();
  out.header.frame_id = "map";

  out.pose.pose.position.x = x;
  out.pose.pose.position.y = y;
  out.pose.pose.position.z = 0.0;

  // Identity orientation (GPS doesn't provide heading).
  out.pose.pose.orientation.w = 1.0;

  // Covariance: low for x/y (simulated RTK-fixed), large for everything else.
  out.pose.covariance[0] = xy_covariance_;  // x
  out.pose.covariance[7] = xy_covariance_;  // y
  out.pose.covariance[14] = 1e6;  // z
  out.pose.covariance[21] = 1e6;  // roll
  out.pose.covariance[28] = 1e6;  // pitch
  out.pose.covariance[35] = 1e6;  // yaw

  pose_pub_->publish(out);
}

}  // namespace mowgli_simulation

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_simulation::NavSatToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
