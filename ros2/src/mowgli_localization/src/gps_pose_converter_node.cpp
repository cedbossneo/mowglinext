// Copyright 2026 Mowgli Project
//
// Licensed under the GNU General Public License, version 3 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.html
/**
 * @file gps_pose_converter_node.cpp
 * @brief GPS pose converter — publishes position + heading for EKF fusion.
 *
 * Position: every GPS fix is forwarded to the EKF with covariance that
 * reflects the actual fix quality (RTK fixed → tight, float → moderate,
 * autonomous → large).
 *
 * Heading: derived from consecutive GPS positions. When the robot moves
 * faster than heading_min_speed (default 0.15 m/s), the velocity vector
 * gives a reliable heading. The yaw covariance is set inversely
 * proportional to speed squared — tight when moving, very loose when
 * stationary. This allows the EKF to align the map frame with geographic
 * ENU when the robot is moving, while the IMU dominates heading when
 * stationary.
 *
 * Covariance layout for robot_localization (6×6 row-major):
 *   index [0]  → x variance
 *   index [7]  → y variance
 *   index [35] → yaw variance
 */

#include "mowgli_localization/gps_pose_converter_node.hpp"

#include <cmath>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_localization
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

GpsPoseConverterNode::GpsPoseConverterNode(const rclcpp::NodeOptions& options)
    : Node("gps_pose_converter", options)
{
  declare_parameters();
  create_publishers();
  create_subscribers();

  RCLCPP_INFO(get_logger(),
              "GpsPoseConverterNode started (heading_min_speed=%.2f m/s)",
              heading_min_speed_);
}

// ---------------------------------------------------------------------------
// Initialisation helpers
// ---------------------------------------------------------------------------

void GpsPoseConverterNode::declare_parameters()
{
  min_accuracy_threshold_ = declare_parameter<double>("min_accuracy_threshold", 0.5);
  heading_min_speed_ = declare_parameter<double>("heading_min_speed", 0.15);
  heading_good_variance_ = declare_parameter<double>("heading_good_variance", 0.1);
}

void GpsPoseConverterNode::create_publishers()
{
  pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gps/pose", rclcpp::QoS(10));
}

void GpsPoseConverterNode::create_subscribers()
{
  abs_pose_sub_ = create_subscription<mowgli_interfaces::msg::AbsolutePose>(
      "/gps/absolute_pose",
      rclcpp::QoS(10),
      [this](mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
      {
        on_absolute_pose(msg);
      });
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------

void GpsPoseConverterNode::on_absolute_pose(
    mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
{
  const double xy_variance = compute_xy_variance(*msg);
  const double cur_x = msg->pose.pose.position.x;
  const double cur_y = msg->pose.pose.position.y;
  const rclcpp::Time cur_stamp = now();

  // --- Compute heading from consecutive GPS positions ---
  //
  // GPS heading is only reliable when:
  //   1. We have RTK fix or float (cm/dm-level accuracy)
  //   2. The robot is moving fast enough for the velocity vector to be meaningful
  //
  // When stationary or without RTK, yaw variance is set very large so the
  // IMU (magnetometer + gyro) and dock_pose_fix dominate heading.
  double yaw = 0.0;
  double yaw_variance = 1e6;  // default: very uncertain (IMU/dock dominates)

  using Flags = mowgli_interfaces::msg::AbsolutePose;
  const bool has_rtk = (msg->flags & (Flags::FLAG_GPS_RTK_FIXED | Flags::FLAG_GPS_RTK_FLOAT)) != 0u;

  if (has_prev_pose_ && has_rtk)
  {
    const double dt = (cur_stamp - prev_stamp_).seconds();
    if (dt > 0.01 && dt < 2.0)
    {  // sanity: 10ms < dt < 2s
      const double dx = cur_x - prev_x_;
      const double dy = cur_y - prev_y_;
      const double dist = std::sqrt(dx * dx + dy * dy);
      const double speed = dist / dt;

      if (speed > 0.01)
      {
        // Heading from velocity vector (atan2 gives yaw in ENU: 0=east, +CCW)
        yaw = std::atan2(dy, dx);

        if (speed >= heading_min_speed_)
        {
          // Good speed + RTK — GPS heading is reliable.
          // Scale variance inversely with speed²: faster = tighter.
          const double speed_ratio = heading_min_speed_ / speed;
          yaw_variance = heading_good_variance_ * speed_ratio * speed_ratio;

          // RTK fixed is more accurate than float
          if ((msg->flags & Flags::FLAG_GPS_RTK_FLOAT) != 0u &&
              (msg->flags & Flags::FLAG_GPS_RTK_FIXED) == 0u)
          {
            yaw_variance *= 4.0;  // float is ~4x less accurate
          }
        }
        else
        {
          // Below speed threshold — smooth ramp to large variance.
          const double alpha = speed / heading_min_speed_;  // 0..1
          yaw_variance = heading_good_variance_ / (alpha * alpha + 0.001);
          yaw_variance = std::min(yaw_variance, 1e6);
        }
      }
    }
  }

  // Update previous pose for next iteration
  prev_x_ = cur_x;
  prev_y_ = cur_y;
  prev_stamp_ = cur_stamp;
  has_prev_pose_ = true;

  // --- Build PoseWithCovarianceStamped ---
  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp = cur_stamp;
  out.header.frame_id = "map";

  out.pose.pose.position = msg->pose.pose.position;

  // Set orientation from computed heading
  out.pose.pose.orientation.w = std::cos(yaw / 2.0);
  out.pose.pose.orientation.z = std::sin(yaw / 2.0);
  out.pose.pose.orientation.x = 0.0;
  out.pose.pose.orientation.y = 0.0;

  out.pose.covariance[0] = xy_variance;  // x
  out.pose.covariance[7] = xy_variance;  // y
  out.pose.covariance[14] = 1e6;  // z  (2D mode)
  out.pose.covariance[21] = 1e6;  // roll
  out.pose.covariance[28] = 1e6;  // pitch
  out.pose.covariance[35] = yaw_variance;  // yaw — tight when moving, loose when stationary

  pose_pub_->publish(out);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

double GpsPoseConverterNode::compute_xy_variance(
    const mowgli_interfaces::msg::AbsolutePose& msg) const
{
  using Flags = mowgli_interfaces::msg::AbsolutePose;

  double multiplier = 1.0;

  if ((msg.flags & Flags::FLAG_GPS_RTK_FIXED) != 0u)
  {
    multiplier = 1.0;
  }
  else if ((msg.flags & Flags::FLAG_GPS_RTK_FLOAT) != 0u)
  {
    multiplier = 4.0;
  }
  else if ((msg.flags & Flags::FLAG_GPS_RTK) != 0u)
  {
    multiplier = 16.0;
  }
  else if ((msg.flags & Flags::FLAG_GPS_DEAD_RECKONING) != 0u)
  {
    multiplier = 64.0;
  }
  else
  {
    multiplier = 32.0;
  }

  const double sigma = (msg.position_accuracy > 0.0f)
                           ? std::max(static_cast<double>(msg.position_accuracy), 0.01)
                           : 5.0;

  return sigma * sigma * multiplier;
}

}  // namespace mowgli_localization

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::GpsPoseConverterNode>());
  rclcpp::shutdown();
  return 0;
}
