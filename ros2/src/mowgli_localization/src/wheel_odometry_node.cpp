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
 * @file wheel_odometry_node.cpp
 * @brief Differential-drive wheel odometry implementation.
 *
 * Wheel assignment in the WheelTick message:
 *   RL (rear-left)  → left  drive wheel
 *   RR (rear-right) → right drive wheel
 *
 * The FL/FR tick fields are ignored because the Mowgli platform is a rear-
 * wheel-drive differential robot; the front wheels are passive casters.
 *
 * Differential drive kinematics (midpoint integration):
 *
 *   d_left   = (ticks_rl_delta) / ticks_per_meter
 *   d_right  = (ticks_rr_delta) / ticks_per_meter
 *   d_center = (d_left + d_right) / 2
 *   d_theta  = (d_right - d_left) / wheel_distance
 *
 *   x     += d_center * cos(theta + d_theta / 2)
 *   y     += d_center * sin(theta + d_theta / 2)
 *   theta += d_theta
 *
 * Covariance notes:
 *   The EKF in localization.yaml consumes only vx and vyaw from this source
 *   (odom0_config), so pose covariance is intentionally large (we do not claim
 *   accurate absolute position from dead-reckoning alone).  Twist covariance
 *   uses moderate values reflecting real encoder noise.
 */

#include "mowgli_localization/wheel_odometry_node.hpp"

#include <array>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace mowgli_localization
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

WheelOdometryNode::WheelOdometryNode(const rclcpp::NodeOptions& options)
    : Node("wheel_odometry", options)
{
  declare_parameters();
  create_publishers();
  create_subscribers();

  RCLCPP_INFO(
      get_logger(),
      "WheelOdometryNode started — wheel_distance=%.3f m, ticks_per_meter=%.1f, publish_tf=%s",
      wheel_distance_,
      ticks_per_meter_,
      publish_tf_ ? "true" : "false");
}

// ---------------------------------------------------------------------------
// Initialisation helpers
// ---------------------------------------------------------------------------

void WheelOdometryNode::declare_parameters()
{
  wheel_distance_ = declare_parameter<double>("wheel_distance", 0.35);
  ticks_per_meter_ = declare_parameter<double>("ticks_per_meter", 1000.0);
  publish_tf_ = declare_parameter<bool>("publish_tf", false);
}

void WheelOdometryNode::create_publishers()
{
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", rclcpp::QoS(10));

  if (publish_tf_)
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
}

void WheelOdometryNode::create_subscribers()
{
  wheel_tick_sub_ = create_subscription<mowgli_interfaces::msg::WheelTick>(
      "/wheel_ticks",
      rclcpp::QoS(10),
      [this](mowgli_interfaces::msg::WheelTick::ConstSharedPtr msg)
      {
        on_wheel_tick(msg);
      });
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------

void WheelOdometryNode::on_wheel_tick(mowgli_interfaces::msg::WheelTick::ConstSharedPtr msg)
{
  using Flags = mowgli_interfaces::msg::WheelTick;

  // Require both driven wheels to be valid before integrating.
  const bool rl_valid = (msg->valid_wheels & Flags::WHEEL_VALID_RL) != 0u;
  const bool rr_valid = (msg->valid_wheels & Flags::WHEEL_VALID_RR) != 0u;

  if (!rl_valid || !rr_valid)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "WheelTick message missing valid RL/RR ticks — skipping integration");
    return;
  }

  // On the very first message seed the previous tick counters and return;
  // we have no delta yet.
  if (first_tick_)
  {
    prev_ticks_left_ = msg->wheel_ticks_rl;
    prev_ticks_right_ = msg->wheel_ticks_rr;
    first_tick_ = false;
    return;
  }

  // ------------------------------------------------------------------
  // Compute signed tick deltas, respecting the direction flags.
  // direction == 1 → forward, direction == 0 → backward.
  // ------------------------------------------------------------------
  const auto ticks_delta = [](uint32_t current, uint32_t previous, uint8_t dir) -> double
  {
    // Tick counters are unsigned and may wrap.  Cast to signed 32-bit
    // difference to handle wraps up to ~2 billion ticks.
    const double raw =
        static_cast<double>(static_cast<int32_t>(current) - static_cast<int32_t>(previous));
    return (dir != 0u) ? raw : -raw;
  };

  const double delta_ticks_left =
      ticks_delta(msg->wheel_ticks_rl, prev_ticks_left_, msg->wheel_direction_rl);
  const double delta_ticks_right =
      ticks_delta(msg->wheel_ticks_rr, prev_ticks_right_, msg->wheel_direction_rr);

  prev_ticks_left_ = msg->wheel_ticks_rl;
  prev_ticks_right_ = msg->wheel_ticks_rr;

  // ------------------------------------------------------------------
  // Differential drive kinematics (midpoint integration)
  // ------------------------------------------------------------------
  const double d_left = delta_ticks_left / ticks_per_meter_;
  const double d_right = delta_ticks_right / ticks_per_meter_;
  const double d_center = (d_left + d_right) / 2.0;
  const double d_theta = (d_right - d_left) / wheel_distance_;

  // Use half-angle for midpoint integration (less drift than simple Euler).
  x_ += d_center * std::cos(theta_ + d_theta / 2.0);
  y_ += d_center * std::sin(theta_ + d_theta / 2.0);
  theta_ += d_theta;

  // Keep theta in [-π, π] to avoid floating-point precision loss over time.
  theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

  // ------------------------------------------------------------------
  // Build odometry message
  // ------------------------------------------------------------------
  const rclcpp::Time stamp = now();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  // Pose
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = yaw_to_quaternion(theta_);

  // Pose covariance (row-major 6×6: x y z roll pitch yaw).
  // Large diagonal — we trust the EKF to own the absolute pose; we only
  // contribute velocity information (odom0_config selects vx and vyaw).
  odom.pose.covariance[0] = 1e6;  // x  (untrusted — EKF uses velocity only)
  odom.pose.covariance[7] = 1e6;  // y  (untrusted — EKF uses velocity only)
  odom.pose.covariance[14] = 1e6;  // z  (irrelevant for 2D)
  odom.pose.covariance[21] = 1e6;  // roll
  odom.pose.covariance[28] = 1e6;  // pitch
  odom.pose.covariance[35] = 1e6;  // yaw (untrusted — EKF uses velocity only)

  // Twist (velocities in body frame)
  // Avoid division by zero: treat zero dt as zero velocity.
  // The EKF does its own differentiation from consecutive poses when
  // differential mode is on, but we publish a best-effort instantaneous v.
  // Use a nominal dt based on the tick_factor field if available, otherwise
  // assume 10 ms (the firmware typically sends at 100 Hz).
  constexpr double kNominalDt = 0.01;
  odom.twist.twist.linear.x = d_center / kNominalDt;
  odom.twist.twist.angular.z = d_theta / kNominalDt;

  // Twist covariance
  odom.twist.covariance[0] = 1e-3;  // vx
  odom.twist.covariance[7] = 1e6;  // vy  (non-holonomic: constrain later)
  odom.twist.covariance[14] = 1e6;  // vz
  odom.twist.covariance[21] = 1e6;  // vroll
  odom.twist.covariance[28] = 1e6;  // vpitch
  odom.twist.covariance[35] = 1e-3;  // vyaw

  odom_pub_->publish(odom);

  // ------------------------------------------------------------------
  // Optional TF broadcast
  // ------------------------------------------------------------------
  if (publish_tf_ && tf_broadcaster_)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = yaw_to_quaternion(theta_);

    tf_broadcaster_->sendTransform(tf);
  }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

geometry_msgs::msg::Quaternion WheelOdometryNode::yaw_to_quaternion(const double yaw)
{
  // For a pure yaw rotation:  q = (cos(yaw/2), 0, 0, sin(yaw/2))
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace mowgli_localization

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
