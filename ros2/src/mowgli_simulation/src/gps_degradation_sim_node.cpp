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
 * @file gps_degradation_sim_node.cpp
 * @brief GPS degradation simulator implementation.
 *
 * The node cycles between NORMAL and DEGRADED states on a configurable timer.
 * In NORMAL mode the incoming GPS pose is forwarded unchanged.  In DEGRADED
 * mode the xy covariance is multiplied by degradation_covariance_scale and an
 * optional random position drift is injected to mimic real-world float/no-fix
 * behaviour.
 *
 * The inflated covariance causes ekf_map_node (or fusion_graph_node when
 * use_fusion_graph is true) to down-weight or effectively ignore the GPS
 * measurement, forcing the system to rely on wheel odometry, IMU
 * dead-reckoning, and (when fusion_graph is enabled) LiDAR scan-matching
 * factors -- exactly the scenario we want to validate.
 */

#include "mowgli_simulation/gps_degradation_sim_node.hpp"

#include <chrono>
#include <random>
#include <string>

namespace mowgli_simulation
{

using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

GpsDegradationSimNode::GpsDegradationSimNode(const rclcpp::NodeOptions& options)
    : Node("gps_degradation_sim", options),
      rng_(std::random_device{}()),
      drift_dist_(0.0, 1.0)  // stddev applied at sampling time
{
  declare_parameters();
  create_publishers();
  create_subscribers();
  create_timer();

  RCLCPP_INFO(get_logger(),
              "GpsDegradationSimNode started — enabled=%s  normal=%.1fs  degraded=%.1fs  "
              "cov_scale=%.1f  drift=%s (stddev=%.2fm)",
              enabled_ ? "true" : "false",
              normal_duration_sec_,
              degraded_duration_sec_,
              degradation_covariance_scale_,
              enable_position_drift_ ? "true" : "false",
              drift_stddev_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Initialisation helpers
// ─────────────────────────────────────────────────────────────────────────────

void GpsDegradationSimNode::declare_parameters()
{
  normal_duration_sec_ = declare_parameter<double>("normal_duration_sec", 30.0);
  degraded_duration_sec_ = declare_parameter<double>("degraded_duration_sec", 10.0);
  degradation_covariance_scale_ = declare_parameter<double>("degradation_covariance_scale", 100.0);
  enable_position_drift_ = declare_parameter<bool>("enable_position_drift", true);
  drift_stddev_ = declare_parameter<double>("drift_stddev", 0.5);
  enabled_ = declare_parameter<bool>("enabled", true);
}

void GpsDegradationSimNode::create_publishers()
{
  pose_pub_ = create_publisher<PoseMsg>("/gps/pose_sim", rclcpp::QoS(10));
  status_pub_ = create_publisher<std_msgs::msg::String>("/gps_degradation_sim/status",
                                                        rclcpp::QoS(10).transient_local());
}

void GpsDegradationSimNode::create_subscribers()
{
  pose_sub_ = create_subscription<PoseMsg>("/gps/pose",
                                           rclcpp::QoS(10),
                                           [this](PoseMsg::ConstSharedPtr msg)
                                           {
                                             on_gps_pose(msg);
                                           });
}

void GpsDegradationSimNode::create_timer()
{
  // Start in NORMAL mode; the first timer fires after normal_duration_sec_.
  const auto period = std::chrono::duration<double>(normal_duration_sec_);
  cycle_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this]()
                                   {
                                     on_cycle_timer();
                                   });

  RCLCPP_INFO(get_logger(), "GPS SIM: entering NORMAL mode (RTK fixed)");
  publish_status();
}

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────

void GpsDegradationSimNode::on_gps_pose(PoseMsg::ConstSharedPtr msg)
{
  if (!enabled_)
  {
    // Disabled: pass through unchanged.
    pose_pub_->publish(*msg);
    return;
  }

  switch (state_)
  {
    case GpsState::kNormal:
      pose_pub_->publish(*msg);
      break;
    case GpsState::kDegraded:
      pose_pub_->publish(degrade_pose(*msg));
      break;
  }
}

void GpsDegradationSimNode::on_cycle_timer()
{
  if (!enabled_)
  {
    return;
  }

  // Toggle state.
  switch (state_)
  {
    case GpsState::kNormal:
    {
      state_ = GpsState::kDegraded;
      const auto period = std::chrono::duration<double>(degraded_duration_sec_);
      cycle_timer_->cancel();
      cycle_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                       [this]()
                                       {
                                         on_cycle_timer();
                                       });
      RCLCPP_INFO(get_logger(), "GPS SIM: entering DEGRADED mode (float)");
      break;
    }
    case GpsState::kDegraded:
    {
      state_ = GpsState::kNormal;
      const auto period = std::chrono::duration<double>(normal_duration_sec_);
      cycle_timer_->cancel();
      cycle_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                       [this]()
                                       {
                                         on_cycle_timer();
                                       });
      RCLCPP_INFO(get_logger(), "GPS SIM: entering NORMAL mode (RTK fixed)");
      break;
    }
  }

  publish_status();
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

PoseMsg GpsDegradationSimNode::degrade_pose(const PoseMsg& input) const
{
  // Build a new message (immutable pattern -- never modify the input).
  PoseMsg out = input;

  // Inflate xy covariance.
  // Indices in the 6x6 row-major covariance: [0]=x, [7]=y.
  out.pose.covariance[0] *= degradation_covariance_scale_;
  out.pose.covariance[7] *= degradation_covariance_scale_;

  // Optionally inject random position drift to simulate float/no-fix wander.
  if (enable_position_drift_)
  {
    const double dx = drift_dist_(rng_) * drift_stddev_;
    const double dy = drift_dist_(rng_) * drift_stddev_;
    out.pose.pose.position.x += dx;
    out.pose.pose.position.y += dy;
  }

  return out;
}

void GpsDegradationSimNode::publish_status() const
{
  std_msgs::msg::String status;
  status.data = state_label(state_);
  status_pub_->publish(status);
}

std::string GpsDegradationSimNode::state_label(GpsState state)
{
  switch (state)
  {
    case GpsState::kNormal:
      return "NORMAL";
    case GpsState::kDegraded:
      return "DEGRADED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace mowgli_simulation

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_simulation::GpsDegradationSimNode>());
  rclcpp::shutdown();
  return 0;
}
