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
 * @file gps_degradation_sim_node.hpp
 * @brief Simulates periodic GPS degradation for testing LiDAR-only localisation.
 *
 * This node intercepts GPS pose messages and periodically inflates their
 * covariance (with optional position drift) to simulate loss of RTK fix.
 * When the EKF sees the high covariance it naturally falls back to odometry
 * and LiDAR SLAM, exercising the system's resilience to GPS outages.
 *
 * Subscribed topics:
 *   /gps/pose   geometry_msgs/msg/PoseWithCovarianceStamped
 *
 * Published topics:
 *   /gps/pose_sim                        geometry_msgs/msg/PoseWithCovarianceStamped
 *   /gps_degradation_sim/status           std_msgs/msg/String
 *
 * In the simulation launch file, remap the EKF's GPS input from /gps/pose to
 * /gps/pose_sim so that degradation is injected transparently.
 *
 * Parameters:
 *   normal_duration_sec           (double, 30.0)  - seconds in normal mode
 *   degraded_duration_sec         (double, 10.0)  - seconds in degraded mode
 *   degradation_covariance_scale  (double, 100.0) - covariance multiplier
 *   enable_position_drift         (bool,   true)  - add random drift when degraded
 *   drift_stddev                  (double, 0.5)   - 1-sigma drift in metres
 *   enabled                       (bool,   true)  - master on/off switch
 */

#pragma once

#include <random>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace mowgli_simulation
{

class GpsDegradationSimNode : public rclcpp::Node
{
public:
  explicit GpsDegradationSimNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GpsDegradationSimNode() override = default;

private:
  // ── State machine ─────────────────────────────────────────────────────────
  enum class GpsState : uint8_t
  {
    kNormal,
    kDegraded,
  };

  // ── Initialisation helpers ────────────────────────────────────────────────
  void declare_parameters();
  void create_publishers();
  void create_subscribers();
  void create_timer();

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void on_gps_pose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_cycle_timer();

  // ── Helpers ───────────────────────────────────────────────────────────────

  /** Build a new PoseWithCovarianceStamped with inflated covariance and optional drift. */
  geometry_msgs::msg::PoseWithCovarianceStamped degrade_pose(
      const geometry_msgs::msg::PoseWithCovarianceStamped& input) const;

  /** Publish current state on the status topic. */
  void publish_status() const;

  /** Human-readable label for the current state. */
  static std::string state_label(GpsState state);

  // ── Parameters ────────────────────────────────────────────────────────────
  double normal_duration_sec_{30.0};
  double degraded_duration_sec_{10.0};
  double degradation_covariance_scale_{100.0};
  bool enable_position_drift_{true};
  double drift_stddev_{0.5};
  bool enabled_{true};

  // ── Runtime state ─────────────────────────────────────────────────────────
  GpsState state_{GpsState::kNormal};

  // ── Random number generator for position drift ────────────────────────────
  mutable std::mt19937 rng_;
  mutable std::normal_distribution<double> drift_dist_;

  // ── ROS handles ───────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr cycle_timer_;
};

}  // namespace mowgli_simulation
