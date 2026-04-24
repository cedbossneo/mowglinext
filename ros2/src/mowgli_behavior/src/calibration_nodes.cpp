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

#include "mowgli_behavior/calibration_nodes.hpp"

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"

namespace mowgli_behavior
{

namespace
{

// Fill the covariance block for a yaw-plus-xy seed: tight trust on the
// states we want to set, effectively infinite variance on the states we
// want the filter to keep from its prior.
void set_seed_covariance(geometry_msgs::msg::PoseWithCovarianceStamped& msg,
                         double yaw_var)
{
  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0]  = 0.01;      // x
  msg.pose.covariance[7]  = 0.01;      // y
  msg.pose.covariance[14] = 1e6;       // z (keep prior)
  msg.pose.covariance[21] = 1e6;       // roll (keep prior)
  msg.pose.covariance[28] = 1e6;       // pitch (keep prior)
  msg.pose.covariance[35] = yaw_var;
}

}  // namespace

// ---------------------------------------------------------------------------
// RecordUndockStart
// ---------------------------------------------------------------------------

BT::NodeStatus RecordUndockStart::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->undock_start_x = ctx->gps_x;
  ctx->undock_start_y = ctx->gps_y;
  ctx->undock_start_recorded = true;
  RCLCPP_INFO(ctx->node->get_logger(),
              "RecordUndockStart: pos=(%.3f, %.3f)",
              ctx->undock_start_x,
              ctx->undock_start_y);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// CalibrateHeadingFromUndock
// ---------------------------------------------------------------------------

BT::NodeStatus CalibrateHeadingFromUndock::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!ctx->undock_start_recorded)
  {
    // RecordUndockStart never fired (e.g., fresh retry after a failed
    // undock where the node was halted). Report SUCCESS rather than
    // FAILURE so the rest of the sequence runs; the dock_yaw injection
    // (hardware_bridge → ekf_map via dock_yaw_to_set_pose) is still
    // active and will have seeded the filter from the charging state.
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: no undock_start recorded, "
                "skipping (relying on dock_yaw seed).");
    return BT::NodeStatus::SUCCESS;
  }

  double min_displacement = 0.5;
  getInput("min_displacement_m", min_displacement);

  const double dx = ctx->gps_x - ctx->undock_start_x;
  const double dy = ctx->gps_y - ctx->undock_start_y;
  const double dist = std::hypot(dx, dy);

  if (dist < min_displacement)
  {
    const bool still_on_dock = ctx->latest_power.charger_enabled;
    ctx->undock_start_recorded = false;
    if (still_on_dock)
    {
      // BackUp reported complete but GPS barely moved AND the dock still
      // charges — robot is genuinely stuck on the dock latch. Fail so
      // UndockSequence fails and the outer ReactiveSequence retries.
      RCLCPP_WARN(ctx->node->get_logger(),
                  "CalibrateHeadingFromUndock: displacement %.3fm below min %.3fm "
                  "AND is_charging=true — robot stuck on dock, retrying undock.",
                  dist, min_displacement);
      return BT::NodeStatus::FAILURE;
    }
    // Partial undock: GPS moved less than expected (wheel slip on the dock
    // ramp is common) but charging has dropped, so the robot IS off the
    // dock. The displacement is too short to refine yaw reliably — trust
    // the dock_yaw injected by dock_yaw_to_set_pose while still on the
    // dock and continue with the rest of the session.
    RCLCPP_INFO(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: partial undock (%.3fm < %.3fm) but "
                "charging dropped — keeping dock_yaw seed, skipping refinement.",
                dist, min_displacement);
    ctx->yaw_seeded_this_session = true;
    return BT::NodeStatus::SUCCESS;
  }

  // Robot moved backward during the BackUp. Motion vector (dx, dy) points
  // OPPOSITE to the robot's heading, so heading = atan2(-dy, -dx).
  const double yaw = std::atan2(-dy, -dx);

  if (!set_pose_pub_)
  {
    auto qos = rclcpp::QoS(1).reliable();
    set_pose_pub_ = ctx->node->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ekf_map_node/set_pose", qos);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped seed{};
  seed.header.stamp = ctx->node->now();
  seed.header.frame_id = "map";
  seed.pose.pose.position.x = ctx->gps_x;
  seed.pose.pose.position.y = ctx->gps_y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  seed.pose.pose.orientation.x = q.x();
  seed.pose.pose.orientation.y = q.y();
  seed.pose.pose.orientation.z = q.z();
  seed.pose.pose.orientation.w = q.w();
  // σ ≈ atan(GPS_σ / displacement). With σ_GPS ~ 7 mm and displacement
  // ≥ 0.5 m that's ~0.014 rad ≈ 0.8°; variance = 2 × 10⁻⁴.
  set_seed_covariance(seed, 2e-4);
  set_pose_pub_->publish(seed);

  ctx->undock_start_recorded = false;
  ctx->yaw_seeded_this_session = true;

  RCLCPP_INFO(ctx->node->get_logger(),
              "CalibrateHeadingFromUndock: dist=%.3fm yaw_seed=%.1f° "
              "pos=(%.3f, %.3f) — set_pose published.",
              dist, yaw * 180.0 / M_PI, ctx->gps_x, ctx->gps_y);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// SeedYawFromMotion
// ---------------------------------------------------------------------------

void SeedYawFromMotion::publish_forward(const rclcpp::Node::SharedPtr& node,
                                        double speed)
{
  geometry_msgs::msg::TwistStamped cmd{};
  cmd.header.stamp = node->now();
  cmd.header.frame_id = "base_footprint";
  cmd.twist.linear.x = speed;
  cmd_pub_->publish(cmd);
}

void SeedYawFromMotion::publish_zero(const rclcpp::Node::SharedPtr& node)
{
  publish_forward(node, 0.0);
}

BT::NodeStatus SeedYawFromMotion::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->yaw_seeded_this_session)
  {
    // Already seeded this session — don't re-drive on ReactiveSequence
    // halt/resume cycles.
    RCLCPP_DEBUG(ctx->node->get_logger(),
                 "SeedYawFromMotion: yaw already seeded this session, "
                 "skipping forward drive.");
    return BT::NodeStatus::SUCCESS;
  }

  if (!cmd_pub_)
  {
    cmd_pub_ = ctx->node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_teleop", 10);
  }
  if (!set_pose_pub_)
  {
    auto qos = rclcpp::QoS(1).reliable();
    set_pose_pub_ = ctx->node->create_publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ekf_map_node/set_pose", qos);
  }

  distance_m_ = 1.0;
  speed_ms_ = 0.2;
  timeout_sec_ = 30.0;
  min_displacement_m_ = 0.5;
  getInput("distance_m", distance_m_);
  getInput("speed_ms", speed_ms_);
  getInput("timeout_sec", timeout_sec_);
  getInput("min_displacement_m", min_displacement_m_);

  if (!ctx->gps_is_fixed)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: no RTK fix at start (fix_type=%u), "
                "aborting seed.",
                ctx->gps_fix_type);
    return BT::NodeStatus::FAILURE;
  }

  x0_ = ctx->gps_x;
  y0_ = ctx->gps_y;
  start_time_ = ctx->node->now();

  RCLCPP_INFO(ctx->node->get_logger(),
              "SeedYawFromMotion: start pos=(%.3f, %.3f), drive %.2fm fwd at %.2fm/s",
              x0_, y0_, distance_m_, speed_ms_);
  publish_forward(ctx->node, speed_ms_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SeedYawFromMotion::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->latest_emergency.active_emergency ||
      ctx->latest_emergency.latched_emergency)
  {
    publish_zero(ctx->node);
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: emergency during seed drive, aborting.");
    return BT::NodeStatus::FAILURE;
  }

  const double elapsed = (ctx->node->now() - start_time_).seconds();
  if (elapsed > timeout_sec_)
  {
    publish_zero(ctx->node);
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: timeout after %.1fs, aborting.", elapsed);
    return BT::NodeStatus::FAILURE;
  }

  const double dx = ctx->gps_x - x0_;
  const double dy = ctx->gps_y - y0_;
  const double dist = std::hypot(dx, dy);

  if (dist < distance_m_)
  {
    publish_forward(ctx->node, speed_ms_);
    return BT::NodeStatus::RUNNING;
  }

  publish_zero(ctx->node);

  if (dist < min_displacement_m_)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: displacement %.3fm below min %.3fm, "
                "seed invalid.",
                dist, min_displacement_m_);
    return BT::NodeStatus::FAILURE;
  }

  const double yaw = std::atan2(dy, dx);

  geometry_msgs::msg::PoseWithCovarianceStamped seed{};
  seed.header.stamp = ctx->node->now();
  seed.header.frame_id = "map";
  seed.pose.pose.position.x = ctx->gps_x;
  seed.pose.pose.position.y = ctx->gps_y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  seed.pose.pose.orientation.x = q.x();
  seed.pose.pose.orientation.y = q.y();
  seed.pose.pose.orientation.z = q.z();
  seed.pose.pose.orientation.w = q.w();
  set_seed_covariance(seed, 5e-3);  // ~4° σ
  set_pose_pub_->publish(seed);

  ctx->yaw_seeded_this_session = true;

  RCLCPP_INFO(ctx->node->get_logger(),
              "SeedYawFromMotion: dist=%.3fm yaw_seed=%.1f° pos=(%.3f, %.3f) — "
              "set_pose published.",
              dist, yaw * 180.0 / M_PI, ctx->gps_x, ctx->gps_y);
  return BT::NodeStatus::SUCCESS;
}

void SeedYawFromMotion::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (cmd_pub_)
  {
    publish_zero(ctx->node);
  }
}

}  // namespace mowgli_behavior
