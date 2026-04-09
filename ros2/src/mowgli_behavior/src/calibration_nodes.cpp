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

#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"

namespace mowgli_behavior
{

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

  ctx->undock_start_recorded = false;

  // Read current pose from EKF via TF (map -> base_link).
  // The EKF already fuses the best available sources (GPS, IMU, wheel ticks),
  // so its heading is the most accurate estimate we have after undock.
  geometry_msgs::msg::TransformStamped tf_msg;
  try
  {
    tf_msg = ctx->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: TF lookup failed: %s",
                ex.what());
    return BT::NodeStatus::SUCCESS;  // non-fatal
  }

  const double pose_x = tf_msg.transform.translation.x;
  const double pose_y = tf_msg.transform.translation.y;
  const double heading = tf2::getYaw(tf_msg.transform.rotation);

  RCLCPP_INFO(ctx->node->get_logger(),
              "CalibrateHeadingFromUndock: EKF pose=(%.3f, %.3f) heading=%.1f deg",
              pose_x,
              pose_y,
              heading * 180.0 / M_PI);

  // In lifelong mode, do NOT reset SLAM — the accumulated posegraph is
  // valuable and scan matching will naturally correct any small drift.
  // Just clear costmaps to remove stale obstacle marks from the docking
  // area that may have been projected with slightly different TF.
  for (const auto& svc : {"/global_costmap/clear_entirely_global_costmap",
                          "/local_costmap/clear_entirely_local_costmap"})
  {
    auto cc = ctx->node->create_client<nav2_msgs::srv::ClearEntireCostmap>(svc);
    if (cc->wait_for_service(std::chrono::seconds(1)))
    {
      auto cr = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
      auto cf = cc->async_send_request(cr);
      (void)cf;
    }
  }
  RCLCPP_INFO(ctx->node->get_logger(), "CalibrateHeadingFromUndock: costmaps cleared");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mowgli_behavior
