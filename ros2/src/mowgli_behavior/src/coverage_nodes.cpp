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

#include "mowgli_behavior/coverage_nodes.hpp"

#include <cmath>

#include "action_msgs/msg/goal_status.hpp"
#include "tf2/exceptions.h"

namespace mowgli_behavior
{

// ===========================================================================
// GetNextStrip — fetch next unmowed strip from map_server
// ===========================================================================

BT::NodeStatus GetNextStrip::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  if (!client_)
  {
    client_ = helper->create_client<mowgli_interfaces::srv::GetNextStrip>(
        "/map_server_node/get_next_strip");
  }

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "GetNextStrip: service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::GetNextStrip::Request>();
  uint32_t area_idx = 0;
  getInput<uint32_t>("area_index", area_idx);
  request->area_index = area_idx;

  try
  {
    auto tf = ctx->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    request->robot_x = tf.transform.translation.x;
    request->robot_y = tf.transform.translation.y;
  }
  catch (const tf2::TransformException&)
  {
    request->robot_x = 0.0;
    request->robot_y = 0.0;
  }
  request->prefer_headland = false;

  // Synchronous service call — poll future without spinning (avoids executor deadlock)
  auto future = client_->async_send_request(request);
  {
    auto timeout = std::chrono::seconds(5);
    auto start = std::chrono::steady_clock::now();
    bool completed = false;
    while (rclcpp::ok())
    {
      if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
      {
        completed = true;
        break;
      }
      if (std::chrono::steady_clock::now() - start > timeout)
      {
        break;
      }
    }
    if (!completed)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "GetNextStrip: service call timed out");
      return BT::NodeStatus::FAILURE;
    }
  }

  auto response = future.get();

  if (!response->success)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "GetNextStrip: service returned failure");
    return BT::NodeStatus::FAILURE;
  }

  if (response->coverage_complete)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextStrip: coverage complete (%.1f%%)",
                response->coverage_percent);
    return BT::NodeStatus::FAILURE;  // FAILURE = no more strips → loop ends
  }

  if (response->strip_path.poses.empty())
  {
    RCLCPP_WARN(ctx->node->get_logger(), "GetNextStrip: empty strip path");
    return BT::NodeStatus::FAILURE;
  }

  ctx->current_strip_path = response->strip_path;
  ctx->current_transit_goal = response->transit_goal;
  ctx->coverage_percent = response->coverage_percent;

  RCLCPP_INFO(ctx->node->get_logger(),
              "GetNextStrip: %zu poses, %.1f%% coverage, %u strips left",
              response->strip_path.poses.size(),
              response->coverage_percent,
              response->strips_remaining);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetNextStrip::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void GetNextStrip::onHalted()
{
}

// ===========================================================================
// FollowStrip — follow strip path with FTCController
// ===========================================================================

BT::NodeStatus FollowStrip::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->current_strip_path.poses.empty())
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "FollowStrip: no strip path in context");
    return BT::NodeStatus::FAILURE;
  }

  if (!follow_client_)
  {
    follow_client_ = rclcpp_action::create_client<Nav2FollowPath>(ctx->node, "/follow_path");
  }
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "FollowStrip: follow_path not available");
    return BT::NodeStatus::FAILURE;
  }

  setBladeEnabled(true);
  blade_start_time_ = std::chrono::steady_clock::now();
  goal_sent_ = false;

  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowStrip: blade enabled, waiting %.1fs for spinup",
              kBladeSpinupDelaySec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowStrip::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Wait for blade to spin up before sending the path goal
  if (!goal_sent_)
  {
    auto elapsed = std::chrono::steady_clock::now() - blade_start_time_;
    if (elapsed < std::chrono::duration<double>(kBladeSpinupDelaySec))
      return BT::NodeStatus::RUNNING;

    // Spinup complete — send path goal
    Nav2FollowPath::Goal goal;
    goal.path = ctx->current_strip_path;
    goal.controller_id = "FollowCoveragePath";
    goal.goal_checker_id = "coverage_goal_checker";

    follow_handle_.reset();
    follow_future_ = follow_client_->async_send_goal(goal);
    goal_sent_ = true;

    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowStrip: sent %zu poses to FTCController",
                goal.path.poses.size());

    return BT::NodeStatus::RUNNING;
  }

  if (!follow_handle_)
  {
    if (follow_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    follow_handle_ = follow_future_.get();
    if (!follow_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "FollowStrip: goal rejected");
      setBladeEnabled(false);
      return BT::NodeStatus::FAILURE;
    }
  }

  auto status = follow_handle_->get_status();

  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "FollowStrip: strip completed");
    follow_handle_.reset();
    setBladeEnabled(false);
    return BT::NodeStatus::SUCCESS;
  }

  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "FollowStrip: aborted/canceled");
    follow_handle_.reset();
    setBladeEnabled(false);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void FollowStrip::onHalted()
{
  if (follow_handle_)
  {
    follow_client_->async_cancel_goal(follow_handle_);
  }
  follow_handle_.reset();
  setBladeEnabled(false);
}

void FollowStrip::setBladeEnabled(bool enabled)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (!blade_client_)
  {
    blade_client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control");
  }
  if (!blade_client_->wait_for_service(std::chrono::milliseconds(200)))
    return;

  auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  req->mow_enabled = enabled ? 1u : 0u;
  blade_client_->async_send_request(req);
}

// ===========================================================================
// TransitToStrip — navigate to strip start using Nav2
// ===========================================================================

BT::NodeStatus TransitToStrip::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  RCLCPP_INFO(ctx->node->get_logger(),
              "TransitToStrip: goal frame='%s' pos=(%.2f, %.2f)",
              ctx->current_transit_goal.header.frame_id.c_str(),
              ctx->current_transit_goal.pose.position.x,
              ctx->current_transit_goal.pose.position.y);

  if (!nav_client_)
  {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "TransitToStrip: navigate_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }

  Nav2Navigate::Goal goal;
  goal.pose = ctx->current_transit_goal;

  nav_handle_.reset();
  nav_future_ = nav_client_->async_send_goal(goal);

  RCLCPP_INFO(ctx->node->get_logger(),
              "TransitToStrip: navigating to (%.2f, %.2f)",
              goal.pose.pose.position.x,
              goal.pose.pose.position.y);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TransitToStrip::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!nav_handle_)
  {
    if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    nav_handle_ = nav_future_.get();
    if (!nav_handle_)
    {
      RCLCPP_WARN(ctx->node->get_logger(), "TransitToStrip: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  auto status = nav_handle_->get_status();

  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "TransitToStrip: arrived at strip start");
    nav_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "TransitToStrip: navigation failed");
    nav_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void TransitToStrip::onHalted()
{
  if (nav_handle_)
  {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  nav_handle_.reset();
}

// ===========================================================================
// DetourAroundObstacle — short side-step via global planner so the robot
// gets out from in front of an obstacle that aborted the strip.
// ===========================================================================

BT::NodeStatus DetourAroundObstacle::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  double forward_m = 0.8;
  double lateral_m = 0.6;
  getInput("forward_m", forward_m);
  getInput("lateral_m", lateral_m);

  // Read current pose from map → base_footprint. If TF isn't ready, bail —
  // the BT will fall through to SkipStrip and we don't risk sending a
  // stale-pose-based goal.
  geometry_msgs::msg::TransformStamped t_map_base;
  try
  {
    t_map_base = ctx->tf_buffer->lookupTransform("map",
                                                 "base_footprint",
                                                 tf2::TimePointZero,
                                                 tf2::durationFromSec(0.2));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DetourAroundObstacle: TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Yaw from quaternion: standard ZYX Euler extraction. Avoids pulling
  // in tf2_geometry_msgs just for tf2::getYaw().
  const auto& q = t_map_base.transform.rotation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  // Body-frame (forward, lateral) → map frame, added to current position.
  // Lateral positive = left (right-hand-rule with z-up).
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = ctx->node->now();
  goal.pose.position.x = t_map_base.transform.translation.x + cy * forward_m - sy * lateral_m;
  goal.pose.position.y = t_map_base.transform.translation.y + sy * forward_m + cy * lateral_m;
  goal.pose.position.z = 0.0;

  // Keep the same heading. The global planner adjusts the path heading;
  // we just don't want to hand Nav2 a wildly different goal yaw.
  goal.pose.orientation = t_map_base.transform.rotation;

  RCLCPP_INFO(ctx->node->get_logger(),
              "DetourAroundObstacle: goal=(%.2f, %.2f) "
              "from (%.2f, %.2f), forward=%.2f lateral=%.2f",
              goal.pose.position.x,
              goal.pose.position.y,
              t_map_base.transform.translation.x,
              t_map_base.transform.translation.y,
              forward_m,
              lateral_m);

  if (!nav_client_)
  {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "DetourAroundObstacle: navigate_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }

  Nav2Navigate::Goal nav_goal;
  nav_goal.pose = goal;

  nav_handle_.reset();
  nav_future_ = nav_client_->async_send_goal(nav_goal);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetourAroundObstacle::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!nav_handle_)
  {
    if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    nav_handle_ = nav_future_.get();
    if (!nav_handle_)
    {
      RCLCPP_WARN(ctx->node->get_logger(), "DetourAroundObstacle: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = nav_handle_->get_status();
  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "DetourAroundObstacle: detour complete");
    nav_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }
  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DetourAroundObstacle: detour navigation failed");
    nav_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void DetourAroundObstacle::onHalted()
{
  if (nav_handle_)
  {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  nav_handle_.reset();
}

// ===========================================================================
// GetNextUnmowedArea — iterate areas, find first with strips remaining
// ===========================================================================

BT::NodeStatus GetNextUnmowedArea::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  if (!client_)
  {
    client_ = helper->create_client<mowgli_interfaces::srv::GetCoverageStatus>(
        "/map_server_node/get_coverage_status");
  }

  if (!client_->service_is_ready())
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "GetNextUnmowedArea: get_coverage_status service not available");
    return BT::NodeStatus::FAILURE;
  }

  // Reset per-run state
  getInput<uint32_t>("max_areas", max_areas_);
  current_area_idx_ = 0;
  areas_queried_ = 0;
  areas_complete_ = 0;

  // Honor a one-shot user-selected target area (set by ~/start_in_area).
  // We start the iteration from the requested index AND clip max_areas_ to
  // (target + 1) so the BT mows just that area and exits MowingSequence,
  // instead of rolling over to the next area. The optional is consumed
  // here so subsequent COMMAND_START runs use the normal ordering.
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    if (ctx->target_area_index.has_value())
    {
      const int target = *ctx->target_area_index;
      if (target >= 0)
      {
        current_area_idx_ = static_cast<uint32_t>(target);
        max_areas_ = current_area_idx_ + 1;
        RCLCPP_INFO(ctx->node->get_logger(),
                    "GetNextUnmowedArea: targeted run — mowing only area %u (single-area mode)",
                    current_area_idx_);
      }
      ctx->target_area_index.reset();
    }
  }

  // Fire off the first async request
  auto request = std::make_shared<mowgli_interfaces::srv::GetCoverageStatus::Request>();
  request->area_index = current_area_idx_;
  pending_future_.emplace(client_->async_send_request(request));
  call_start_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetNextUnmowedArea::onRunning()
{
  // Check if current async call has completed
  if (pending_future_->future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    // Still waiting — check 2s timeout
    if (std::chrono::steady_clock::now() - call_start_ > std::chrono::seconds(2))
    {
      auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "GetNextUnmowedArea: get_coverage_status timed out for area %u after 2s — "
                   "returning FAILURE (BT should retry, not assume mowing complete)",
                   current_area_idx_);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  return processResponse();
}

BT::NodeStatus GetNextUnmowedArea::processResponse()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto response = pending_future_->future.get();

  if (!response->success)
  {
    // Area index out of range — no more areas to check.
    if (areas_queried_ == 0)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "GetNextUnmowedArea: no mowing areas defined in map_server "
                  "(first get_coverage_status returned success=false). "
                  "Record an area via the GUI before starting mowing.");
    }
    else
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "GetNextUnmowedArea: all %u area(s) complete",
                  areas_complete_);
    }
    return BT::NodeStatus::FAILURE;
  }

  areas_queried_++;

  if (response->strips_remaining > 0)
  {
    setOutput("area_index", current_area_idx_);
    ctx->current_area = static_cast<int>(current_area_idx_);

    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: area %u has %u strips remaining (%.1f%% done)",
                current_area_idx_,
                response->strips_remaining,
                response->coverage_percent);
    return BT::NodeStatus::SUCCESS;
  }

  areas_complete_++;
  RCLCPP_INFO(ctx->node->get_logger(),
              "GetNextUnmowedArea: area %u complete (%.1f%%)",
              current_area_idx_,
              response->coverage_percent);

  // Move to the next area
  current_area_idx_++;
  if (current_area_idx_ >= max_areas_)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: all %u area(s) complete",
                areas_complete_);
    return BT::NodeStatus::FAILURE;
  }

  // Fire off the next async request
  auto request = std::make_shared<mowgli_interfaces::srv::GetCoverageStatus::Request>();
  request->area_index = current_area_idx_;
  pending_future_.emplace(client_->async_send_request(request));
  call_start_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

void GetNextUnmowedArea::onHalted()
{
  // Nothing to cancel — service calls complete on their own.
  // State will be reset in onStart() on next invocation.
}

// ===========================================================================
// GetNextSegment (Path C — cell-based coverage)
// ===========================================================================

BT::NodeStatus GetNextSegment::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  if (!client_)
  {
    client_ = helper->create_client<mowgli_interfaces::srv::GetNextSegment>(
        "/map_server_node/get_next_segment");
  }
  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "GetNextSegment: service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::GetNextSegment::Request>();
  uint32_t area_idx = 0;
  getInput<uint32_t>("area_index", area_idx);
  request->area_index = area_idx;

  // Robot pose from TF.
  double yaw = 0.0;
  try
  {
    auto tf = ctx->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    request->robot_x = tf.transform.translation.x;
    request->robot_y = tf.transform.translation.y;
    const auto& q = tf.transform.rotation;
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  catch (const tf2::TransformException&)
  {
    request->robot_x = 0.0;
    request->robot_y = 0.0;
  }
  request->robot_yaw_rad = yaw;

  // prefer_dir: NaN → use the robot's current heading as a hint so the
  // server's auto-MBR direction kicks in. The server's
  // compute_optimal_mow_angle is internal to ensure_strip_layout —
  // for now we pass the robot heading so the first segment doesn't
  // require a 180° rotation. A future iteration can extract the
  // per-area auto angle into a service parameter.
  double prefer_dir = std::numeric_limits<double>::quiet_NaN();
  getInput<double>("prefer_dir_yaw_rad", prefer_dir);
  if (!std::isfinite(prefer_dir))
    prefer_dir = yaw;
  request->prefer_dir_yaw_rad = prefer_dir;

  bool boust = true;
  getInput<bool>("boustrophedon", boust);
  request->boustrophedon = boust;

  double max_len = 0.0;
  getInput<double>("max_segment_length_m", max_len);
  request->max_segment_length_m = max_len;

  auto future = client_->async_send_request(request);
  {
    auto timeout = std::chrono::seconds(5);
    auto start = std::chrono::steady_clock::now();
    bool completed = false;
    while (rclcpp::ok())
    {
      if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
      {
        completed = true;
        break;
      }
      if (std::chrono::steady_clock::now() - start > timeout)
        break;
    }
    if (!completed)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "GetNextSegment: service call timed out");
      return BT::NodeStatus::FAILURE;
    }
  }

  auto response = future.get();
  if (!response->success)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "GetNextSegment: service returned failure");
    return BT::NodeStatus::FAILURE;
  }
  if (response->coverage_complete)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextSegment: area %u coverage complete (%.1f%%)",
                area_idx,
                response->coverage_percent);
    return BT::NodeStatus::FAILURE;  // FAILURE → outer loop picks next area.
  }
  if (response->segment_path.poses.empty())
  {
    RCLCPP_WARN(ctx->node->get_logger(), "GetNextSegment: empty segment path");
    return BT::NodeStatus::FAILURE;
  }

  ctx->current_strip_path = response->segment_path;
  ctx->current_transit_goal = response->target_cell_pose;
  ctx->coverage_percent = response->coverage_percent;
  ctx->current_segment_is_long_transit = response->is_long_transit;
  ctx->current_segment_phase = response->phase;
  ctx->current_segment_termination_reason = response->termination_reason;

  RCLCPP_INFO(ctx->node->get_logger(),
              "GetNextSegment: area %u, %zu poses, %.1f%% coverage, "
              "transit=%s, end=%s, dead_cells=%u",
              area_idx,
              response->segment_path.poses.size(),
              response->coverage_percent,
              response->is_long_transit ? "yes" : "no",
              response->termination_reason.c_str(),
              response->dead_cells_count);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetNextSegment::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void GetNextSegment::onHalted()
{
}

// ===========================================================================
// IsShortSegment (condition)
// ===========================================================================

BT::NodeStatus IsShortSegment::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  return ctx->current_segment_is_long_transit ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

// ===========================================================================
// MarkSegmentBlocked
// ===========================================================================

BT::NodeStatus MarkSegmentBlocked::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  // Don't bump fail_count when the previous GetNextSegment ended at a
  // known obstacle / dead cell — the failure is already accounted for.
  if (ctx->current_segment_termination_reason == "obstacle" ||
      ctx->current_segment_termination_reason == "dead_zone" ||
      ctx->current_segment_termination_reason == "boundary")
  {
    RCLCPP_DEBUG(ctx->node->get_logger(),
                 "MarkSegmentBlocked: skipping bump — segment ended at "
                 "known '%s'",
                 ctx->current_segment_termination_reason.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if (ctx->current_strip_path.poses.empty())
  {
    return BT::NodeStatus::SUCCESS;
  }

  if (!client_)
  {
    client_ = helper->create_client<mowgli_interfaces::srv::MarkSegmentBlocked>(
        "/map_server_node/mark_segment_blocked");
  }
  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "MarkSegmentBlocked: service unavailable, skipping");
    return BT::NodeStatus::SUCCESS;
  }

  uint32_t area_idx = 0;
  getInput<uint32_t>("area_index", area_idx);

  auto request = std::make_shared<mowgli_interfaces::srv::MarkSegmentBlocked::Request>();
  request->area_index = area_idx;
  request->failed_path = ctx->current_strip_path;

  future_ = client_->async_send_request(request).future.share();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MarkSegmentBlocked::onRunning()
{
  if (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    return BT::NodeStatus::RUNNING;

  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto resp = future_.get();
  if (resp && resp->success)
  {
    if (resp->cells_promoted_dead > 0)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "MarkSegmentBlocked: %u cells bumped, %u promoted DEAD",
                  resp->cells_marked_blocked,
                  resp->cells_promoted_dead);
    }
    else
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "MarkSegmentBlocked: %u cells bumped",
                  resp->cells_marked_blocked);
    }
  }
  // Always SUCCESS — this is bookkeeping. The outer Fallback handles
  // the actual control-flow recovery (next segment / next area).
  return BT::NodeStatus::SUCCESS;
}

void MarkSegmentBlocked::onHalted()
{
}

}  // namespace mowgli_behavior
