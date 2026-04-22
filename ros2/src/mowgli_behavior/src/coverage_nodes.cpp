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
// GetNextUnmowedArea — iterate areas, find first with strips remaining
// ===========================================================================

BT::NodeStatus GetNextUnmowedArea::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  if (!client_)
  {
    client_ = helper->create_client<mowgli_interfaces::srv::GetCoverageStatus>(
        "/map_server_node/get_coverage_status");
  }

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "GetNextUnmowedArea: get_coverage_status service not available");
    return BT::NodeStatus::FAILURE;
  }

  uint32_t max_areas = 20;
  getInput<uint32_t>("max_areas", max_areas);

  uint32_t areas_queried = 0;
  uint32_t areas_complete = 0;

  for (uint32_t i = 0; i < max_areas; ++i)
  {
    auto request = std::make_shared<mowgli_interfaces::srv::GetCoverageStatus::Request>();
    request->area_index = i;

    auto future = client_->async_send_request(request);
    // Poll future without spinning (avoids executor deadlock)
    {
      auto timeout = std::chrono::seconds(2);
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
        // Service call timed out. This is DIFFERENT from "no more areas";
        // we must NOT conclude mowing is done. FAIL loudly so the BT can
        // retry instead of falling through to the dock-return branch.
        RCLCPP_ERROR(ctx->node->get_logger(),
                     "GetNextUnmowedArea: get_coverage_status timed out for area %u after 2s — "
                     "returning FAILURE (BT should retry, not assume mowing complete)",
                     i);
        return BT::NodeStatus::FAILURE;
      }
    }

    auto response = future.get();
    if (!response->success)
    {
      // Area index out of range — no more areas to check.
      // If we haven't queried any area yet, this means map_server has no
      // areas defined at all — distinct from "all areas mowed".
      if (areas_queried == 0)
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "GetNextUnmowedArea: no mowing areas defined in map_server "
                    "(first get_coverage_status returned success=false). "
                    "Record an area via the GUI before starting mowing.");
      }
      break;
    }

    areas_queried++;

    if (response->strips_remaining > 0)
    {
      setOutput("area_index", i);
      ctx->current_area = static_cast<int>(i);

      RCLCPP_INFO(ctx->node->get_logger(),
                  "GetNextUnmowedArea: area %u has %u strips remaining (%.1f%% done)",
                  i,
                  response->strips_remaining,
                  response->coverage_percent);
      return BT::NodeStatus::SUCCESS;
    }

    areas_complete++;
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: area %u complete (%.1f%%)",
                i,
                response->coverage_percent);
  }

  if (areas_queried == 0)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "GetNextUnmowedArea: no areas to mow (none defined)");
  }
  else
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: all %u area(s) complete",
                areas_complete);
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace mowgli_behavior
