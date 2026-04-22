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

#include "mowgli_behavior/navigation_nodes.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace
{

/// Parse a pose string "x;y;yaw" and fill a PoseStamped (frame_id = "map").
geometry_msgs::msg::PoseStamped parsePoseString(const std::string& pose_str,
                                                const rclcpp::Node::SharedPtr& node)
{
  std::istringstream ss(pose_str);
  std::string token;
  double x = 0.0, y = 0.0, yaw = 0.0;

  if (!std::getline(ss, token, ';'))
  {
    throw std::invalid_argument("NavigateToPose: missing 'x' in goal string");
  }
  x = std::stod(token);

  if (!std::getline(ss, token, ';'))
  {
    throw std::invalid_argument("NavigateToPose: missing 'y' in goal string");
  }
  y = std::stod(token);

  if (!std::getline(ss, token, ';'))
  {
    throw std::invalid_argument("NavigateToPose: missing 'yaw' in goal string");
  }
  yaw = std::stod(token);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node->now();
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

}  // namespace

// ---------------------------------------------------------------------------
// StopMoving
// ---------------------------------------------------------------------------

BT::NodeStatus StopMoving::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!pub_)
  {
    pub_ = ctx->node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_emergency", 10);
  }

  geometry_msgs::msg::TwistStamped zero{};
  zero.header.stamp = ctx->node->now();
  zero.header.frame_id = "base_footprint";
  pub_->publish(zero);

  RCLCPP_DEBUG(ctx->node->get_logger(), "StopMoving: published zero velocity on /cmd_vel_emergency");

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// ClearCostmap
// ---------------------------------------------------------------------------

BT::NodeStatus ClearCostmap::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!global_client_)
  {
    global_client_ = ctx->node->create_client<std_srvs::srv::Empty>(
        "/global_costmap/clear_entirely_global_costmap");
  }
  if (!local_client_)
  {
    local_client_ = ctx->node->create_client<std_srvs::srv::Empty>(
        "/local_costmap/clear_entirely_local_costmap");
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  // Just send the requests. If the service isn't ready, async_send_request
  // will fail silently (no response). This avoids DDS discovery issues
  // where service_is_ready() and wait_for_service() never return true
  // even though the services exist (Cyclone DDS on ARM).
  global_client_->async_send_request(request);
  local_client_->async_send_request(request);
  RCLCPP_INFO(ctx->node->get_logger(), "ClearCostmap: sent clear requests");

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// NavigateToPose
// ---------------------------------------------------------------------------

void NavigateToPose::ensureActionClient(const rclcpp::Node::SharedPtr& node)
{
  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<Nav2Goal>(node, "/navigate_to_pose");
  }
}

BT::NodeStatus NavigateToPose::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  auto goal_res = getInput<std::string>("goal");
  if (!goal_res)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "NavigateToPose: missing required port 'goal': %s",
                 goal_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped target_pose;
  try
  {
    target_pose = parsePoseString(goal_res.value(), ctx->node);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "NavigateToPose: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  ensureActionClient(ctx->node);

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "NavigateToPose: action server '/navigate_to_pose' not available");
    return BT::NodeStatus::FAILURE;
  }

  Nav2Goal::Goal goal_msg;
  goal_msg.pose = target_pose;

  auto send_goal_options = rclcpp_action::Client<Nav2Goal>::SendGoalOptions{};

  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
              "NavigateToPose: goal sent (x=%.2f y=%.2f yaw=%.2f)",
              target_pose.pose.position.x,
              target_pose.pose.position.y,
              0.0 /* yaw logged for info, already in quaternion */);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPose::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Resolve the goal handle the first time it is ready.
  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "NavigateToPose: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "NavigateToPose: goal succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "NavigateToPose: goal aborted");
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "NavigateToPose: goal canceled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void NavigateToPose::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (goal_handle_)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "NavigateToPose: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// BackUp
// ---------------------------------------------------------------------------

BT::NodeStatus BackUp::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<BackUpAction>(ctx->node, "/backup");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "BackUp: /backup action server not available");
    return BT::NodeStatus::FAILURE;
  }

  double dist = 0.5;
  double speed = 0.15;
  getInput("backup_dist", dist);
  getInput("backup_speed", speed);

  auto goal_msg = BackUpAction::Goal{};
  // BackUp target is negative X (reverse) in base_link frame
  goal_msg.target.x = -dist;
  goal_msg.target.y = 0.0;
  goal_msg.speed = speed;
  // Generous timeout: slow motors need extra time. 3x nominal duration.
  goal_msg.time_allowance = rclcpp::Duration::from_seconds(dist / speed * 3.0);

  RCLCPP_INFO(ctx->node->get_logger(), "BackUp: reversing %.2fm at %.2f m/s", dist, speed);

  goal_handle_future_ = action_client_->async_send_goal(goal_msg);
  goal_handle_ = nullptr;
  result_requested_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BackUp::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Wait for goal acceptance
  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "BackUp: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  // Request result future only once
  if (!result_requested_)
  {
    result_future_ = action_client_->async_get_result(goal_handle_);
    result_requested_ = true;
  }

  if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    return BT::NodeStatus::RUNNING;
  }

  auto wrapped = result_future_.get();
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "BackUp: complete");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(ctx->node->get_logger(),
              "BackUp: action ended with code %d",
              static_cast<int>(wrapped.code));
  return BT::NodeStatus::FAILURE;
}

void BackUp::onHalted()
{
  if (goal_handle_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_INFO(ctx->node->get_logger(), "BackUp: halted, goal cancelled");
  }
  goal_handle_ = nullptr;
}

// ---------------------------------------------------------------------------
// SetNavMode
// ---------------------------------------------------------------------------

BT::NodeStatus SetNavMode::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  auto mode_res = getInput<std::string>("mode");
  if (!mode_res)
  {
    return BT::NodeStatus::FAILURE;
  }
  const std::string mode = mode_res.value();

  if (mode == ctx->current_nav_mode)
  {
    return BT::NodeStatus::SUCCESS;
  }

  // Reconfigure controller speed via dynamic parameter API.
  auto param_client =
      std::make_shared<rclcpp::AsyncParametersClient>(ctx->node, "/controller_server");

  if (!param_client->wait_for_service(std::chrono::milliseconds(200)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "SetNavMode: controller_server param service unavailable");
    // Still update the mode in context so we don't retry every tick.
    ctx->current_nav_mode = mode;
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<rclcpp::Parameter> params;
  if (mode == "precise")
  {
    params = {
        rclcpp::Parameter("FollowCoveragePath.desired_linear_vel", 0.5),
        rclcpp::Parameter("FollowPath.desired_linear_vel", 0.5),
    };
  }
  else
  {
    // degraded: half speed
    params = {
        rclcpp::Parameter("FollowCoveragePath.desired_linear_vel", 0.25),
        rclcpp::Parameter("FollowPath.desired_linear_vel", 0.25),
    };
  }

  param_client->set_parameters(params);
  ctx->current_nav_mode = mode;

  RCLCPP_INFO(ctx->node->get_logger(), "SetNavMode: mode set to '%s'", mode.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mowgli_behavior
