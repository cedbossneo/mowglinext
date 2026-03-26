#include "mowgli_behavior/action_nodes.hpp"

#include <sstream>
#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_behavior {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace {

/// Parse a pose string "x;y;yaw" and fill a PoseStamped (frame_id = "map").
geometry_msgs::msg::PoseStamped parsePoseString(
  const std::string& pose_str,
  const rclcpp::Node::SharedPtr& node)
{
  std::istringstream ss(pose_str);
  std::string token;
  double x = 0.0, y = 0.0, yaw = 0.0;

  if (!std::getline(ss, token, ';')) {
    throw std::invalid_argument("NavigateToPose: missing 'x' in goal string");
  }
  x = std::stod(token);

  if (!std::getline(ss, token, ';')) {
    throw std::invalid_argument("NavigateToPose: missing 'y' in goal string");
  }
  y = std::stod(token);

  if (!std::getline(ss, token, ';')) {
    throw std::invalid_argument("NavigateToPose: missing 'yaw' in goal string");
  }
  yaw = std::stod(token);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp    = node->now();
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

/// Spin the node's executor for up to timeout_ms to wait for a service.
bool waitForService(
  rclcpp::ClientBase::SharedPtr client,
  const rclcpp::Node::SharedPtr& node,
  int timeout_ms = 1000)
{
  if (client->wait_for_service(std::chrono::milliseconds(timeout_ms))) {
    return true;
  }
  RCLCPP_WARN(node->get_logger(), "Service '%s' not available", client->get_service_name());
  return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// SetMowerEnabled
// ---------------------------------------------------------------------------

BT::NodeStatus SetMowerEnabled::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  auto res = getInput<bool>("enabled");
  if (!res) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "SetMowerEnabled: missing required port 'enabled': %s", res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const bool enabled = res.value();

  if (!client_) {
    client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
      "/hardware_bridge/mower_control");
  }

  if (!waitForService(client_, ctx->node)) {
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  request->mow_enabled  = enabled ? 1u : 0u;
  request->mow_direction = 0u;

  auto future = client_->async_send_request(request);

  // Spin until result arrives (or 2-second timeout).
  if (rclcpp::spin_until_future_complete(
        ctx->node, future, std::chrono::seconds(2))
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "SetMowerEnabled: service call timed out");
    return BT::NodeStatus::FAILURE;
  }

  const bool success = future.get()->success;
  if (!success) {
    RCLCPP_WARN(ctx->node->get_logger(),
      "SetMowerEnabled: service returned failure (enabled=%s)", enabled ? "true" : "false");
  }

  RCLCPP_DEBUG(ctx->node->get_logger(),
    "SetMowerEnabled: mow_enabled set to %s", enabled ? "true" : "false");

  return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// StopMoving
// ---------------------------------------------------------------------------

BT::NodeStatus StopMoving::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!pub_) {
    pub_ = ctx->node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  geometry_msgs::msg::Twist zero{};
  pub_->publish(zero);

  RCLCPP_DEBUG(ctx->node->get_logger(), "StopMoving: published zero velocity");

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// PublishHighLevelStatus
// ---------------------------------------------------------------------------

BT::NodeStatus PublishHighLevelStatus::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  auto state_res = getInput<uint8_t>("state");
  if (!state_res) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "PublishHighLevelStatus: missing required port 'state': %s",
      state_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto name_res = getInput<std::string>("state_name");
  if (!name_res) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "PublishHighLevelStatus: missing required port 'state_name': %s",
      name_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!pub_) {
    pub_ = ctx->node->create_publisher<mowgli_interfaces::msg::HighLevelStatus>(
      "~/high_level_status", 10);
  }

  mowgli_interfaces::msg::HighLevelStatus msg;
  msg.state           = state_res.value();
  msg.state_name      = name_res.value();
  msg.sub_state_name  = "";
  msg.current_area    = -1;
  msg.current_path    = -1;
  msg.current_path_index = -1;
  msg.gps_quality_percent = ctx->gps_quality;
  msg.battery_percent = ctx->battery_percent;
  msg.is_charging     = ctx->latest_power.charger_enabled;
  msg.emergency       = ctx->latest_emergency.active_emergency;

  pub_->publish(msg);

  RCLCPP_DEBUG(ctx->node->get_logger(),
    "PublishHighLevelStatus: state=%u name='%s'", msg.state, msg.state_name.c_str());

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// WaitForDuration
// ---------------------------------------------------------------------------

BT::NodeStatus WaitForDuration::onStart()
{
  double duration_sec = 1.0;
  if (auto res = getInput<double>("duration_sec")) {
    duration_sec = res.value();
  }

  duration_   = std::chrono::duration<double>(duration_sec);
  start_time_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDuration::onRunning()
{
  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  return elapsed >= duration_
    ? BT::NodeStatus::SUCCESS
    : BT::NodeStatus::RUNNING;
}

void WaitForDuration::onHalted()
{
  // Nothing to clean up; the timer is purely in-process.
}

// ---------------------------------------------------------------------------
// NavigateToPose
// ---------------------------------------------------------------------------

void NavigateToPose::ensureActionClient(const rclcpp::Node::SharedPtr& node)
{
  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<Nav2Goal>(
      node, "/navigate_to_pose");
  }
}

BT::NodeStatus NavigateToPose::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  auto goal_res = getInput<std::string>("goal");
  if (!goal_res) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "NavigateToPose: missing required port 'goal': %s", goal_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped target_pose;
  try {
    target_pose = parsePoseString(goal_res.value(), ctx->node);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(ctx->node->get_logger(), "NavigateToPose: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  ensureActionClient(ctx->node);

  if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
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
  if (!goal_handle_) {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0))
        != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(ctx->node->get_logger(),
        "NavigateToPose: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status) {
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

  if (goal_handle_) {
    RCLCPP_INFO(ctx->node->get_logger(), "NavigateToPose: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// PlanCoveragePath
// ---------------------------------------------------------------------------

BT::NodeStatus PlanCoveragePath::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  uint32_t area_index = 0u;
  if (auto res = getInput<uint32_t>("area_index")) {
    area_index = res.value();
  }

  if (!client_) {
    client_ = ctx->node->create_client<mowgli_interfaces::srv::PlanCoveragePath>(
      "/coverage_planner/plan_coverage_path");
  }

  if (!waitForService(client_, ctx->node)) {
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::PlanCoveragePath::Request>();
  request->area_index = area_index;
  request->mow_angle  = 0.0f;
  request->mow_width  = 0.2f;

  auto future = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
        ctx->node, future, std::chrono::seconds(5))
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "PlanCoveragePath: service call timed out for area_index=%u", area_index);
    return BT::NodeStatus::FAILURE;
  }

  const bool success = future.get()->success;
  if (!success) {
    RCLCPP_WARN(ctx->node->get_logger(),
      "PlanCoveragePath: planner returned failure for area_index=%u", area_index);
  } else {
    RCLCPP_INFO(ctx->node->get_logger(),
      "PlanCoveragePath: planner acknowledged area_index=%u", area_index);
  }

  return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace mowgli_behavior
