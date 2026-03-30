#include "mowgli_behavior/action_nodes.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"
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
    // In simulation, no hardware bridge is running — proceed gracefully.
    RCLCPP_WARN(ctx->node->get_logger(),
      "SetMowerEnabled: hardware service unavailable, continuing (simulation mode)");
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  request->mow_enabled  = enabled ? 1u : 0u;
  request->mow_direction = 0u;

  // Fire-and-forget: send the request asynchronously without blocking.
  // The node is already being spun by the main executor, so
  // spin_until_future_complete would throw.
  client_->async_send_request(request);

  RCLCPP_INFO(ctx->node->get_logger(),
    "SetMowerEnabled: mow_enabled set to %s", enabled ? "true" : "false");

  return BT::NodeStatus::SUCCESS;
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
// ClearCostmap
// ---------------------------------------------------------------------------

BT::NodeStatus ClearCostmap::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Lazily create service clients once and reuse across ticks.
  if (!global_client_) {
    global_client_ = ctx->node->create_client<std_srvs::srv::Empty>(
      "/global_costmap/clear_entirely_global_costmap");
  }
  if (!local_client_) {
    local_client_ = ctx->node->create_client<std_srvs::srv::Empty>(
      "/local_costmap/clear_entirely_local_costmap");
  }

  // Fire-and-forget: send both requests without blocking the BT tick.
  // The main executor spins the node and will deliver the responses
  // asynchronously.  We log a warning if a service is unavailable but
  // still return SUCCESS so the recovery sequence can proceed — a missing
  // costmap service should not permanently block mowing.
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  if (global_client_->service_is_ready()) {
    global_client_->async_send_request(request);
    RCLCPP_INFO(ctx->node->get_logger(),
      "ClearCostmap: sent clear request to global costmap");
  } else {
    RCLCPP_WARN(ctx->node->get_logger(),
      "ClearCostmap: global costmap service not ready, skipping");
  }

  if (local_client_->service_is_ready()) {
    local_client_->async_send_request(request);
    RCLCPP_INFO(ctx->node->get_logger(),
      "ClearCostmap: sent clear request to local costmap");
  } else {
    RCLCPP_WARN(ctx->node->get_logger(),
      "ClearCostmap: local costmap service not ready, skipping");
  }

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

BT::NodeStatus PlanCoveragePath::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  uint32_t area_index = 0u;
  if (auto res = getInput<uint32_t>("area_index")) {
    area_index = res.value();
  }
  area_index_       = area_index;
  result_received_  = false;
  latest_result_.reset();
  goal_handle_.reset();

  // Lazily create the action client once and reuse it across ticks.
  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<PlanCoverageAction>(
      ctx->node, "/coverage_planner_node/plan_coverage");
  }

  if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(ctx->node->get_logger(),
      "PlanCoveragePath: action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // Build goal.
  PlanCoverageAction::Goal goal_msg;
  goal_msg.mow_angle_deg = -1.0;  // auto-optimize
  goal_msg.skip_outline  = false;

  // Try reading the mowing boundary from the blackboard / port.
  std::string boundary_str;
  if (auto res = getInput<std::string>("boundary")) {
    boundary_str = res.value();
  }
  if (boundary_str.empty()) {
    auto entry = config().blackboard->getEntry("mowing_boundary");
    if (entry) {
      boundary_str = entry->value.cast<std::string>();
    }
  }

  if (boundary_str.empty()) {
    // Default simulation area inset 1.0m from garden walls (20x15m garden).
    // Inset = robot_radius (0.22m) + inflation_radius (0.3m) + 0.48m margin
    // so the coverage path never drives into inflated costmap zones near walls.
    geometry_msgs::msg::Point32 p;
    p.z = 0.0f;
    p.x = -7.0f;  p.y = -6.5f; goal_msg.outer_boundary.points.push_back(p);
    p.x = 11.0f;  p.y = -6.5f; goal_msg.outer_boundary.points.push_back(p);
    p.x = 11.0f;  p.y =  6.5f; goal_msg.outer_boundary.points.push_back(p);
    p.x = -7.0f;  p.y =  6.5f; goal_msg.outer_boundary.points.push_back(p);
    RCLCPP_INFO(ctx->node->get_logger(),
      "PlanCoveragePath: using default 18x13m simulation boundary (1.0m inset from walls)");
  }

  // Fetch obstacles from the map server so F2C can plan around them.
  {
    auto get_area_client = ctx->node->create_client<mowgli_interfaces::srv::GetMowingArea>(
      "/map_server/get_mowing_area");
    if (get_area_client->wait_for_service(std::chrono::milliseconds(500))) {
      auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
      request->index = area_index;
      auto future = get_area_client->async_send_request(request);
      if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        auto response = future.get();
        if (response->success) {
          goal_msg.obstacles = response->area.obstacles;
          RCLCPP_INFO(ctx->node->get_logger(),
            "PlanCoveragePath: fetched %zu obstacles from map server",
            goal_msg.obstacles.size());
        }
      }
    } else {
      RCLCPP_WARN(ctx->node->get_logger(),
        "PlanCoveragePath: map_server get_mowing_area service unavailable, "
        "planning without obstacles");
    }
  }

  // Send the goal asynchronously.
  auto send_goal_options = rclcpp_action::Client<PlanCoverageAction>::SendGoalOptions{};

  // Feedback callback: log progress.
  send_goal_options.feedback_callback =
    [ctx](GoalHandle::SharedPtr /*gh*/,
          const std::shared_ptr<const PlanCoverageAction::Feedback> fb)
    {
      RCLCPP_DEBUG(ctx->node->get_logger(),
        "PlanCoveragePath feedback: %.0f%% [%s]",
        static_cast<double>(fb->progress_percent),
        fb->phase.c_str());
    };

  // Result callback: store result and signal readiness.
  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult & wr)
    {
      latest_result_  = wr.result;
      result_received_ = true;
    };

  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
    "PlanCoveragePath: goal sent for area_index=%u", area_index);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanCoveragePath::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Phase 1: wait for the goal to be accepted.
  if (!goal_handle_) {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0))
        != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(ctx->node->get_logger(),
        "PlanCoveragePath: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  // Phase 2: wait for the result callback to fire.
  if (!result_received_) {
    const auto status = goal_handle_->get_status();
    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
        status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
        "PlanCoveragePath: goal aborted/canceled by planner");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  // Phase 3: evaluate the result.
  if (!latest_result_ || !latest_result_->success) {
    RCLCPP_WARN(ctx->node->get_logger(),
      "PlanCoveragePath: planner returned failure for area_index=%u", area_index_);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ctx->node->get_logger(),
    "PlanCoveragePath: planner returned path with %zu poses for area_index=%u",
    latest_result_->path.poses.size(), area_index_);

  // Write the first waypoint to the output port so NavigateToPose can use it.
  if (!latest_result_->path.poses.empty()) {
    const auto & first = latest_result_->path.poses.front().pose;
    const double siny = 2.0 * (first.orientation.w * first.orientation.z +
                                first.orientation.x * first.orientation.y);
    const double cosy = 1.0 - 2.0 * (first.orientation.y * first.orientation.y +
                                       first.orientation.z * first.orientation.z);
    const double yaw = std::atan2(siny, cosy);

    std::ostringstream oss;
    oss << first.position.x << ";" << first.position.y << ";" << yaw;
    setOutput("first_waypoint", oss.str());

    RCLCPP_INFO(ctx->node->get_logger(),
      "PlanCoveragePath: first_waypoint = (%.2f, %.2f, yaw=%.2f)",
      first.position.x, first.position.y, yaw);
  }

  return BT::NodeStatus::SUCCESS;
}

void PlanCoveragePath::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (goal_handle_) {
    RCLCPP_INFO(ctx->node->get_logger(),
      "PlanCoveragePath: canceling active planning goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
  result_received_ = false;
  latest_result_.reset();
}

// ---------------------------------------------------------------------------
// FollowCoveragePath — simple MPPI path following
// ---------------------------------------------------------------------------
//
// Sends the coverage path to the MPPI controller via FollowPath action.
// MPPI naturally steers around obstacles detected in the costmap while
// staying close to the path.  The keepout filter prevents the robot from
// leaving the mowing area.  If the controller aborts (e.g. progress checker
// timeout), this node returns FAILURE and lets the BT recovery handle it.

void FollowCoveragePath::sendFollowGoal()
{
  FollowPathAction::Goal goal_msg;
  goal_msg.path = full_path_;
  goal_msg.controller_id = "FollowCoveragePath";
  goal_msg.goal_checker_id = "coverage_goal_checker";

  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  RCLCPP_INFO(ctx->node->get_logger(),
    "FollowCoveragePath: sending full %zu-pose path to RPP controller",
    full_path_.poses.size());

  auto opts = rclcpp_action::Client<FollowPathAction>::SendGoalOptions{};
  follow_handle_.reset();
  follow_future_ = follow_client_->async_send_goal(goal_msg, opts);
}

BT::NodeStatus FollowCoveragePath::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  std::string path_topic = "/coverage_planner_node/coverage_path";
  if (auto res = getInput<std::string>("path_topic")) {
    path_topic = res.value();
  }

  path_received_ = false;
  path_sub_.reset();
  path_sub_ = ctx->node->create_subscription<nav_msgs::msg::Path>(
    path_topic, rclcpp::QoS(1).transient_local(),
    [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
      full_path_ = *msg;
      path_received_ = true;
    });

  if (!follow_client_) {
    follow_client_ = rclcpp_action::create_client<FollowPathAction>(
      ctx->node, "/follow_path");
  }

  RCLCPP_INFO(ctx->node->get_logger(),
    "FollowCoveragePath: waiting for coverage path on '%s'", path_topic.c_str());

  follow_handle_.reset();
  follow_future_ = {};
  phase_ = Phase::WAIT_PATH;
  current_path_index_ = 0;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowCoveragePath::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  switch (phase_) {

  // --- Wait for coverage path from planner ---
  case Phase::WAIT_PATH: {
    if (!path_received_) {
      return BT::NodeStatus::RUNNING;
    }
    if (full_path_.poses.empty()) {
      RCLCPP_WARN(ctx->node->get_logger(), "FollowCoveragePath: empty path");
      return BT::NodeStatus::FAILURE;
    }
    if (!follow_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN(ctx->node->get_logger(),
        "FollowCoveragePath: /follow_path not available");
      return BT::NodeStatus::FAILURE;
    }

    current_path_index_ = 0;
    sendFollowGoal();
    phase_ = Phase::FOLLOWING;
    return BT::NodeStatus::RUNNING;
  }

  // --- Following coverage path ---
  case Phase::FOLLOWING: {
    if (!follow_handle_) {
      if (!follow_future_.valid() ||
          follow_future_.wait_for(std::chrono::milliseconds(0))
            != std::future_status::ready)
      {
        return BT::NodeStatus::RUNNING;
      }
      follow_handle_ = follow_future_.get();
      if (!follow_handle_) {
        RCLCPP_ERROR(ctx->node->get_logger(),
          "FollowCoveragePath: FollowPath goal rejected");
        return BT::NodeStatus::FAILURE;
      }
    }

    const auto status = follow_handle_->get_status();
    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      RCLCPP_INFO(ctx->node->get_logger(),
        "FollowCoveragePath: coverage path completed (%zu poses)",
        full_path_.poses.size());
      return BT::NodeStatus::SUCCESS;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
        status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
        "FollowCoveragePath: controller aborted — returning FAILURE for BT recovery");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  default:
    return BT::NodeStatus::FAILURE;
  }
}

void FollowCoveragePath::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (follow_handle_) {
    RCLCPP_INFO(ctx->node->get_logger(),
      "FollowCoveragePath: canceling active follow_path goal");
    follow_client_->async_cancel_goal(follow_handle_);
    follow_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// SaveSlamMap
// ---------------------------------------------------------------------------

BT::NodeStatus SaveSlamMap::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  std::string map_path = "/ros2_ws/maps/garden_map";
  if (auto res = getInput<std::string>("map_path")) {
    map_path = res.value();
  }

  // Lazily create the service client once.
  if (!client_) {
    client_ = ctx->node->create_client<SerializeSrv>(
      "/slam_toolbox/serialize_map");
  }

  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(ctx->node->get_logger(),
      "SaveSlamMap: /slam_toolbox/serialize_map not available — skipping save");
    // Non-fatal: a missing service should not abort the post-mow sequence.
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<SerializeSrv::Request>();
  request->filename = map_path;

  response_future_ = client_->async_send_request(request).future.share();

  RCLCPP_INFO(ctx->node->get_logger(),
    "SaveSlamMap: serialize_map request sent (path='%s')", map_path.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SaveSlamMap::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (response_future_.wait_for(std::chrono::milliseconds(0))
      != std::future_status::ready)
  {
    return BT::NodeStatus::RUNNING;
  }

  auto response = response_future_.get();
  if (!response) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "SaveSlamMap: serialize_map returned null response");
    return BT::NodeStatus::FAILURE;
  }

  // result == 0 means RESULT_SUCCESS in slam_toolbox.
  if (response->result != 0) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "SaveSlamMap: serialize_map failed with result code %d", response->result);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ctx->node->get_logger(), "SaveSlamMap: map saved successfully");
  return BT::NodeStatus::SUCCESS;
}

void SaveSlamMap::onHalted()
{
  // The future cannot be cancelled once sent; release the handle so the
  // response is discarded if it arrives after the node is halted.
  response_future_ = {};
}

// ---------------------------------------------------------------------------
// ClearCommand
// ---------------------------------------------------------------------------

BT::NodeStatus ClearCommand::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  RCLCPP_INFO(ctx->node->get_logger(),
    "ClearCommand: resetting current_command from %u to 0", ctx->current_command);
  ctx->current_command = 0;
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// BackUp
// ---------------------------------------------------------------------------

BT::NodeStatus BackUp::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<BackUpAction>(
      ctx->node, "/backup");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
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
  goal_msg.time_allowance = rclcpp::Duration::from_seconds(dist / speed + 5.0);

  RCLCPP_INFO(ctx->node->get_logger(),
    "BackUp: reversing %.2fm at %.2f m/s", dist, speed);

  goal_handle_future_ = action_client_->async_send_goal(goal_msg);
  goal_handle_ = nullptr;
  result_requested_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BackUp::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Wait for goal acceptance
  if (!goal_handle_) {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(ctx->node->get_logger(), "BackUp: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  // Request result future only once
  if (!result_requested_) {
    result_future_ = action_client_->async_get_result(goal_handle_);
    result_requested_ = true;
  }

  if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  auto wrapped = result_future_.get();
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(ctx->node->get_logger(), "BackUp: complete");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(ctx->node->get_logger(), "BackUp: action ended with code %d",
    static_cast<int>(wrapped.code));
  return BT::NodeStatus::FAILURE;
}

void BackUp::onHalted()
{
  if (goal_handle_) {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_INFO(ctx->node->get_logger(), "BackUp: halted, goal cancelled");
  }
  goal_handle_ = nullptr;
}

}  // namespace mowgli_behavior
