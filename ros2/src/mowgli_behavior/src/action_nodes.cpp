#include "mowgli_behavior/action_nodes.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "robot_localization/srv/set_pose.hpp"
#include "slam_toolbox/srv/reset.hpp"
#include "std_srvs/srv/empty.hpp"
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

/// Spin the node's executor for up to timeout_ms to wait for a service.
bool waitForService(rclcpp::ClientBase::SharedPtr client,
                    const rclcpp::Node::SharedPtr& node,
                    int timeout_ms = 1000)
{
  if (client->wait_for_service(std::chrono::milliseconds(timeout_ms)))
  {
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
  if (!res)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "SetMowerEnabled: missing required port 'enabled': %s",
                 res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const bool enabled = res.value();

  if (!client_)
  {
    client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control");
  }

  if (!waitForService(client_, ctx->node))
  {
    // In simulation, no hardware bridge is running — proceed gracefully.
    RCLCPP_WARN(ctx->node->get_logger(),
                "SetMowerEnabled: hardware service unavailable, continuing (simulation mode)");
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  request->mow_enabled = enabled ? 1u : 0u;
  request->mow_direction = 0u;

  // Fire-and-forget: the firmware is the safety authority for the blade.
  // It has its own lift/tilt/emergency checks and will refuse or stop the
  // blade regardless of what ROS2 requests.
  auto future = client_->async_send_request(request);
  (void)future;

  RCLCPP_INFO(ctx->node->get_logger(),
              "SetMowerEnabled: requested mow_enabled=%s",
              enabled ? "true" : "false");

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// StopMoving
// ---------------------------------------------------------------------------

BT::NodeStatus StopMoving::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!pub_)
  {
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

  // Fire-and-forget: send both requests without blocking the BT tick.
  // The main executor spins the node and will deliver the responses
  // asynchronously.  We log a warning if a service is unavailable but
  // still return SUCCESS so the recovery sequence can proceed — a missing
  // costmap service should not permanently block mowing.
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  if (global_client_->service_is_ready())
  {
    global_client_->async_send_request(request);
    RCLCPP_INFO(ctx->node->get_logger(), "ClearCostmap: sent clear request to global costmap");
  }
  else
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "ClearCostmap: global costmap service not ready, skipping");
  }

  if (local_client_->service_is_ready())
  {
    local_client_->async_send_request(request);
    RCLCPP_INFO(ctx->node->get_logger(), "ClearCostmap: sent clear request to local costmap");
  }
  else
  {
    RCLCPP_WARN(ctx->node->get_logger(), "ClearCostmap: local costmap service not ready, skipping");
  }

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// WasRainingAtStart
// ---------------------------------------------------------------------------

BT::NodeStatus WasRainingAtStart::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->raining_at_mow_start = ctx->latest_status.rain_detected;
  RCLCPP_INFO(ctx->node->get_logger(),
              "WasRainingAtStart: rain_at_start=%s",
              ctx->raining_at_mow_start ? "true" : "false");
  return BT::NodeStatus::SUCCESS;
}

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
              ctx->undock_start_x, ctx->undock_start_y);
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
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: no start position recorded");
    return BT::NodeStatus::SUCCESS;  // non-fatal
  }

  // Only calibrate with RTK fix — without it, GPS position is too noisy
  // and the computed heading would be wrong.
  if (!ctx->gps_is_fixed)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: no RTK fix, skipping (would corrupt heading)");
    ctx->undock_start_recorded = false;
    return BT::NodeStatus::SUCCESS;  // non-fatal
  }

  const double dx = ctx->gps_x - ctx->undock_start_x;
  const double dy = ctx->gps_y - ctx->undock_start_y;
  const double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < 0.3)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: displacement too small (%.2f m), skipping", dist);
    return BT::NodeStatus::SUCCESS;  // non-fatal
  }

  // The robot moved BACKWARD, so the heading is OPPOSITE to the displacement vector
  const double heading = std::atan2(-dy, -dx);

  RCLCPP_INFO(ctx->node->get_logger(),
              "CalibrateHeadingFromUndock: displacement=(%.3f, %.3f) dist=%.2f heading=%.1f deg",
              dx, dy, dist, heading * 180.0 / M_PI);

  // Set EKF pose via /set_pose service
  auto client = ctx->node->create_client<robot_localization::srv::SetPose>("/set_pose");
  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: /set_pose service not available");
    return BT::NodeStatus::SUCCESS;  // non-fatal
  }

  auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
  request->pose.header.stamp = ctx->node->now();
  request->pose.header.frame_id = "map";
  request->pose.pose.pose.position.x = ctx->gps_x;
  request->pose.pose.pose.position.y = ctx->gps_y;
  request->pose.pose.pose.orientation.z = std::sin(heading / 2.0);
  request->pose.pose.pose.orientation.w = std::cos(heading / 2.0);
  // Tight covariance for position and yaw
  request->pose.pose.covariance[0] = 0.01;   // x
  request->pose.pose.covariance[7] = 0.01;   // y
  request->pose.pose.covariance[35] = 0.05;  // yaw

  auto future = client->async_send_request(request);
  // Don't wait for result — fire and forget
  (void)future;

  ctx->undock_start_recorded = false;

  // Also reset SLAM so it restarts with correct heading.
  // Set map_start_pose parameter to GPS position + heading, then reset.
  auto param_client = ctx->node->create_client<rcl_interfaces::srv::SetParameters>(
    "/slam_toolbox/set_parameters");
  if (param_client->wait_for_service(std::chrono::seconds(2)))
  {
    auto param_req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter p;
    p.name = "map_start_pose";
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    p.value.double_array_value = {ctx->gps_x, ctx->gps_y, heading};
    param_req->parameters.push_back(p);
    auto param_future = param_client->async_send_request(param_req);
    // Wait briefly for parameter to be set before resetting
    param_future.wait_for(std::chrono::seconds(1));

    RCLCPP_INFO(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: set SLAM map_start_pose to [%.3f, %.3f, %.3f]",
                ctx->gps_x, ctx->gps_y, heading);

    // Now reset SLAM — it will restart from the new map_start_pose
    auto reset_client = ctx->node->create_client<slam_toolbox::srv::Reset>(
      "/slam_toolbox/reset");
    if (reset_client->wait_for_service(std::chrono::seconds(2)))
    {
      auto reset_req = std::make_shared<slam_toolbox::srv::Reset::Request>();
      reset_req->pause_new_measurements = false;
      auto reset_future = reset_client->async_send_request(reset_req);
      (void)reset_future;
      RCLCPP_INFO(ctx->node->get_logger(),
                  "CalibrateHeadingFromUndock: SLAM reset with correct heading");
    }
    else
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "CalibrateHeadingFromUndock: /slam_toolbox/reset not available");
    }
  }
  else
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: /slam_toolbox/set_parameters not available");
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
  if (!state_res)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "PublishHighLevelStatus: missing required port 'state': %s",
                 state_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto name_res = getInput<std::string>("state_name");
  if (!name_res)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "PublishHighLevelStatus: missing required port 'state_name': %s",
                 name_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!pub_)
  {
    pub_ =
        ctx->node->create_publisher<mowgli_interfaces::msg::HighLevelStatus>("~/high_level_status",
                                                                             10);
  }

  mowgli_interfaces::msg::HighLevelStatus msg;
  msg.state = state_res.value();
  msg.state_name = name_res.value();
  msg.sub_state_name = "";
  msg.current_area = static_cast<int16_t>(ctx->current_area);
  msg.current_path = -1;
  msg.current_path_index = -1;
  msg.total_swaths = static_cast<int16_t>(ctx->total_swaths);
  msg.completed_swaths = static_cast<int16_t>(ctx->completed_swaths);
  msg.skipped_swaths = static_cast<int16_t>(ctx->skipped_swaths);
  msg.gps_quality_percent = ctx->gps_quality;
  msg.battery_percent = ctx->battery_percent;
  msg.is_charging = ctx->latest_power.charger_enabled;
  msg.emergency = ctx->latest_emergency.active_emergency;

  pub_->publish(msg);

  RCLCPP_DEBUG(ctx->node->get_logger(),
               "PublishHighLevelStatus: state=%u name='%s'",
               msg.state,
               msg.state_name.c_str());

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// WaitForDuration
// ---------------------------------------------------------------------------

BT::NodeStatus WaitForDuration::onStart()
{
  double duration_sec = 1.0;
  if (auto res = getInput<double>("duration_sec"))
  {
    duration_sec = res.value();
  }

  duration_ = std::chrono::duration<double>(duration_sec);
  start_time_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDuration::onRunning()
{
  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  return elapsed >= duration_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
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

  if (!action_client_->wait_for_action_server(std::chrono::seconds(3)))
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
// PlanCoveragePath
// ---------------------------------------------------------------------------

BT::NodeStatus PlanCoveragePath::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  uint32_t area_index = 0u;
  if (auto res = getInput<uint32_t>("area_index"))
  {
    area_index = res.value();
  }
  area_index_ = area_index;
  result_received_ = false;
  latest_result_.reset();
  goal_handle_.reset();

  // Lazily create the action client once and reuse it across ticks.
  if (!action_client_)
  {
    action_client_ =
        rclcpp_action::create_client<PlanCoverageAction>(ctx->node,
                                                         "/coverage_planner_node/plan_coverage");
  }

  if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "PlanCoveragePath: action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // Build goal: fetch area boundary + obstacles from map_server (single source of truth).
  PlanCoverageAction::Goal goal_msg;
  goal_msg.mow_angle_deg = -1.0;  // auto-optimize
  goal_msg.skip_outline = false;

  {
    auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
    request->index = area_index;

    // Use the shared helper node to avoid "already added to an executor" error
    // (the main behavior_tree_node is already spinning in rclcpp::spin).
    auto helper = ctx->helper_node;
    auto tmp_client = helper->create_client<mowgli_interfaces::srv::GetMowingArea>(
        "/map_server_node/get_mowing_area");
    if (!tmp_client->wait_for_service(std::chrono::milliseconds(2000)))
    {
      RCLCPP_ERROR(
          ctx->node->get_logger(),
          "PlanCoveragePath: map_server get_mowing_area service unavailable — cannot plan");
      return BT::NodeStatus::FAILURE;
    }
    auto future = tmp_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(helper, future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "PlanCoveragePath: get_mowing_area timed out");
      return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response->success)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "PlanCoveragePath: map_server returned no area for index %u",
                   area_index);
      return BT::NodeStatus::FAILURE;
    }

    if (response->area.is_navigation_area)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "PlanCoveragePath: area %u ('%s') is a navigation area — skipping",
                  area_index,
                  response->area.name.c_str());
      return BT::NodeStatus::FAILURE;
    }

    goal_msg.outer_boundary = response->area.area;
    goal_msg.obstacles = response->area.obstacles;

    RCLCPP_INFO(ctx->node->get_logger(),
                "PlanCoveragePath: area '%s' — %zu boundary vertices, %zu obstacles",
                response->area.name.c_str(),
                goal_msg.outer_boundary.points.size(),
                goal_msg.obstacles.size());
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
  send_goal_options.result_callback = [this](const GoalHandle::WrappedResult& wr)
  {
    latest_result_ = wr.result;
    result_received_ = true;
  };

  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(), "PlanCoveragePath: goal sent for area_index=%u", area_index);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanCoveragePath::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Phase 1: wait for the goal to be accepted.
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
                   "PlanCoveragePath: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  // Phase 2: wait for the result callback to fire.
  if (!result_received_)
  {
    const auto status = goal_handle_->get_status();
    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
        status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(), "PlanCoveragePath: goal aborted/canceled by planner");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  // Phase 3: evaluate the result.
  if (!latest_result_ || !latest_result_->success)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "PlanCoveragePath: planner returned failure for area_index=%u",
                area_index_);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(ctx->node->get_logger(),
              "PlanCoveragePath: planner returned path with %zu poses for area_index=%u",
              latest_result_->path.poses.size(),
              area_index_);

  // Write the first waypoint to the output port so NavigateToPose can use it.
  if (!latest_result_->path.poses.empty())
  {
    const auto& first = latest_result_->path.poses.front().pose;
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
                first.position.x,
                first.position.y,
                yaw);
  }

  return BT::NodeStatus::SUCCESS;
}

void PlanCoveragePath::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (goal_handle_)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "PlanCoveragePath: canceling active planning goal");
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
  if (auto res = getInput<std::string>("path_topic"))
  {
    path_topic = res.value();
  }

  path_received_ = false;
  path_sub_.reset();
  path_sub_ = ctx->node->create_subscription<nav_msgs::msg::Path>(
      path_topic,
      rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg)
      {
        full_path_ = *msg;
        path_received_ = true;
      });

  if (!follow_client_)
  {
    follow_client_ = rclcpp_action::create_client<FollowPathAction>(ctx->node, "/follow_path");
  }

  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowCoveragePath: waiting for coverage path on '%s'",
              path_topic.c_str());

  follow_handle_.reset();
  follow_future_ = {};
  phase_ = Phase::WAIT_PATH;
  current_path_index_ = 0;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowCoveragePath::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  switch (phase_)
  {
    // --- Wait for coverage path from planner ---
    case Phase::WAIT_PATH:
    {
      if (!path_received_)
      {
        return BT::NodeStatus::RUNNING;
      }
      if (full_path_.poses.empty())
      {
        RCLCPP_WARN(ctx->node->get_logger(), "FollowCoveragePath: empty path");
        return BT::NodeStatus::FAILURE;
      }
      if (!follow_client_->wait_for_action_server(std::chrono::seconds(5)))
      {
        RCLCPP_WARN(ctx->node->get_logger(), "FollowCoveragePath: /follow_path not available");
        return BT::NodeStatus::FAILURE;
      }

      current_path_index_ = 0;
      sendFollowGoal();
      phase_ = Phase::FOLLOWING;
      return BT::NodeStatus::RUNNING;
    }

    // --- Following coverage path ---
    case Phase::FOLLOWING:
    {
      if (!follow_handle_)
      {
        if (!follow_future_.valid() ||
            follow_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
        {
          return BT::NodeStatus::RUNNING;
        }
        follow_handle_ = follow_future_.get();
        if (!follow_handle_)
        {
          RCLCPP_ERROR(ctx->node->get_logger(), "FollowCoveragePath: FollowPath goal rejected");
          return BT::NodeStatus::FAILURE;
        }
      }

      const auto status = follow_handle_->get_status();
      if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
      {
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

  if (follow_handle_)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "FollowCoveragePath: canceling active follow_path goal");
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
  if (auto res = getInput<std::string>("map_path"))
  {
    map_path = res.value();
  }

  // Lazily create the service client once.
  if (!client_)
  {
    client_ = ctx->node->create_client<SerializeSrv>("/slam_toolbox/serialize_map");
  }

  if (!client_->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SaveSlamMap: /slam_toolbox/serialize_map not available — skipping save");
    // Non-fatal: a missing service should not abort the post-mow sequence.
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<SerializeSrv::Request>();
  request->filename = map_path;

  response_future_ = client_->async_send_request(request).future.share();

  RCLCPP_INFO(ctx->node->get_logger(),
              "SaveSlamMap: serialize_map request sent (path='%s')",
              map_path.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SaveSlamMap::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (response_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    return BT::NodeStatus::RUNNING;
  }

  auto response = response_future_.get();
  if (!response)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "SaveSlamMap: serialize_map returned null response");
    return BT::NodeStatus::FAILURE;
  }

  // result == 0 means RESULT_SUCCESS in slam_toolbox.
  if (response->result != 0)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "SaveSlamMap: serialize_map failed with result code %d",
                 response->result);
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
// ReplanCoverage
// ---------------------------------------------------------------------------

BT::NodeStatus ReplanCoverage::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  result_received_ = false;
  latest_result_.reset();
  goal_handle_.reset();

  if (!action_client_)
  {
    action_client_ =
        rclcpp_action::create_client<PlanCoverageAction>(ctx->node,
                                                         "/coverage_planner_node/plan_coverage");
  }

  if (!action_client_->wait_for_action_server(std::chrono::milliseconds(500)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "ReplanCoverage: action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // Build goal: fetch current area + obstacles from map server (single source of truth).
  PlanCoverageAction::Goal goal_msg;
  goal_msg.mow_angle_deg = -1.0;
  goal_msg.skip_outline = true;  // Faster replanning — skip outline visualization

  {
    auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
    request->index = 0;

    // Use the shared helper node to avoid "already added to an executor" error.
    auto helper = ctx->helper_node;
    auto tmp_client = helper->create_client<mowgli_interfaces::srv::GetMowingArea>(
        "/map_server_node/get_mowing_area");
    if (!tmp_client->wait_for_service(std::chrono::milliseconds(2000)))
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "ReplanCoverage: map_server get_mowing_area service unavailable");
      return BT::NodeStatus::FAILURE;
    }
    auto future = tmp_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(helper, future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "ReplanCoverage: get_mowing_area timed out");
      return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response->success)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "ReplanCoverage: map_server returned no area");
      return BT::NodeStatus::FAILURE;
    }

    goal_msg.outer_boundary = response->area.area;
    goal_msg.obstacles = response->area.obstacles;
    RCLCPP_INFO(ctx->node->get_logger(),
                "ReplanCoverage: area '%s' — %zu obstacles",
                response->area.name.c_str(),
                goal_msg.obstacles.size());
  }

  auto send_goal_options = rclcpp_action::Client<PlanCoverageAction>::SendGoalOptions{};
  send_goal_options.result_callback = [this](const GoalHandle::WrappedResult& wr)
  {
    latest_result_ = wr.result;
    result_received_ = true;
  };

  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(), "ReplanCoverage: goal sent");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReplanCoverage::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "ReplanCoverage: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!result_received_)
  {
    const auto status = goal_handle_->get_status();
    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
        status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(), "ReplanCoverage: planner aborted/canceled");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (!latest_result_ || !latest_result_->success)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "ReplanCoverage: planner returned failure");
    return BT::NodeStatus::FAILURE;
  }

  // Clear the replan flag.
  ctx->replan_needed = false;

  // Write first waypoint to output port.
  if (!latest_result_->path.poses.empty())
  {
    const auto& first = latest_result_->path.poses.front().pose;
    const double siny = 2.0 * (first.orientation.w * first.orientation.z +
                               first.orientation.x * first.orientation.y);
    const double cosy = 1.0 - 2.0 * (first.orientation.y * first.orientation.y +
                                     first.orientation.z * first.orientation.z);
    const double yaw = std::atan2(siny, cosy);

    std::ostringstream oss;
    oss << first.position.x << ";" << first.position.y << ";" << yaw;
    setOutput("first_waypoint", oss.str());
  }

  RCLCPP_INFO(ctx->node->get_logger(),
              "ReplanCoverage: new path with %zu poses ready",
              latest_result_->path.poses.size());

  return BT::NodeStatus::SUCCESS;
}

void ReplanCoverage::onHalted()
{
  if (goal_handle_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_INFO(ctx->node->get_logger(), "ReplanCoverage: halted, goal cancelled");
  }
  goal_handle_.reset();
  result_received_ = false;
  latest_result_.reset();
}

// ---------------------------------------------------------------------------
// SaveObstacles
// ---------------------------------------------------------------------------

BT::NodeStatus SaveObstacles::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!client_)
  {
    client_ = ctx->node->create_client<std_srvs::srv::Trigger>("/obstacle_tracker/save_obstacles");
  }

  if (!client_->service_is_ready())
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SaveObstacles: service unavailable, skipping (no obstacle tracker running)");
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client_->async_send_request(request);

  RCLCPP_INFO(ctx->node->get_logger(), "SaveObstacles: save request sent");
  return BT::NodeStatus::SUCCESS;
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

// ---------------------------------------------------------------------------
// ClearCommand
// ---------------------------------------------------------------------------

BT::NodeStatus ClearCommand::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  RCLCPP_INFO(ctx->node->get_logger(),
              "ClearCommand: resetting current_command from %u to 0",
              ctx->current_command);
  ctx->current_command = 0;
  return BT::NodeStatus::SUCCESS;
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
  goal_msg.time_allowance = rclcpp::Duration::from_seconds(dist / speed + 5.0);

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
// DockRobot
// ---------------------------------------------------------------------------

BT::NodeStatus DockRobot::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  std::string dock_id = "home_dock";
  if (auto res = getInput<std::string>("dock_id"))
  {
    dock_id = res.value();
  }

  std::string dock_type = "simple_charging_dock";
  if (auto res = getInput<std::string>("dock_type"))
  {
    dock_type = res.value();
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<DockAction>(ctx->node, "/dock_robot");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: /dock_robot action server not available");
    return BT::NodeStatus::FAILURE;
  }

  DockAction::Goal goal_msg;
  goal_msg.dock_id = dock_id;
  goal_msg.dock_type = dock_type;
  goal_msg.navigate_to_staging_pose = true;

  auto send_goal_options = rclcpp_action::Client<DockAction>::SendGoalOptions{};
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
              "DockRobot: goal sent (dock_id='%s', dock_type='%s')",
              dock_id.c_str(), dock_type.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockRobot::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "DockRobot: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: docking succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking aborted");
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking canceled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void DockRobot::onHalted()
{
  if (goal_handle_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// UndockRobot
// ---------------------------------------------------------------------------

BT::NodeStatus UndockRobot::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  std::string dock_type = "simple_charging_dock";
  if (auto res = getInput<std::string>("dock_type"))
  {
    dock_type = res.value();
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<UndockAction>(ctx->node, "/undock_robot");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: /undock_robot action server not available");
    return BT::NodeStatus::FAILURE;
  }

  UndockAction::Goal goal_msg;
  goal_msg.dock_type = dock_type;

  auto send_goal_options = rclcpp_action::Client<UndockAction>::SendGoalOptions{};
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
              "UndockRobot: goal sent (dock_type='%s')",
              dock_type.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus UndockRobot::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "UndockRobot: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "UndockRobot: undocking succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: undocking aborted");
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: undocking canceled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void UndockRobot::onHalted()
{
  if (goal_handle_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    RCLCPP_INFO(ctx->node->get_logger(), "UndockRobot: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

}  // namespace mowgli_behavior
