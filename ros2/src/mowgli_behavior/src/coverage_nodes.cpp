// Copyright 2026 Mowgli Project
//
// Licensed under the GNU General Public License, version 3 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.html

#include "mowgli_behavior/coverage_nodes.hpp"

#include <cmath>
#include <sstream>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_behavior
{

// ===========================================================================
// ComputeCoverage — calls coverage_planner_node's PlanCoverage action
// ===========================================================================

BT::NodeStatus ComputeCoverage::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  uint32_t area_index = 0u;
  getInput<uint32_t>("area_index", area_index);

  result_received_ = false;
  latest_result_.reset();
  goal_handle_.reset();

  if (!action_client_) {
    action_client_ =
      rclcpp_action::create_client<CoverageAction>(
      ctx->node, "/mowgli/coverage/plan");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "ComputeCoverage: plan_coverage action not available");
    return BT::NodeStatus::FAILURE;
  }

  // Fetch mowing area boundary from map_server.
  CoverageAction::Goal goal_msg;

  {
    auto tmp_node = rclcpp::Node::make_shared("_compute_coverage_srv_helper");
    auto tmp_client = tmp_node->create_client<mowgli_interfaces::srv::GetMowingArea>(
      "/mowgli/map/get_area");

    if (!tmp_client->wait_for_service(std::chrono::milliseconds(2000))) {
      RCLCPP_ERROR(ctx->node->get_logger(),
        "ComputeCoverage: get_mowing_area service unavailable");
      return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
    request->index = area_index;

    auto future = tmp_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(tmp_node, future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "ComputeCoverage: get_mowing_area timed out");
      return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(ctx->node->get_logger(),
        "ComputeCoverage: map_server returned no area for index %u", area_index);
      return BT::NodeStatus::FAILURE;
    }

    if (response->area.is_navigation_area) {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ComputeCoverage: area %u is navigation-only, skipping", area_index);
      return BT::NodeStatus::FAILURE;
    }

    // Set boundary polygon directly (PlanCoverage uses geometry_msgs/Polygon).
    goal_msg.outer_boundary = response->area.area;

    // Add obstacle polygons.
    goal_msg.obstacles = response->area.obstacles;

    // Use default mow angle (auto-optimize).
    goal_msg.mow_angle_deg = -1.0;
    goal_msg.skip_outline = false;

    RCLCPP_INFO(ctx->node->get_logger(),
      "ComputeCoverage: area '%s' — %zu boundary pts, %zu obstacles",
      response->area.name.c_str(),
      response->area.area.points.size(),
      response->area.obstacles.size());
  }

  // Send goal.
  auto opts = rclcpp_action::Client<CoverageAction>::SendGoalOptions{};
  opts.result_callback = [this](const GoalHandle::WrappedResult & wr) {
      latest_result_ = wr.result;
      result_received_ = true;
    };

  goal_handle_future_ = action_client_->async_send_goal(goal_msg, opts);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(), "ComputeCoverage: goal sent to plan_coverage");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputeCoverage::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Wait for goal acceptance.
  if (!goal_handle_) {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(ctx->node->get_logger(), "ComputeCoverage: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  // Wait for result.
  if (!result_received_) {
    const auto status = goal_handle_->get_status();
    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(), "ComputeCoverage: planner aborted/canceled");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  // Evaluate result.
  if (!latest_result_ || !latest_result_->success) {
    RCLCPP_WARN(ctx->node->get_logger(),
      "ComputeCoverage: planner failed — %s",
      latest_result_ ? latest_result_->message.c_str() : "no result");
    return BT::NodeStatus::FAILURE;
  }

  // Extract swath endpoints into BTContext::CoveragePlan.
  BTContext::CoveragePlan plan;

  const auto & starts = latest_result_->swath_starts;
  const auto & ends = latest_result_->swath_ends;
  const size_t n_swaths = std::min(starts.size(), ends.size());

  for (size_t i = 0; i < n_swaths; ++i) {
    BTContext::Swath s;
    s.start.x = starts[i].x;
    s.start.y = starts[i].y;
    s.end.x = ends[i].x;
    s.end.y = ends[i].y;
    plan.swaths.push_back(s);
  }

  RCLCPP_INFO(ctx->node->get_logger(),
    "ComputeCoverage: received %zu swaths, %.1f m path, %.2f m2 area",
    plan.swaths.size(),
    latest_result_->total_distance,
    latest_result_->coverage_area);

  if (plan.swaths.empty()) {
    RCLCPP_ERROR(ctx->node->get_logger(), "ComputeCoverage: empty plan");
    return BT::NodeStatus::FAILURE;
  }

  // Store in context.
  ctx->coverage_plan = std::move(plan);
  ctx->next_swath_index = 0;

  // Output first swath start pose.
  const auto & first = ctx->coverage_plan->swaths.front();
  const double dx = first.end.x - first.start.x;
  const double dy = first.end.y - first.start.y;
  const double yaw = std::atan2(dy, dx);

  std::ostringstream oss;
  oss << first.start.x << ";" << first.start.y << ";" << yaw;
  setOutput("first_swath_start", oss.str());

  RCLCPP_INFO(ctx->node->get_logger(),
    "ComputeCoverage: first swath start (%.2f, %.2f, yaw=%.2f)",
    first.start.x, first.start.y, yaw);

  return BT::NodeStatus::SUCCESS;
}

void ComputeCoverage::onHalted()
{
  if (goal_handle_) {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_INFO(ctx->node->get_logger(), "ComputeCoverage: halted, goal cancelled");
  }
  goal_handle_.reset();
  result_received_ = false;
  latest_result_.reset();
}

// ===========================================================================
// ExecuteSwathBySwath (unchanged — works with BTContext::CoveragePlan)
// ===========================================================================

nav_msgs::msg::Path ExecuteSwathBySwath::swathToPath(
  const BTContext::Swath & swath, const rclcpp::Node::SharedPtr & node) const
{
  const double dx = swath.end.x - swath.start.x;
  const double dy = swath.end.y - swath.start.y;
  const double length = std::hypot(dx, dy);
  const double yaw = std::atan2(dy, dx);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  const auto quat = tf2::toMsg(q);

  nav_msgs::msg::Path path;
  path.header.stamp = node->now();
  path.header.frame_id = "map";

  const double spacing = 0.10;
  const int num_points = std::max(2, static_cast<int>(std::ceil(length / spacing)) + 1);

  for (int i = 0; i < num_points; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(num_points - 1);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = swath.start.x + t * dx;
    pose.pose.position.y = swath.start.y + t * dy;
    pose.pose.orientation = quat;
    path.poses.push_back(pose);
  }

  return path;
}

void ExecuteSwathBySwath::sendTransitGoal(const BTContext::Swath & swath)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Always use NavigateToPose for transits — it plans obstacle-aware paths.
  // F2C generates the complete coverage path with Dubins turns for
  // visualization, but execution uses Nav2 for obstacle avoidance.
  Nav2Navigate::Goal goal;
  goal.pose.header.stamp = ctx->node->now();
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = swath.start.x;
  goal.pose.pose.position.y = swath.start.y;

  const double dx = swath.end.x - swath.start.x;
  const double dy = swath.end.y - swath.start.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, std::atan2(dy, dx));
  goal.pose.pose.orientation = tf2::toMsg(q);

  nav_handle_.reset();
  nav_future_ = nav_client_->async_send_goal(goal);
  use_follow_for_transit_ = false;

  RCLCPP_INFO(ctx->node->get_logger(),
    "ExecuteSwathBySwath: transit to swath %zu/%zu start (%.2f, %.2f)",
    swath_index_ + 1, total_swaths_, swath.start.x, swath.start.y);
}

void ExecuteSwathBySwath::sendSwathGoal(const BTContext::Swath & swath)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  Nav2FollowPath::Goal goal;
  goal.path = swathToPath(swath, ctx->node);
  goal.controller_id = "FollowCoveragePath";
  goal.goal_checker_id = "coverage_goal_checker";

  follow_handle_.reset();
  follow_future_ = follow_client_->async_send_goal(goal);

  const double len = std::hypot(swath.end.x - swath.start.x, swath.end.y - swath.start.y);
  RCLCPP_INFO(ctx->node->get_logger(),
    "ExecuteSwathBySwath: mowing swath %zu/%zu (%.1fm)",
    swath_index_ + 1, total_swaths_, len);
}

nav_msgs::msg::Path ExecuteSwathBySwath::remainingSwathPath(
  const BTContext::Swath & swath, double from_x, double from_y,
  const rclcpp::Node::SharedPtr & node) const
{
  // Project (from_x, from_y) onto the swath line to find the resume point.
  const double dx = swath.end.x - swath.start.x;
  const double dy = swath.end.y - swath.start.y;
  const double seg_len_sq = dx * dx + dy * dy;
  double t = 0.0;
  if (seg_len_sq > 1e-12) {
    t = ((from_x - swath.start.x) * dx + (from_y - swath.start.y) * dy) / seg_len_sq;
    t = std::max(0.0, std::min(1.0, t));
  }

  const double yaw = std::atan2(dy, dx);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  const auto quat = tf2::toMsg(q);

  nav_msgs::msg::Path path;
  path.header.stamp = node->now();
  path.header.frame_id = "map";

  const double remaining_len = std::hypot(dx, dy) * (1.0 - t);
  const double spacing = 0.10;
  const int num_points = std::max(2, static_cast<int>(std::ceil(remaining_len / spacing)) + 1);

  for (int i = 0; i < num_points; ++i) {
    const double frac = t + (1.0 - t) * static_cast<double>(i) / static_cast<double>(num_points - 1);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = swath.start.x + frac * dx;
    pose.pose.position.y = swath.start.y + frac * dy;
    pose.pose.orientation = quat;
    path.poses.push_back(pose);
  }

  return path;
}

void ExecuteSwathBySwath::sendRerouteGoal(
  const BTContext::Swath & swath, double rx, double ry)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Find a waypoint past the obstacle: project current position onto swath,
  // then advance by 1.5m (enough to clear a typical obstacle + inflation).
  const double dx = swath.end.x - swath.start.x;
  const double dy = swath.end.y - swath.start.y;
  const double seg_len = std::hypot(dx, dy);
  double t = 0.0;
  if (seg_len > 1e-6) {
    t = ((rx - swath.start.x) * dx + (ry - swath.start.y) * dy) / (seg_len * seg_len);
    t = std::max(0.0, std::min(1.0, t));
  }

  // Advance 1.5m past current position along the swath
  const double advance = 1.5;
  const double t_advance = t + advance / std::max(seg_len, 0.01);
  const double t_target = std::min(t_advance, 1.0);

  Nav2Navigate::Goal goal;
  goal.pose.header.stamp = ctx->node->now();
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = swath.start.x + t_target * dx;
  goal.pose.pose.position.y = swath.start.y + t_target * dy;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, std::atan2(dy, dx));
  goal.pose.pose.orientation = tf2::toMsg(q);

  nav_handle_.reset();
  nav_future_ = nav_client_->async_send_goal(goal);
  blocked_swath_fraction_ = t_target;

  RCLCPP_INFO(ctx->node->get_logger(),
    "ExecuteSwathBySwath: rerouting around obstacle on swath %zu/%zu "
    "(from %.2f,%.2f to %.2f,%.2f via planner)",
    swath_index_ + 1, total_swaths_, rx, ry,
    goal.pose.pose.position.x, goal.pose.pose.position.y);
}

void ExecuteSwathBySwath::sendRemainingSwathGoal(
  const BTContext::Swath & swath, double from_x, double from_y)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  Nav2FollowPath::Goal goal;
  goal.path = remainingSwathPath(swath, from_x, from_y, ctx->node);
  goal.controller_id = "FollowCoveragePath";
  goal.goal_checker_id = "coverage_goal_checker";

  follow_handle_.reset();
  follow_future_ = follow_client_->async_send_goal(goal);

  RCLCPP_INFO(ctx->node->get_logger(),
    "ExecuteSwathBySwath: resuming swath %zu/%zu from (%.2f, %.2f)",
    swath_index_ + 1, total_swaths_, from_x, from_y);
}

void ExecuteSwathBySwath::setBladeEnabled(bool enabled)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!blade_client_) {
    blade_client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
      "/mowgli/hardware/mower_control");
  }

  if (!blade_client_->service_is_ready()) {
    RCLCPP_WARN(ctx->node->get_logger(),
      "ExecuteSwathBySwath: blade service unavailable (sim mode)");
    return;
  }

  auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  req->mow_enabled = enabled ? 1u : 0u;
  req->mow_direction = 0u;
  blade_client_->async_send_request(req);
}

bool ExecuteSwathBySwath::advanceToNextSwath()
{
  swath_index_++;
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->next_swath_index = swath_index_;
  return swath_index_ < total_swaths_;
}

bool ExecuteSwathBySwath::checkStuck(const std::shared_ptr<BTContext> & ctx)
{
  double rx = 0.0, ry = 0.0;
  try {
    auto transform = ctx->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    rx = transform.transform.translation.x;
    ry = transform.transform.translation.y;
  } catch (const tf2::TransformException &) {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  const double dx = rx - last_progress_x_;
  const double dy = ry - last_progress_y_;
  const double dist = std::hypot(dx, dy);

  if (dist >= stuck_min_progress_) {
    last_progress_x_ = rx;
    last_progress_y_ = ry;
    last_progress_time_ = now;
    return false;
  }

  const double elapsed = std::chrono::duration<double>(now - last_progress_time_).count();
  return elapsed >= stuck_timeout_sec_;
}

BT::NodeStatus ExecuteSwathBySwath::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!ctx->coverage_plan || ctx->coverage_plan->swaths.empty()) {
    RCLCPP_ERROR(ctx->node->get_logger(), "ExecuteSwathBySwath: no coverage plan in context");
    return BT::NodeStatus::FAILURE;
  }

  total_swaths_ = ctx->coverage_plan->swaths.size();
  swath_index_ = ctx->next_swath_index;
  completed_swaths_ = 0;
  skipped_swaths_ = 0;

  if (swath_index_ >= total_swaths_) {
    RCLCPP_INFO(ctx->node->get_logger(),
      "ExecuteSwathBySwath: all %zu swaths already completed", total_swaths_);
    return BT::NodeStatus::SUCCESS;
  }

  if (!nav_client_) {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }
  if (!follow_client_) {
    follow_client_ = rclcpp_action::create_client<Nav2FollowPath>(ctx->node, "/follow_path");
  }

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(ctx->node->get_logger(), "ExecuteSwathBySwath: /navigate_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(ctx->node->get_logger(), "ExecuteSwathBySwath: /follow_path not available");
    return BT::NodeStatus::FAILURE;
  }

  setBladeEnabled(false);
  phase_ = Phase::TRANSIT_TO_SWATH;
  reroute_attempts_ = 0;
  blocked_swath_fraction_ = 0.0;
  last_progress_time_ = std::chrono::steady_clock::now();
  last_progress_x_ = 0.0;
  last_progress_y_ = 0.0;

  const auto & swath = ctx->coverage_plan->swaths[swath_index_];
  sendTransitGoal(swath);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteSwathBySwath::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  const auto now = std::chrono::steady_clock::now();

  if (phase_ == Phase::DONE) {
    return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // Cooldown after transit failure.
  if (now < transit_cooldown_until_) {
    return BT::NodeStatus::RUNNING;
  }

  const auto & swath = ctx->coverage_plan->swaths[swath_index_];

  if (phase_ == Phase::TRANSIT_TO_SWATH) {
    // Wait for goal acceptance.
    if (!nav_handle_) {
      if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
      }
      nav_handle_ = nav_future_.get();
      if (!nav_handle_) {
        RCLCPP_WARN(ctx->node->get_logger(),
          "ExecuteSwathBySwath: transit goal rejected for swath %zu", swath_index_ + 1);
        skipped_swaths_++;
        if (!advanceToNextSwath()) {
          phase_ = Phase::DONE;
          return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        transit_cooldown_until_ = now + std::chrono::seconds(3);
        sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
        return BT::NodeStatus::RUNNING;
      }
      last_progress_time_ = now;
    }

    auto status = nav_handle_->get_status();

    if (checkStuck(ctx)) {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: stuck during transit to swath %zu, skipping", swath_index_ + 1);
      nav_client_->async_cancel_goal(nav_handle_);
      nav_handle_.reset();
      skipped_swaths_++;
      if (!advanceToNextSwath()) {
        phase_ = Phase::DONE;
        return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      transit_cooldown_until_ = now + std::chrono::seconds(3);
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      RCLCPP_INFO(ctx->node->get_logger(),
        "ExecuteSwathBySwath: arrived at swath %zu start", swath_index_ + 1);
      nav_handle_.reset();
      setBladeEnabled(true);
      phase_ = Phase::MOWING_SWATH;
      last_progress_time_ = now;
      sendSwathGoal(swath);
      return BT::NodeStatus::RUNNING;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: transit failed for swath %zu", swath_index_ + 1);
      nav_handle_.reset();
      skipped_swaths_++;
      if (!advanceToNextSwath()) {
        phase_ = Phase::DONE;
        return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      transit_cooldown_until_ = now + std::chrono::seconds(3);
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::MOWING_SWATH) {
    if (!follow_handle_) {
      if (follow_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
      }
      follow_handle_ = follow_future_.get();
      if (!follow_handle_) {
        RCLCPP_WARN(ctx->node->get_logger(),
          "ExecuteSwathBySwath: follow goal rejected for swath %zu", swath_index_ + 1);
        setBladeEnabled(false);
        skipped_swaths_++;
        if (!advanceToNextSwath()) {
          phase_ = Phase::DONE;
          return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        phase_ = Phase::TRANSIT_TO_SWATH;
        sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
        return BT::NodeStatus::RUNNING;
      }
      last_progress_time_ = now;
    }

    auto status = follow_handle_->get_status();

    if (checkStuck(ctx)) {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: stuck during swath %zu (obstacle?)", swath_index_ + 1);
      follow_client_->async_cancel_goal(follow_handle_);
      follow_handle_.reset();
      setBladeEnabled(false);

      // Try rerouting around the obstacle using NavigateToPose (planner-aware).
      if (reroute_attempts_ < max_reroute_attempts_) {
        reroute_attempts_++;
        double rx = last_progress_x_;
        double ry = last_progress_y_;
        try {
          auto tf = ctx->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
          rx = tf.transform.translation.x;
          ry = tf.transform.translation.y;
        } catch (const tf2::TransformException &) {}

        RCLCPP_INFO(ctx->node->get_logger(),
          "ExecuteSwathBySwath: rerouting attempt %zu/%zu around obstacle on swath %zu",
          reroute_attempts_, max_reroute_attempts_, swath_index_ + 1);
        phase_ = Phase::REROUTING_AROUND_OBSTACLE;
        last_progress_time_ = now;
        sendRerouteGoal(ctx->coverage_plan->swaths[swath_index_], rx, ry);
        return BT::NodeStatus::RUNNING;
      }

      // Rerouting exhausted — skip this swath.
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: reroute attempts exhausted, skipping swath %zu", swath_index_ + 1);
      skipped_swaths_++;
      reroute_attempts_ = 0;
      if (!advanceToNextSwath()) {
        phase_ = Phase::DONE;
        return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      phase_ = Phase::TRANSIT_TO_SWATH;
      transit_cooldown_until_ = now + std::chrono::seconds(3);
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      RCLCPP_INFO(ctx->node->get_logger(),
        "ExecuteSwathBySwath: swath %zu/%zu completed", swath_index_ + 1, total_swaths_);
      follow_handle_.reset();
      setBladeEnabled(false);
      completed_swaths_++;
      reroute_attempts_ = 0;

      if (!advanceToNextSwath()) {
        RCLCPP_INFO(ctx->node->get_logger(),
          "ExecuteSwathBySwath: all swaths done (%zu completed, %zu skipped)",
          completed_swaths_, skipped_swaths_);
        phase_ = Phase::DONE;
        return BT::NodeStatus::SUCCESS;
      }

      phase_ = Phase::TRANSIT_TO_SWATH;
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: swath %zu follow aborted/canceled", swath_index_ + 1);
      follow_handle_.reset();
      setBladeEnabled(false);

      // Attempt reroute before skipping.
      if (reroute_attempts_ < max_reroute_attempts_) {
        reroute_attempts_++;
        double rx = last_progress_x_;
        double ry = last_progress_y_;
        try {
          auto tf = ctx->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
          rx = tf.transform.translation.x;
          ry = tf.transform.translation.y;
        } catch (const tf2::TransformException &) {}

        phase_ = Phase::REROUTING_AROUND_OBSTACLE;
        last_progress_time_ = now;
        sendRerouteGoal(ctx->coverage_plan->swaths[swath_index_], rx, ry);
        return BT::NodeStatus::RUNNING;
      }

      skipped_swaths_++;
      reroute_attempts_ = 0;
      if (!advanceToNextSwath()) {
        phase_ = Phase::DONE;
        return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      phase_ = Phase::TRANSIT_TO_SWATH;
      transit_cooldown_until_ = now + std::chrono::seconds(3);
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
  }

  // ── REROUTING_AROUND_OBSTACLE: NavigateToPose around obstacle, then resume swath ──
  if (phase_ == Phase::REROUTING_AROUND_OBSTACLE) {
    if (!nav_handle_) {
      if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
      }
      nav_handle_ = nav_future_.get();
      if (!nav_handle_) {
        RCLCPP_WARN(ctx->node->get_logger(),
          "ExecuteSwathBySwath: reroute goal rejected for swath %zu", swath_index_ + 1);
        // Fall back to skipping.
        skipped_swaths_++;
        reroute_attempts_ = 0;
        if (!advanceToNextSwath()) {
          phase_ = Phase::DONE;
          return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        phase_ = Phase::TRANSIT_TO_SWATH;
        transit_cooldown_until_ = now + std::chrono::seconds(3);
        sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
        return BT::NodeStatus::RUNNING;
      }
      last_progress_time_ = now;
    }

    auto status = nav_handle_->get_status();

    if (checkStuck(ctx)) {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: stuck during reroute on swath %zu", swath_index_ + 1);
      nav_client_->async_cancel_goal(nav_handle_);
      nav_handle_.reset();
      skipped_swaths_++;
      reroute_attempts_ = 0;
      if (!advanceToNextSwath()) {
        phase_ = Phase::DONE;
        return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      phase_ = Phase::TRANSIT_TO_SWATH;
      transit_cooldown_until_ = now + std::chrono::seconds(3);
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      RCLCPP_INFO(ctx->node->get_logger(),
        "ExecuteSwathBySwath: rerouted past obstacle on swath %zu, resuming mowing",
        swath_index_ + 1);
      nav_handle_.reset();

      // Resume mowing from current position to swath end.
      double rx = 0.0, ry = 0.0;
      try {
        auto tf = ctx->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        rx = tf.transform.translation.x;
        ry = tf.transform.translation.y;
      } catch (const tf2::TransformException &) {
        // Use the target point from the reroute goal.
        const auto & sw = ctx->coverage_plan->swaths[swath_index_];
        rx = sw.start.x + blocked_swath_fraction_ * (sw.end.x - sw.start.x);
        ry = sw.start.y + blocked_swath_fraction_ * (sw.end.y - sw.start.y);
      }

      setBladeEnabled(true);
      phase_ = Phase::MOWING_SWATH;
      last_progress_time_ = now;
      sendRemainingSwathGoal(ctx->coverage_plan->swaths[swath_index_], rx, ry);
      return BT::NodeStatus::RUNNING;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: reroute navigation failed for swath %zu", swath_index_ + 1);
      nav_handle_.reset();
      skipped_swaths_++;
      reroute_attempts_ = 0;
      if (!advanceToNextSwath()) {
        phase_ = Phase::DONE;
        return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      phase_ = Phase::TRANSIT_TO_SWATH;
      transit_cooldown_until_ = now + std::chrono::seconds(3);
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

void ExecuteSwathBySwath::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (nav_handle_) {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  if (follow_handle_) {
    follow_client_->async_cancel_goal(follow_handle_);
  }

  setBladeEnabled(false);
  nav_handle_.reset();
  follow_handle_.reset();
  phase_ = Phase::DONE;
  reroute_attempts_ = 0;
  blocked_swath_fraction_ = 0.0;

  RCLCPP_INFO(ctx->node->get_logger(),
    "ExecuteSwathBySwath: halted (%zu completed, %zu skipped of %zu)",
    completed_swaths_, skipped_swaths_, total_swaths_);
}

}  // namespace mowgli_behavior
