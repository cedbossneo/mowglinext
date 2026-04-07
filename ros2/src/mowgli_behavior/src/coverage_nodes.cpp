// Copyright 2026 Mowgli Project
//
// Licensed under the GNU General Public License, version 3 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.html

#include "mowgli_behavior/coverage_nodes.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_behavior
{

// ===========================================================================
// Coverage checkpoint persistence helpers
// ===========================================================================

static const std::string kCheckpointPath = "/ros2_ws/maps/coverage_checkpoint.txt";

/// Compute a simple hash of the coverage plan for identity comparison.
/// Uses number of swaths + first/last swath positions.
static uint64_t computePlanHash(const BTContext::CoveragePlan &plan)
{
  std::size_t h = std::hash<size_t>{}(plan.swaths.size());
  if (!plan.swaths.empty())
  {
    const auto &first = plan.swaths.front();
    const auto &last = plan.swaths.back();
    auto combine = [&h](float v)
    {
      h ^= std::hash<float>{}(v) + 0x9e3779b9 + (h << 6) + (h >> 2);
    };
    combine(first.start.x);
    combine(first.start.y);
    combine(first.end.x);
    combine(first.end.y);
    combine(last.start.x);
    combine(last.start.y);
    combine(last.end.x);
    combine(last.end.y);
  }
  return static_cast<uint64_t>(h);
}

/// Save checkpoint: plan_hash and next_swath_index.
static void saveCheckpoint(uint64_t plan_hash, size_t next_swath_index)
{
  try
  {
    std::filesystem::create_directories(std::filesystem::path(kCheckpointPath).parent_path());
    std::ofstream ofs(kCheckpointPath);
    if (ofs.is_open())
    {
      ofs << plan_hash << " " << next_swath_index << "\n";
    }
  }
  catch (...)
  {
    // Non-fatal -- checkpoint is best-effort.
  }
}

/// Load checkpoint.  Returns true if file exists and was parsed successfully.
static bool loadCheckpoint(uint64_t &plan_hash, size_t &next_swath_index)
{
  try
  {
    std::ifstream ifs(kCheckpointPath);
    if (!ifs.is_open())
    {
      return false;
    }
    ifs >> plan_hash >> next_swath_index;
    return !ifs.fail();
  }
  catch (...)
  {
    return false;
  }
}

// ===========================================================================
// ComputeCoverage — calls coverage_planner_node's PlanCoverage action
// ===========================================================================

BT::NodeStatus ComputeCoverage::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  area_index_ = 0u;
  getInput<uint32_t>("area_index", area_index_);

  result_received_ = false;
  latest_result_.reset();
  goal_handle_.reset();

  if (!action_client_)
  {
    action_client_ =
        rclcpp_action::create_client<CoverageAction>(ctx->node,
                                                     "/coverage_planner_node/plan_coverage");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "ComputeCoverage: plan_coverage action not available");
    return BT::NodeStatus::FAILURE;
  }

  // Fetch mowing area boundary from map_server.
  CoverageAction::Goal goal_msg;

  {
    auto helper = ctx->helper_node;
    auto tmp_client = helper->create_client<mowgli_interfaces::srv::GetMowingArea>(
        "/map_server_node/get_mowing_area");

    if (!tmp_client->wait_for_service(std::chrono::milliseconds(2000)))
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "ComputeCoverage: get_mowing_area service unavailable");
      return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
    request->index = area_index_;

    auto future = tmp_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(helper, future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "ComputeCoverage: get_mowing_area timed out");
      return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response->success)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "ComputeCoverage: map_server returned no area for index %u",
                   area_index_);
      return BT::NodeStatus::FAILURE;
    }

    if (response->area.is_navigation_area)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "ComputeCoverage: area %u is navigation-only, skipping",
                  area_index_);
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
  opts.result_callback = [this](const GoalHandle::WrappedResult &wr)
  {
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
  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "ComputeCoverage: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  // Wait for result.
  if (!result_received_)
  {
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
  if (!latest_result_ || !latest_result_->success)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "ComputeCoverage: planner failed — %s",
                latest_result_ ? latest_result_->message.c_str() : "no result");
    return BT::NodeStatus::FAILURE;
  }

  // Extract swath endpoints into BTContext::CoveragePlan.
  BTContext::CoveragePlan plan;

  const auto &starts = latest_result_->swath_starts;
  const auto &ends = latest_result_->swath_ends;
  const size_t n_swaths = std::min(starts.size(), ends.size());

  for (size_t i = 0; i < n_swaths; ++i)
  {
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

  if (plan.swaths.empty())
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "ComputeCoverage: empty plan");
    return BT::NodeStatus::FAILURE;
  }

  // Store the full F2C discretized path (swaths + Dubins turns).
  plan.full_path = latest_result_->path;

  // Store in context.
  ctx->coverage_plan = std::move(plan);
  ctx->next_swath_index = 0;
  ctx->current_area = static_cast<int>(area_index_);

  // Check for a saved checkpoint — resume from where we left off if the
  // plan hash matches (same area, same swath layout).
  {
    const uint64_t current_hash = computePlanHash(*ctx->coverage_plan);
    uint64_t saved_hash = 0;
    size_t saved_index = 0;
    if (loadCheckpoint(saved_hash, saved_index) && saved_hash == current_hash &&
        saved_index < ctx->coverage_plan->swaths.size())
    {
      ctx->next_swath_index = saved_index;
      RCLCPP_INFO(ctx->node->get_logger(),
                  "ComputeCoverage: resuming from checkpoint — swath %zu/%zu",
                  saved_index + 1,
                  ctx->coverage_plan->swaths.size());
    }
    else
    {
      // New plan — save initial checkpoint.
      saveCheckpoint(current_hash, 0);
    }
  }

  // Output first swath start pose.
  const auto &first = ctx->coverage_plan->swaths.front();
  const double dx = first.end.x - first.start.x;
  const double dy = first.end.y - first.start.y;
  const double yaw = std::atan2(dy, dx);

  std::ostringstream oss;
  oss << first.start.x << ";" << first.start.y << ";" << yaw;
  setOutput("first_swath_start", oss.str());

  RCLCPP_INFO(ctx->node->get_logger(),
              "ComputeCoverage: first swath start (%.2f, %.2f, yaw=%.2f)",
              first.start.x,
              first.start.y,
              yaw);

  return BT::NodeStatus::SUCCESS;
}

void ComputeCoverage::onHalted()
{
  if (goal_handle_)
  {
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

nav_msgs::msg::Path ExecuteSwathBySwath::swathToPath(const BTContext::Swath &swath,
                                                     const rclcpp::Node::SharedPtr &node) const
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

  for (int i = 0; i < num_points; ++i)
  {
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

void ExecuteSwathBySwath::sendTransitGoal(const BTContext::Swath &swath)
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
              swath_index_ + 1,
              total_swaths_,
              swath.start.x,
              swath.start.y);
}

void ExecuteSwathBySwath::sendSwathGoal(const BTContext::Swath &swath)
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
              swath_index_ + 1,
              total_swaths_,
              len);
}

void ExecuteSwathBySwath::setBladeEnabled(bool enabled)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!blade_client_)
  {
    blade_client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control");
  }

  auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  req->mow_enabled = enabled ? 1u : 0u;
  req->mow_direction = 0u;

  // Fire-and-forget: firmware is the safety authority for the blade.
  auto future = blade_client_->async_send_request(req);
  (void)future;
}

bool ExecuteSwathBySwath::advanceToNextSwath()
{
  swath_index_++;
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->next_swath_index = swath_index_;

  // Sync coverage progress to context for PublishHighLevelStatus
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->total_swaths = static_cast<int>(total_swaths_);
    ctx->completed_swaths = static_cast<int>(completed_swaths_);
    ctx->skipped_swaths = static_cast<int>(skipped_swaths_);
  }

  // Persist progress so we can resume after a restart.
  if (ctx->coverage_plan)
  {
    saveCheckpoint(computePlanHash(*ctx->coverage_plan), swath_index_);
  }

  return swath_index_ < total_swaths_;
}

bool ExecuteSwathBySwath::checkStuck(const std::shared_ptr<BTContext> &ctx)
{
  double rx = 0.0, ry = 0.0;
  try
  {
    auto transform = ctx->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    rx = transform.transform.translation.x;
    ry = transform.transform.translation.y;
  }
  catch (const tf2::TransformException &)
  {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  const double dx = rx - last_progress_x_;
  const double dy = ry - last_progress_y_;
  const double dist = std::hypot(dx, dy);

  if (dist >= stuck_min_progress_)
  {
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

  if (!ctx->coverage_plan || ctx->coverage_plan->swaths.empty())
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "ExecuteSwathBySwath: no coverage plan in context");
    return BT::NodeStatus::FAILURE;
  }

  total_swaths_ = ctx->coverage_plan->swaths.size();
  swath_index_ = ctx->next_swath_index;
  completed_swaths_ = 0;
  skipped_swaths_ = 0;

  // Read configurable stuck detection thresholds from input ports
  getInput<double>("stuck_timeout_sec", stuck_timeout_sec_);
  getInput<double>("stuck_min_progress", stuck_min_progress_);

  // Publish initial progress to context
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->total_swaths = static_cast<int>(total_swaths_);
    ctx->completed_swaths = static_cast<int>(completed_swaths_);
    ctx->skipped_swaths = static_cast<int>(skipped_swaths_);
  }

  if (swath_index_ >= total_swaths_)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "ExecuteSwathBySwath: all %zu swaths already completed",
                total_swaths_);
    return BT::NodeStatus::SUCCESS;
  }

  if (!nav_client_)
  {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }
  if (!follow_client_)
  {
    follow_client_ = rclcpp_action::create_client<Nav2FollowPath>(ctx->node, "/follow_path");
  }

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "ExecuteSwathBySwath: /navigate_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "ExecuteSwathBySwath: /follow_path not available");
    return BT::NodeStatus::FAILURE;
  }

  setBladeEnabled(false);
  phase_ = Phase::TRANSIT_TO_SWATH;
  last_progress_time_ = std::chrono::steady_clock::now();
  last_progress_x_ = 0.0;
  last_progress_y_ = 0.0;

  const auto &swath = ctx->coverage_plan->swaths[swath_index_];
  sendTransitGoal(swath);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteSwathBySwath::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  const auto now = std::chrono::steady_clock::now();

  if (phase_ == Phase::DONE)
  {
    return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // Cooldown after transit failure.
  if (now < transit_cooldown_until_)
  {
    return BT::NodeStatus::RUNNING;
  }

  const auto &swath = ctx->coverage_plan->swaths[swath_index_];

  if (phase_ == Phase::TRANSIT_TO_SWATH)
  {
    // Wait for goal acceptance.
    if (!nav_handle_)
    {
      if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      {
        return BT::NodeStatus::RUNNING;
      }
      nav_handle_ = nav_future_.get();
      if (!nav_handle_)
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "ExecuteSwathBySwath: transit goal rejected for swath %zu",
                    swath_index_ + 1);
        skipped_swaths_++;
        if (!advanceToNextSwath())
        {
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

    if (checkStuck(ctx))
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "ExecuteSwathBySwath: stuck during transit to swath %zu, skipping",
                  swath_index_ + 1);
      nav_client_->async_cancel_goal(nav_handle_);
      nav_handle_.reset();
      skipped_swaths_++;
      if (!advanceToNextSwath())
      {
        phase_ = Phase::DONE;
        return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      transit_cooldown_until_ = now + std::chrono::seconds(3);
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "ExecuteSwathBySwath: arrived at swath %zu start",
                  swath_index_ + 1);
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
                  "ExecuteSwathBySwath: transit failed for swath %zu",
                  swath_index_ + 1);
      nav_handle_.reset();
      skipped_swaths_++;
      if (!advanceToNextSwath())
      {
        phase_ = Phase::DONE;
        return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      transit_cooldown_until_ = now + std::chrono::seconds(3);
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::MOWING_SWATH)
  {
    if (!follow_handle_)
    {
      if (follow_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      {
        return BT::NodeStatus::RUNNING;
      }
      follow_handle_ = follow_future_.get();
      if (!follow_handle_)
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "ExecuteSwathBySwath: follow goal rejected for swath %zu",
                    swath_index_ + 1);
        setBladeEnabled(false);
        skipped_swaths_++;
        if (!advanceToNextSwath())
        {
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

    if (checkStuck(ctx))
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "ExecuteSwathBySwath: stuck during swath %zu",
                  swath_index_ + 1);
      follow_client_->async_cancel_goal(follow_handle_);
      follow_handle_.reset();
      setBladeEnabled(false);
      skipped_swaths_++;
      if (!advanceToNextSwath())
      {
        phase_ = Phase::DONE;
        return (completed_swaths_ > 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      phase_ = Phase::TRANSIT_TO_SWATH;
      transit_cooldown_until_ = now + std::chrono::seconds(3);
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "ExecuteSwathBySwath: swath %zu/%zu completed",
                  swath_index_ + 1,
                  total_swaths_);
      follow_handle_.reset();
      setBladeEnabled(false);
      completed_swaths_++;

      if (!advanceToNextSwath())
      {
        RCLCPP_INFO(ctx->node->get_logger(),
                    "ExecuteSwathBySwath: all swaths done (%zu completed, %zu skipped)",
                    completed_swaths_,
                    skipped_swaths_);
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
                  "ExecuteSwathBySwath: swath %zu follow failed",
                  swath_index_ + 1);
      follow_handle_.reset();
      setBladeEnabled(false);
      skipped_swaths_++;
      if (!advanceToNextSwath())
      {
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

  if (nav_handle_)
  {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  if (follow_handle_)
  {
    follow_client_->async_cancel_goal(follow_handle_);
  }

  setBladeEnabled(false);
  nav_handle_.reset();
  follow_handle_.reset();
  phase_ = Phase::DONE;

  RCLCPP_INFO(ctx->node->get_logger(),
              "ExecuteSwathBySwath: halted (%zu completed, %zu skipped of %zu)",
              completed_swaths_,
              skipped_swaths_,
              total_swaths_);
}

// ===========================================================================
// ExecuteFullCoveragePath — sends full F2C path to Nav2 FollowPath
// ===========================================================================

size_t ExecuteFullCoveragePath::findClosestPoseIndex(const nav_msgs::msg::Path &path,
                                                     double rx,
                                                     double ry) const
{
  size_t best = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i)
  {
    const double dx = path.poses[i].pose.position.x - rx;
    const double dy = path.poses[i].pose.position.y - ry;
    const double d = dx * dx + dy * dy;
    if (d < best_dist)
    {
      best_dist = d;
      best = i;
    }
  }
  return best;
}

void ExecuteFullCoveragePath::setBladeEnabled(bool enabled)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!blade_client_)
  {
    blade_client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control");
  }

  if (!blade_client_->service_is_ready())
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "ExecuteFullCoveragePath: blade service unavailable (sim mode)");
    return;
  }

  auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  req->mow_enabled = enabled ? 1u : 0u;
  req->mow_direction = 0u;
  blade_client_->async_send_request(req);
}

bool ExecuteFullCoveragePath::checkStuck(const std::shared_ptr<BTContext> &ctx)
{
  double rx = 0.0, ry = 0.0;
  try
  {
    auto transform = ctx->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    rx = transform.transform.translation.x;
    ry = transform.transform.translation.y;
  }
  catch (const tf2::TransformException &)
  {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  const double dist = std::hypot(rx - last_progress_x_, ry - last_progress_y_);

  if (dist >= stuck_min_progress_)
  {
    last_progress_x_ = rx;
    last_progress_y_ = ry;
    last_progress_time_ = now;
    return false;
  }

  const double elapsed = std::chrono::duration<double>(now - last_progress_time_).count();
  return elapsed >= stuck_timeout_sec_;
}

BT::NodeStatus ExecuteFullCoveragePath::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!ctx->coverage_plan || ctx->coverage_plan->full_path.poses.empty())
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "ExecuteFullCoveragePath: no coverage path in context");
    return BT::NodeStatus::FAILURE;
  }

  if (!nav_client_)
  {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }
  if (!follow_client_)
  {
    follow_client_ = rclcpp_action::create_client<Nav2FollowPath>(ctx->node, "/follow_path");
  }

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "ExecuteFullCoveragePath: /navigate_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "ExecuteFullCoveragePath: /follow_path not available");
    return BT::NodeStatus::FAILURE;
  }

  last_progress_time_ = std::chrono::steady_clock::now();
  last_progress_x_ = 0.0;
  last_progress_y_ = 0.0;
  inline_retries_ = 0;

  const auto &full_path = ctx->coverage_plan->full_path;

  // Check robot distance to the closest point on the path.
  // If close enough, send FollowPath directly from that point.
  // Otherwise, prepend current position so the controller smoothly drives there.
  double rx = 0.0, ry = 0.0;
  bool have_pose = false;
  try
  {
    auto tf = ctx->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    rx = tf.transform.translation.x;
    ry = tf.transform.translation.y;
    have_pose = true;
  }
  catch (const tf2::TransformException &)
  {
  }

  size_t closest_index = 0;
  double closest_dist = std::numeric_limits<double>::max();
  if (have_pose)
  {
    closest_index = findClosestPoseIndex(full_path, rx, ry);
    const double dx = full_path.poses[closest_index].pose.position.x - rx;
    const double dy = full_path.poses[closest_index].pose.position.y - ry;
    closest_dist = std::hypot(dx, dy);
  }

  if (have_pose)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "ExecuteFullCoveragePath: robot at (%.2f, %.2f), %.2fm from path, "
                "following from pose %zu/%zu",
                rx,
                ry,
                closest_dist,
                closest_index,
                full_path.poses.size());

    nav_msgs::msg::Path sub_path;
    sub_path.header = full_path.header;
    sub_path.header.stamp = ctx->node->now();

    // If robot is far from the path, prepend current position as first waypoint
    if (closest_dist > 0.5)
    {
      geometry_msgs::msg::PoseStamped robot_pose;
      robot_pose.header = full_path.header;
      robot_pose.header.stamp = ctx->node->now();
      robot_pose.pose.position.x = rx;
      robot_pose.pose.position.y = ry;
      robot_pose.pose.orientation = full_path.poses[0].pose.orientation;
      sub_path.poses.push_back(robot_pose);
    }

    sub_path.poses.insert(sub_path.poses.end(),
                          full_path.poses.begin() + static_cast<long>(closest_index),
                          full_path.poses.end());

    if (sub_path.poses.empty())
    {
      return BT::NodeStatus::SUCCESS;
    }

    setBladeEnabled(true);

    // Distance-based path thinning.  Keep poses at most `min_spacing` apart,
    // but always retain poses where heading changes significantly (turns between
    // swaths).  This preserves turn geometry while staying manageable.
    nav_msgs::msg::Path send_path = sub_path;
    if (send_path.poses.size() > 5000)
    {
      const double target_count = 1400.0;
      const double total_len = [&]()
      {
        double d = 0.0;
        for (size_t i = 1; i < send_path.poses.size(); ++i)
        {
          const double ddx =
              send_path.poses[i].pose.position.x - send_path.poses[i - 1].pose.position.x;
          const double ddy =
              send_path.poses[i].pose.position.y - send_path.poses[i - 1].pose.position.y;
          d += std::hypot(ddx, ddy);
        }
        return d;
      }();
      const double min_spacing = total_len / target_count;
      const double heading_threshold = 0.3;  // ~17 deg — keep turn waypoints

      nav_msgs::msg::Path thinned;
      thinned.header = send_path.header;
      thinned.poses.push_back(send_path.poses.front());
      double accum_dist = 0.0;
      for (size_t i = 1; i < send_path.poses.size(); ++i)
      {
        const double ddx =
            send_path.poses[i].pose.position.x - send_path.poses[i - 1].pose.position.x;
        const double ddy =
            send_path.poses[i].pose.position.y - send_path.poses[i - 1].pose.position.y;
        accum_dist += std::hypot(ddx, ddy);

        // Check heading change (detect turns between swaths)
        bool is_turn = false;
        if (i + 1 < send_path.poses.size())
        {
          const double dx1 = ddx, dy1 = ddy;
          const double dx2 =
              send_path.poses[i + 1].pose.position.x - send_path.poses[i].pose.position.x;
          const double dy2 =
              send_path.poses[i + 1].pose.position.y - send_path.poses[i].pose.position.y;
          const double len1 = std::hypot(dx1, dy1);
          const double len2 = std::hypot(dx2, dy2);
          if (len1 > 1e-6 && len2 > 1e-6)
          {
            const double cross = std::abs(dx1 * dy2 - dy1 * dx2) / (len1 * len2);
            is_turn = cross > heading_threshold;
          }
        }

        if (accum_dist >= min_spacing || is_turn)
        {
          thinned.poses.push_back(send_path.poses[i]);
          accum_dist = 0.0;
        }
      }
      // Always include the last pose
      const auto &last = send_path.poses.back();
      const auto &kept = thinned.poses.back();
      if (last.pose.position.x != kept.pose.position.x ||
          last.pose.position.y != kept.pose.position.y)
      {
        thinned.poses.push_back(last);
      }
      RCLCPP_INFO(ctx->node->get_logger(),
                  "ExecuteFullCoveragePath: thinned %zu -> %zu poses (spacing=%.3fm, path=%.1fm)",
                  send_path.poses.size(),
                  thinned.poses.size(),
                  min_spacing,
                  total_len);
      send_path = thinned;
    }

    Nav2FollowPath::Goal goal;
    goal.path = send_path;
    goal.controller_id = "FollowPath";
    goal.goal_checker_id = "coverage_goal_checker";
    follow_handle_.reset();
    follow_future_ = follow_client_->async_send_goal(goal);
    phase_ = Phase::FOLLOWING_PATH;
  }
  else
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "ExecuteFullCoveragePath: no robot pose available");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteFullCoveragePath::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (phase_ == Phase::FOLLOWING_PATH)
  {
    if (!follow_handle_)
    {
      if (follow_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      {
        return BT::NodeStatus::RUNNING;
      }
      follow_handle_ = follow_future_.get();
      if (!follow_handle_)
      {
        RCLCPP_ERROR(ctx->node->get_logger(), "ExecuteFullCoveragePath: FollowPath goal rejected");
        setBladeEnabled(false);
        return BT::NodeStatus::FAILURE;
      }
      last_progress_time_ = std::chrono::steady_clock::now();
    }

    auto status = follow_handle_->get_status();

    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
    {
      RCLCPP_INFO(ctx->node->get_logger(), "ExecuteFullCoveragePath: coverage path completed");
      follow_handle_.reset();
      setBladeEnabled(false);
      return BT::NodeStatus::SUCCESS;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
        status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "ExecuteFullCoveragePath: FollowPath aborted/canceled (retry %d/%d)",
                  inline_retries_,
                  max_inline_retries_);
      follow_handle_.reset();

      // Inline recovery: skip ahead on path and resend immediately.
      // This avoids the slow BT recovery (BackUp + ClearCostmap + 3s wait).
      if (inline_retries_ < max_inline_retries_ && resendFromCurrentPose(ctx))
      {
        inline_retries_++;
        return BT::NodeStatus::RUNNING;
      }
      setBladeEnabled(false);
      return BT::NodeStatus::FAILURE;
    }

    if (checkStuck(ctx))
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "ExecuteFullCoveragePath: stuck detected (retry %d/%d)",
                  inline_retries_,
                  max_inline_retries_);
      follow_client_->async_cancel_goal(follow_handle_);
      follow_handle_.reset();

      if (inline_retries_ < max_inline_retries_ && resendFromCurrentPose(ctx))
      {
        inline_retries_++;
        return BT::NodeStatus::RUNNING;
      }
      setBladeEnabled(false);
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

bool ExecuteFullCoveragePath::resendFromCurrentPose(const std::shared_ptr<BTContext> &ctx)
{
  if (!ctx->coverage_plan || ctx->coverage_plan->full_path.poses.empty())
  {
    return false;
  }

  double rx = 0.0, ry = 0.0;
  try
  {
    auto tf = ctx->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    rx = tf.transform.translation.x;
    ry = tf.transform.translation.y;
  }
  catch (const tf2::TransformException &)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "ExecuteFullCoveragePath: resend failed — no TF");
    return false;
  }

  const auto &full_path = ctx->coverage_plan->full_path;
  size_t closest = findClosestPoseIndex(full_path, rx, ry);

  // Skip ahead past the failure point to avoid hitting the same obstacle
  size_t resume_index = std::min(closest + skip_poses_on_retry_, full_path.poses.size() - 1);

  if (resume_index >= full_path.poses.size() - 10)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "ExecuteFullCoveragePath: near end of path, declaring complete");
    return false;
  }

  nav_msgs::msg::Path sub_path;
  sub_path.header = full_path.header;
  sub_path.header.stamp = ctx->node->now();

  // Prepend current position for smooth transition
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header = sub_path.header;
  robot_pose.pose.position.x = rx;
  robot_pose.pose.position.y = ry;
  robot_pose.pose.orientation = full_path.poses[resume_index].pose.orientation;
  sub_path.poses.push_back(robot_pose);

  sub_path.poses.insert(sub_path.poses.end(),
                        full_path.poses.begin() + static_cast<long>(resume_index),
                        full_path.poses.end());

  RCLCPP_INFO(ctx->node->get_logger(),
              "ExecuteFullCoveragePath: inline resend from pose %zu/%zu "
              "(skipped %zu poses, robot at %.2f,%.2f)",
              resume_index,
              full_path.poses.size(),
              resume_index - closest,
              rx,
              ry);

  Nav2FollowPath::Goal goal;
  goal.path = sub_path;
  goal.controller_id = "FollowPath";
  goal.goal_checker_id = "coverage_goal_checker";
  follow_handle_.reset();
  follow_future_ = follow_client_->async_send_goal(goal);
  phase_ = Phase::FOLLOWING_PATH;
  last_progress_time_ = std::chrono::steady_clock::now();
  last_progress_x_ = rx;
  last_progress_y_ = ry;
  return true;
}

void ExecuteFullCoveragePath::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (nav_handle_)
  {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  if (follow_handle_)
  {
    follow_client_->async_cancel_goal(follow_handle_);
  }
  nav_handle_.reset();
  follow_handle_.reset();
  setBladeEnabled(false);

  RCLCPP_INFO(ctx->node->get_logger(), "ExecuteFullCoveragePath: halted");
}

}  // namespace mowgli_behavior
