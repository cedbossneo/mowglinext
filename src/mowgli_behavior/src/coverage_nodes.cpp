#include "mowgli_behavior/coverage_nodes.hpp"

#include <cmath>
#include <sstream>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_behavior {

// ===========================================================================
// ComputeCoverage
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
    action_client_ = rclcpp_action::create_client<CoverageAction>(
      ctx->node, "/compute_coverage_path");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "ComputeCoverage: coverage_server action not available");
    return BT::NodeStatus::FAILURE;
  }

  // Fetch mowing area boundary from map_server.
  CoverageAction::Goal goal_msg;
  goal_msg.generate_headland = true;
  goal_msg.generate_route = true;
  goal_msg.generate_path = true;
  goal_msg.use_gml_file = false;
  goal_msg.frame_id = "map";

  {
    auto tmp_node = rclcpp::Node::make_shared("_compute_coverage_srv_helper");
    auto tmp_client = tmp_node->create_client<mowgli_interfaces::srv::GetMowingArea>(
      "/map_server_node/get_mowing_area");

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

    // Convert geometry_msgs/Polygon to opennav_coverage Coordinates format.
    // Coordinate uses axis1/axis2 (float32), not Point32.
    // F2C requires closed polygons: first point == last point.
    opennav_coverage_msgs::msg::Coordinates coords;
    for (const auto& pt : response->area.area.points) {
      opennav_coverage_msgs::msg::Coordinate c;
      c.axis1 = pt.x;
      c.axis2 = pt.y;
      coords.coordinates.push_back(c);
    }
    // Close the polygon if not already closed.
    if (!coords.coordinates.empty()) {
      const auto& first = coords.coordinates.front();
      const auto& last = coords.coordinates.back();
      if (first.axis1 != last.axis1 || first.axis2 != last.axis2) {
        coords.coordinates.push_back(first);
      }
    }
    goal_msg.polygons.push_back(coords);

    // Add obstacle polygons as interior voids (subsequent polygons).
    for (const auto& obstacle : response->area.obstacles) {
      opennav_coverage_msgs::msg::Coordinates obs_coords;
      for (const auto& pt : obstacle.points) {
        opennav_coverage_msgs::msg::Coordinate c;
        c.axis1 = pt.x;
        c.axis2 = pt.y;
        obs_coords.coordinates.push_back(c);
      }
      // Close obstacle polygon if not already closed.
      if (!obs_coords.coordinates.empty()) {
        const auto& first = obs_coords.coordinates.front();
        const auto& last = obs_coords.coordinates.back();
        if (first.axis1 != last.axis1 || first.axis2 != last.axis2) {
          obs_coords.coordinates.push_back(first);
        }
      }
      goal_msg.polygons.push_back(obs_coords);
    }

    RCLCPP_INFO(ctx->node->get_logger(),
      "ComputeCoverage: area '%s' — %zu boundary pts, %zu obstacles",
      response->area.name.c_str(),
      response->area.area.points.size(),
      response->area.obstacles.size());
  }

  // Send goal.
  auto opts = rclcpp_action::Client<CoverageAction>::SendGoalOptions{};
  opts.result_callback =
    [this](const GoalHandle::WrappedResult& wr) {
      latest_result_ = wr.result;
      result_received_ = true;
    };

  goal_handle_future_ = action_client_->async_send_goal(goal_msg, opts);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(), "ComputeCoverage: goal sent");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputeCoverage::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Wait for goal acceptance.
  if (!goal_handle_) {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
    {
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
  if (!latest_result_ || latest_result_->error_code != 0) {
    RCLCPP_WARN(ctx->node->get_logger(),
      "ComputeCoverage: planner returned error_code=%u",
      latest_result_ ? latest_result_->error_code : 999);
    return BT::NodeStatus::FAILURE;
  }

  // Extract PathComponents into BTContext::CoveragePlan.
  const auto& pc = latest_result_->coverage_path;
  BTContext::CoveragePlan plan;

  for (const auto& swath : pc.swaths) {
    BTContext::Swath s;
    s.start = swath.start;
    s.end = swath.end;
    plan.swaths.push_back(s);
  }
  plan.turns = pc.turns;

  RCLCPP_INFO(ctx->node->get_logger(),
    "ComputeCoverage: received %zu swaths, %zu turns",
    plan.swaths.size(), plan.turns.size());

  if (plan.swaths.empty()) {
    RCLCPP_ERROR(ctx->node->get_logger(), "ComputeCoverage: empty plan");
    return BT::NodeStatus::FAILURE;
  }

  // Store in context.
  ctx->coverage_plan = std::move(plan);
  ctx->next_swath_index = 0;

  // Output first swath start pose.
  const auto& first = ctx->coverage_plan->swaths.front();
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
// ExecuteSwathBySwath
// ===========================================================================

nav_msgs::msg::Path ExecuteSwathBySwath::swathToPath(
  const BTContext::Swath& swath,
  const rclcpp::Node::SharedPtr& node) const
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

  // Interpolate at ~0.10m spacing so RPP has enough poses to track.
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

void ExecuteSwathBySwath::sendTransitGoal(const BTContext::Swath& swath)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

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

  RCLCPP_INFO(ctx->node->get_logger(),
    "ExecuteSwathBySwath: transit to swath %zu/%zu start (%.2f, %.2f)",
    swath_index_ + 1, total_swaths_, swath.start.x, swath.start.y);
}

void ExecuteSwathBySwath::sendSwathGoal(const BTContext::Swath& swath)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  Nav2FollowPath::Goal goal;
  goal.path = swathToPath(swath, ctx->node);
  goal.controller_id = "FollowCoveragePath";
  goal.goal_checker_id = "coverage_goal_checker";

  follow_handle_.reset();
  follow_future_ = follow_client_->async_send_goal(goal);

  const double len = std::hypot(
    swath.end.x - swath.start.x, swath.end.y - swath.start.y);
  RCLCPP_INFO(ctx->node->get_logger(),
    "ExecuteSwathBySwath: mowing swath %zu/%zu (%.1fm)",
    swath_index_ + 1, total_swaths_, len);
}

void ExecuteSwathBySwath::setBladeEnabled(bool enabled)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!blade_client_) {
    blade_client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
      "/hardware_bridge/mower_control");
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

bool ExecuteSwathBySwath::checkStuck(const std::shared_ptr<BTContext>& ctx)
{
  // Try to get the robot's current position from TF.
  double rx = 0.0, ry = 0.0;
  try {
    auto transform = ctx->tf_buffer->lookupTransform(
      "map", "base_link", tf2::TimePointZero);
    rx = transform.transform.translation.x;
    ry = transform.transform.translation.y;
  } catch (const tf2::TransformException&) {
    return false;  // Can't check if stuck without TF.
  }

  const auto now = std::chrono::steady_clock::now();
  const double dx = rx - last_progress_x_;
  const double dy = ry - last_progress_y_;
  const double dist = std::hypot(dx, dy);

  if (dist >= stuck_min_progress_) {
    // Robot has moved — reset progress tracker.
    last_progress_x_ = rx;
    last_progress_y_ = ry;
    last_progress_time_ = now;
    return false;
  }

  // Robot hasn't moved enough — check timeout.
  const double elapsed = std::chrono::duration<double>(
    now - last_progress_time_).count();
  return elapsed >= stuck_timeout_sec_;
}

BT::NodeStatus ExecuteSwathBySwath::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!ctx->coverage_plan || ctx->coverage_plan->swaths.empty()) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "ExecuteSwathBySwath: no coverage plan in context");
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

  // Create action clients.
  if (!nav_client_) {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(
      ctx->node, "/navigate_to_pose");
  }
  if (!follow_client_) {
    follow_client_ = rclcpp_action::create_client<Nav2FollowPath>(
      ctx->node, "/follow_path");
  }

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "ExecuteSwathBySwath: /navigate_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "ExecuteSwathBySwath: /follow_path not available");
    return BT::NodeStatus::FAILURE;
  }

  // Start transit to first swath.
  setBladeEnabled(false);
  phase_ = Phase::TRANSIT_TO_SWATH;
  last_progress_time_ = std::chrono::steady_clock::now();
  last_progress_x_ = 0.0;
  last_progress_y_ = 0.0;
  // Force initial position reading on next checkStuck call.
  try {
    auto t = ctx->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
    last_progress_x_ = t.transform.translation.x;
    last_progress_y_ = t.transform.translation.y;
  } catch (...) {}
  sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);

  RCLCPP_INFO(ctx->node->get_logger(),
    "ExecuteSwathBySwath: starting from swath %zu/%zu",
    swath_index_ + 1, total_swaths_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteSwathBySwath::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  switch (phase_) {

  // --- Transit to swath start (obstacle-aware via SmacPlanner2D) ---
  case Phase::TRANSIT_TO_SWATH: {
    // Cooldown after a failed transit: wait before sending the next goal so
    // we don't flood bt_navigator with rapid-fire requests that overwhelm it.
    if (transit_cooldown_until_.time_since_epoch().count() > 0) {
      if (std::chrono::steady_clock::now() < transit_cooldown_until_) {
        return BT::NodeStatus::RUNNING;
      }
      // Cooldown expired — reset and send the next transit goal.
      transit_cooldown_until_ = std::chrono::steady_clock::time_point{};
      sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    if (!nav_handle_) {
      if (nav_future_.wait_for(std::chrono::milliseconds(0)) !=
          std::future_status::ready)
      {
        return BT::NodeStatus::RUNNING;
      }
      nav_handle_ = nav_future_.get();
      if (!nav_handle_) {
        RCLCPP_WARN(ctx->node->get_logger(),
          "ExecuteSwathBySwath: transit goal rejected for swath %zu, skipping",
          swath_index_ + 1);
        skipped_swaths_++;
        if (advanceToNextSwath()) {
          // Arm cooldown before next transit attempt.
          transit_cooldown_until_ = std::chrono::steady_clock::now()
            + std::chrono::seconds(2);
          nav_handle_.reset();
          return BT::NodeStatus::RUNNING;
        }
        // All remaining swaths exhausted.
        phase_ = Phase::DONE;
        break;
      }
      // Transit goal accepted — reset stuck timer.
      last_progress_time_ = std::chrono::steady_clock::now();
    }

    const auto status = nav_handle_->get_status();

    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      // Arrived at swath start. Begin mowing.
      RCLCPP_INFO(ctx->node->get_logger(),
        "ExecuteSwathBySwath: arrived at swath %zu start, blade ON",
        swath_index_ + 1);
      setBladeEnabled(true);
      phase_ = Phase::MOWING_SWATH;
      last_progress_time_ = std::chrono::steady_clock::now();
      sendSwathGoal(ctx->coverage_plan->swaths[swath_index_]);
      return BT::NodeStatus::RUNNING;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
        status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: transit to swath %zu failed, skipping",
        swath_index_ + 1);
      skipped_swaths_++;
      if (advanceToNextSwath()) {
        // Arm cooldown before next transit attempt.
        transit_cooldown_until_ = std::chrono::steady_clock::now()
          + std::chrono::seconds(2);
        nav_handle_.reset();
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::DONE;
      break;
    }

    // Stuck detection during transit.
    if (checkStuck(ctx)) {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: stuck during transit to swath %zu, cancelling",
        swath_index_ + 1);
      nav_client_->async_cancel_goal(nav_handle_);
      skipped_swaths_++;
      if (advanceToNextSwath()) {
        transit_cooldown_until_ = std::chrono::steady_clock::now()
          + std::chrono::seconds(3);
        nav_handle_.reset();
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::DONE;
      break;
    }

    return BT::NodeStatus::RUNNING;
  }

  // --- Mowing swath (straight-line, RPP controller) ---
  case Phase::MOWING_SWATH: {
    if (!follow_handle_) {
      if (follow_future_.wait_for(std::chrono::milliseconds(0)) !=
          std::future_status::ready)
      {
        return BT::NodeStatus::RUNNING;
      }
      follow_handle_ = follow_future_.get();
      if (!follow_handle_) {
        RCLCPP_WARN(ctx->node->get_logger(),
          "ExecuteSwathBySwath: swath %zu follow goal rejected", swath_index_ + 1);
        setBladeEnabled(false);
        skipped_swaths_++;
        if (advanceToNextSwath()) {
          phase_ = Phase::TRANSIT_TO_SWATH;
          nav_handle_.reset();
          last_progress_time_ = std::chrono::steady_clock::now();
          sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
          return BT::NodeStatus::RUNNING;
        }
        phase_ = Phase::DONE;
        break;
      }
      // Follow goal accepted — reset stuck timer.
      last_progress_time_ = std::chrono::steady_clock::now();
    }

    const auto status = follow_handle_->get_status();

    if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      completed_swaths_++;
      setBladeEnabled(false);
      RCLCPP_INFO(ctx->node->get_logger(),
        "ExecuteSwathBySwath: swath %zu/%zu complete (%zu done, %zu skipped)",
        swath_index_ + 1, total_swaths_, completed_swaths_, skipped_swaths_);

      if (advanceToNextSwath()) {
        phase_ = Phase::TRANSIT_TO_SWATH;
        nav_handle_.reset();
        follow_handle_.reset();
        last_progress_time_ = std::chrono::steady_clock::now();
        sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::DONE;
      break;
    }

    if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
        status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: swath %zu follow aborted, skipping",
        swath_index_ + 1);
      setBladeEnabled(false);
      skipped_swaths_++;
      if (advanceToNextSwath()) {
        phase_ = Phase::TRANSIT_TO_SWATH;
        nav_handle_.reset();
        follow_handle_.reset();
        last_progress_time_ = std::chrono::steady_clock::now();
        sendTransitGoal(ctx->coverage_plan->swaths[swath_index_]);
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::DONE;
      break;
    }

    // Stuck detection: if robot hasn't moved in 10s (e.g. collision_monitor
    // stopped it), cancel the current goal and skip to next swath.
    if (checkStuck(ctx)) {
      RCLCPP_WARN(ctx->node->get_logger(),
        "ExecuteSwathBySwath: stuck during swath %zu (no progress for %.0fs), "
        "cancelling and skipping",
        swath_index_ + 1, stuck_timeout_sec_);
      follow_client_->async_cancel_goal(follow_handle_);
      setBladeEnabled(false);
      skipped_swaths_++;
      if (advanceToNextSwath()) {
        phase_ = Phase::TRANSIT_TO_SWATH;
        nav_handle_.reset();
        follow_handle_.reset();
        transit_cooldown_until_ = std::chrono::steady_clock::now()
          + std::chrono::seconds(3);
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::DONE;
      break;
    }

    return BT::NodeStatus::RUNNING;
  }

  case Phase::DONE:
    break;
  }

  // DONE phase.
  setBladeEnabled(false);

  RCLCPP_INFO(ctx->node->get_logger(),
    "ExecuteSwathBySwath: finished — %zu completed, %zu skipped out of %zu",
    completed_swaths_, skipped_swaths_, total_swaths_);

  if (completed_swaths_ == 0) {
    RCLCPP_ERROR(ctx->node->get_logger(),
      "ExecuteSwathBySwath: no swaths completed, returning FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void ExecuteSwathBySwath::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  setBladeEnabled(false);

  if (nav_handle_) {
    nav_client_->async_cancel_goal(nav_handle_);
    nav_handle_.reset();
  }
  if (follow_handle_) {
    follow_client_->async_cancel_goal(follow_handle_);
    follow_handle_.reset();
  }

  // Persist progress so battery resume can continue from here.
  ctx->next_swath_index = swath_index_;

  RCLCPP_INFO(ctx->node->get_logger(),
    "ExecuteSwathBySwath: halted at swath %zu/%zu", swath_index_ + 1, total_swaths_);
}

}  // namespace mowgli_behavior
