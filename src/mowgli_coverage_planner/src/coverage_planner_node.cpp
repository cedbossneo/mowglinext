// Copyright (C) 2024 Cedric
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * @file coverage_planner_node.cpp
 * @brief Implementation of the CoveragePlannerNode using Fields2Cover v2.
 *
 * Algorithm overview (F2C pipeline):
 *   1. Validate the input boundary polygon (must have >= 3 vertices).
 *   2. Convert the ROS Polygon to f2c::types::Field / Cells.
 *   3. Generate headlands via f2c::hg::ConstHL (headland_passes * headland_width).
 *   4. Generate swaths via f2c::sg::BruteForce (optimal angle search).
 *   5. Order swaths via f2c::rp::BoustrophedonOrder.
 *   6. Generate smooth Dubins path via f2c::pp::DubinsCurves.
 *   7. Convert F2C path states back to nav_msgs::msg::Path.
 *
 * The headland outline path (for RViz visualisation) is still generated via
 * polygon_utils offset_polygon so it is independent of the F2C pipeline.
 *
 * Action feedback is published at each pipeline phase so the BT node can
 * display progress and detect hangs early.
 */

#include "mowgli_coverage_planner/coverage_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// Fields2Cover single-include header
#include "fields2cover.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "mowgli_coverage_planner/polygon_utils.hpp"

namespace mowgli_coverage_planner
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CoveragePlannerNode::CoveragePlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("coverage_planner_node", options)
{
  // Declare parameters with defaults.
  tool_width_          = declare_parameter<double>("tool_width", 0.18);
  headland_passes_     = declare_parameter<int>("headland_passes", 2);
  headland_width_      = declare_parameter<double>("headland_width", 0.18);
  default_mow_angle_   = declare_parameter<double>("default_mow_angle", -1.0);
  path_spacing_        = declare_parameter<double>("path_spacing", 0.18);
  min_turning_radius_  = declare_parameter<double>("min_turning_radius", 0.3);
  map_frame_           = declare_parameter<std::string>("map_frame", "map");

  // Validate parameters at startup.
  if (tool_width_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "tool_width must be positive; clamping to 0.01 m");
    tool_width_ = 0.01;
  }
  if (headland_passes_ < 0) {
    RCLCPP_WARN(get_logger(), "headland_passes must be >= 0; setting to 0");
    headland_passes_ = 0;
  }
  if (headland_width_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "headland_width must be positive; clamping to tool_width");
    headland_width_ = tool_width_;
  }
  if (path_spacing_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "path_spacing must be positive; clamping to tool_width");
    path_spacing_ = tool_width_;
  }
  if (min_turning_radius_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "min_turning_radius must be positive; clamping to 0.1 m");
    min_turning_radius_ = 0.1;
  }

  RCLCPP_INFO(
    get_logger(),
    "CoveragePlannerNode: tool_width=%.3f m, headland_passes=%d, "
    "headland_width=%.3f m, path_spacing=%.3f m, min_turning_radius=%.3f m, frame='%s'",
    tool_width_, headland_passes_, headland_width_,
    path_spacing_, min_turning_radius_, map_frame_.c_str());

  // Publishers (transient-local so late subscribers receive the last message).
  const auto qos = rclcpp::QoS(1).transient_local();
  path_pub_    = create_publisher<nav_msgs::msg::Path>("~/coverage_path", qos);
  outline_pub_ = create_publisher<nav_msgs::msg::Path>("~/coverage_outline", qos);

  // Primary interface: PlanCoverage action server.
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<PlanCoverageAction>(
    this,
    "~/plan_coverage",
    std::bind(&CoveragePlannerNode::handle_goal,     this, _1, _2),
    std::bind(&CoveragePlannerNode::handle_cancel,   this, _1),
    std::bind(&CoveragePlannerNode::handle_accepted, this, _1));

  RCLCPP_INFO(get_logger(), "Coverage planner ready — action: ~/plan_coverage");
}

// ---------------------------------------------------------------------------
// Action server: handle_goal
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CoveragePlannerNode::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlanCoverageAction::Goal> /*goal*/)
{
  RCLCPP_INFO(get_logger(), "PlanCoverage action: goal received, accepting");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ---------------------------------------------------------------------------
// Action server: handle_cancel
// ---------------------------------------------------------------------------

rclcpp_action::CancelResponse CoveragePlannerNode::handle_cancel(
  const std::shared_ptr<GoalHandlePlanCoverage> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "PlanCoverage action: cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

// ---------------------------------------------------------------------------
// Action server: handle_accepted
// ---------------------------------------------------------------------------

void CoveragePlannerNode::handle_accepted(
  const std::shared_ptr<GoalHandlePlanCoverage> goal_handle)
{
  // Detach execution to a new thread so the action server callback returns
  // immediately.  The thread owns the goal_handle shared_ptr via capture.
  std::thread(
    [this, goal_handle]() {execute(goal_handle);})
  .detach();
}

// ---------------------------------------------------------------------------
// Action server: execute  (Fields2Cover v2 pipeline)
// ---------------------------------------------------------------------------

void CoveragePlannerNode::execute(
  const std::shared_ptr<GoalHandlePlanCoverage> goal_handle)
{
  const auto goal    = goal_handle->get_goal();
  auto result        = std::make_shared<PlanCoverageAction::Result>();
  auto feedback      = std::make_shared<PlanCoverageAction::Feedback>();

  // ---- Helper: publish feedback -------------------------------------------
  auto publish_feedback = [&](float progress, const std::string & phase) {
    if (goal_handle->is_canceling()) {return;}
    feedback->progress_percent = progress;
    feedback->phase            = phase;
    goal_handle->publish_feedback(feedback);
    RCLCPP_DEBUG(get_logger(), "PlanCoverage [%.0f%%] %s", progress, phase.c_str());
  };

  // ---- Clear previous coverage path (transient_local) so Foxglove / RViz
  //       don't show stale data from the last run.
  {
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = map_frame_;
    path_pub_->publish(empty_path);
  }

  // ---- Validate outer boundary --------------------------------------------
  const Polygon2D outer = geometry_polygon_to_points(goal->outer_boundary);

  if (outer.size() < 3) {
    result->success = false;
    result->message = "outer_boundary must have at least 3 vertices";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const double outer_area = std::abs(polygon_area(outer));
  if (outer_area < 1e-6) {
    result->success = false;
    result->message = "outer_boundary has zero or near-zero area";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  // ---- Headland outline (polygon_utils, for visualisation) ----------------
  publish_feedback(5.0f, "headland");

  nav_msgs::msg::Path outline_path;
  if (!goal->skip_outline) {
    outline_path = generate_outline_path(outer, map_frame_);
  }

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Cancelled during headland outline generation";
    goal_handle->canceled(result);
    return;
  }

  // ---- Build Fields2Cover types -------------------------------------------
  //
  // f2c::types::LinearRing requires the ring to be explicitly closed (first
  // point repeated at the end).

  F2CLinearRing ring;
  for (const auto & pt : outer) {
    ring.addPoint(F2CPoint(pt.first, pt.second));
  }
  // Close the ring by repeating the first vertex.
  ring.addPoint(F2CPoint(outer.front().first, outer.front().second));

  F2CCell  cell(ring);

  // Add obstacle polygons as interior rings (holes) in the cell.
  // Fields2Cover will clip swaths around these holes so no coverage
  // path crosses an obstacle.
  for (const auto & obs_polygon : goal->obstacles) {
    if (obs_polygon.points.size() < 3) {
      continue;
    }
    F2CLinearRing obs_ring;
    for (const auto & pt : obs_polygon.points) {
      obs_ring.addPoint(F2CPoint(
        static_cast<double>(pt.x), static_cast<double>(pt.y)));
    }
    // Close the ring.
    obs_ring.addPoint(F2CPoint(
      static_cast<double>(obs_polygon.points.front().x),
      static_cast<double>(obs_polygon.points.front().y)));
    cell.addRing(obs_ring);
  }

  F2CCells cells(cell);

  // Robot model: width = tool_width_, minimum turning radius for Dubins.
  F2CRobot robot(tool_width_);
  robot.setMinTurningRadius(min_turning_radius_);

  // ---- Phase: headland (F2C) ----------------------------------------------
  publish_feedback(15.0f, "headland");

  f2c::hg::ConstHL headland_gen;
  F2CCells inner_cells;

  try {
    // Ensure headland is at least 2.5x the turning radius so Dubins
    // turn curves stay within the field boundary (F2C issue #76).
    const double min_headland = 2.5 * min_turning_radius_;
    const double requested_headland =
      static_cast<double>(headland_passes_) * headland_width_;
    const double total_headland = std::max(requested_headland, min_headland);

    if (total_headland > requested_headland) {
      RCLCPP_INFO(
        get_logger(),
        "Headland increased from %.3f m to %.3f m (2.5x turning radius) "
        "to keep turns inside boundary",
        requested_headland, total_headland);
    }

    inner_cells = headland_gen.generateHeadlands(cells, total_headland);
  } catch (const std::exception & ex) {
    result->success = false;
    result->message = std::string("F2C headland generation failed: ") + ex.what();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  if (inner_cells.size() == 0) {
    result->success = false;
    result->message =
      "Headland contraction collapsed the inner area. "
      "Reduce headland_passes or headland_width.";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Cancelled during F2C headland generation";
    goal_handle->canceled(result);
    return;
  }

  // ---- Phase: swaths -------------------------------------------------------
  publish_feedback(35.0f, "swaths");

  // Select swath angle.  If the user requested auto-optimize or the default is
  // unset, pass 0.0 to BruteForce which will search the optimal angle itself.
  // Otherwise convert the requested angle to radians.
  const double mow_angle_deg = goal->mow_angle_deg;
  double swath_angle_rad = 0.0;
  bool auto_angle = (mow_angle_deg < 0.0 && default_mow_angle_ < 0.0);
  if (!auto_angle) {
    const double deg = (mow_angle_deg >= 0.0) ? mow_angle_deg : default_mow_angle_;
    swath_angle_rad = deg * M_PI / 180.0;
    RCLCPP_INFO(get_logger(), "Using mowing angle: %.1f deg", deg);
  } else {
    RCLCPP_INFO(get_logger(), "F2C BruteForce will auto-optimize mowing angle");
  }

  f2c::sg::BruteForce swath_gen;
  f2c::obj::NSwath n_swath_obj;
  F2CSwaths swaths;

  try {
    if (auto_angle) {
      // BruteForce::generateBestSwaths searches over all angles using NSwath objective.
      swaths = swath_gen.generateBestSwaths(n_swath_obj, path_spacing_, inner_cells.getGeometry(0));
    } else {
      swaths = swath_gen.generateSwaths(swath_angle_rad, path_spacing_,
                                        inner_cells.getGeometry(0));
    }
  } catch (const std::exception & ex) {
    result->success = false;
    result->message = std::string("F2C swath generation failed: ") + ex.what();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  if (swaths.size() == 0) {
    result->success = false;
    result->message =
      "F2C swath generator produced no swaths — polygon may be too small "
      "relative to path_spacing.";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(get_logger(), "F2C: %zu swaths generated", swaths.size());

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Cancelled during swath generation";
    goal_handle->canceled(result);
    return;
  }

  // ---- Phase: routing ------------------------------------------------------
  publish_feedback(55.0f, "routing");

  f2c::rp::BoustrophedonOrder route_planner;
  F2CSwaths route;

  try {
    route = route_planner.genSortedSwaths(swaths);
  } catch (const std::exception & ex) {
    result->success = false;
    result->message = std::string("F2C route planning failed: ") + ex.what();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Cancelled during route planning";
    goal_handle->canceled(result);
    return;
  }

  // ---- Phase: path planning -------------------------------------------------
  // Use Fields2Cover's path planner to generate the complete path including
  // turn connections between swaths.  With min_turning_radius ~0.01 (diff-drive
  // can turn in place), Dubins curves produce essentially point turns that
  // stay well within the field boundary.
  publish_feedback(75.0f, "path_planning");

  if (route.size() == 0) {
    result->success = false;
    result->message = "F2C route planner returned no swaths";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->message = "Cancelled during path planning";
    goal_handle->canceled(result);
    return;
  }

  // Generate complete path with turns using F2C path planner.
  f2c::pp::PathPlanning path_planner;
  f2c::pp::DubinsCurves dubins_turn;
  F2CPath f2c_path;

  try {
    f2c_path = path_planner.planPath(robot, route, dubins_turn);
    // Discretize to get dense waypoints (0.10 m spacing for smooth Nav2 tracking).
    // Too-dense spacing (< 0.10m) can cause MPPI to prune all remaining waypoints
    // when it detects the closest point jumps ahead in the path.
    f2c_path.discretize(0.10);
  } catch (const std::exception & ex) {
    result->success = false;
    result->message = std::string("F2C path planning failed: ") + ex.what();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  // ---- Convert F2CPath states to nav_msgs::msg::Path ----------------------
  publish_feedback(90.0f, "path_planning");

  nav_msgs::msg::Path swath_path;
  swath_path.header.frame_id = map_frame_;
  swath_path.header.stamp    = this->now();

  for (size_t i = 0; i < f2c_path.size(); ++i) {
    const auto & state = f2c_path.getState(i);
    swath_path.poses.push_back(
      make_pose(state.point.getX(), state.point.getY(), state.angle, map_frame_));
  }

  RCLCPP_INFO(get_logger(),
    "Coverage path: %zu waypoints (%zu swaths, F2C path length: %.1f m)",
    swath_path.poses.size(), route.size(), f2c_path.length());

  // ---- Coverage area: area of inner cells ---------------------------------
  double coverage_area = 0.0;
  for (size_t ci = 0; ci < static_cast<size_t>(inner_cells.size()); ++ci) {
    coverage_area += std::abs(inner_cells.getGeometry(static_cast<int>(ci)).area());
  }

  // ---- Populate result ----------------------------------------------------
  result->success        = true;
  result->message        = "Coverage path generated successfully via Fields2Cover";
  result->path           = swath_path;
  result->outline_path   = outline_path;
  result->total_distance =
    compute_path_length(swath_path) + compute_path_length(outline_path);
  result->coverage_area  = coverage_area;

  // Publish for visualisation.
  path_pub_->publish(swath_path);
  outline_pub_->publish(outline_path);

  RCLCPP_INFO(
    get_logger(),
    "Coverage plan: %zu path states, %.1f m total distance, %.2f m² inner area",
    swath_path.poses.size(),
    result->total_distance,
    result->coverage_area);

  publish_feedback(100.0f, "path_planning");
  goal_handle->succeed(result);
}

// ---------------------------------------------------------------------------
// generate_outline_path  (uses polygon_utils — unchanged)
// ---------------------------------------------------------------------------

nav_msgs::msg::Path CoveragePlannerNode::generate_outline_path(
  const Polygon2D & outer,
  const std::string & frame) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp    = this->now();

  for (int pass = 1; pass <= headland_passes_; ++pass) {
    const double inset = static_cast<double>(pass) * headland_width_;
    const Polygon2D ring = (pass == 1)
      ? offset_polygon(outer, headland_width_ * 0.5)
      : offset_polygon(outer, inset - headland_width_ * 0.5);

    if (ring.size() < 3) {
      break;  // contraction collapsed
    }

    for (std::size_t i = 0; i < ring.size(); ++i) {
      const Point2D & curr = ring[i];
      const Point2D & next = ring[(i + 1) % ring.size()];
      const double yaw = std::atan2(next.second - curr.second, next.first - curr.first);
      path.poses.push_back(make_pose(curr.first, curr.second, yaw, frame));
    }
    // Close the ring.
    if (!ring.empty()) {
      const Point2D & last  = ring.back();
      const Point2D & first = ring.front();
      const double yaw = std::atan2(first.second - last.second, first.first - last.first);
      path.poses.push_back(make_pose(last.first, last.second, yaw, frame));
    }
  }

  return path;
}

// ---------------------------------------------------------------------------
// compute_path_length
// ---------------------------------------------------------------------------

double CoveragePlannerNode::compute_path_length(const nav_msgs::msg::Path & path)
{
  double total = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    total += std::hypot(b.x - a.x, b.y - a.y);
  }
  return total;
}

// ---------------------------------------------------------------------------
// make_pose
// ---------------------------------------------------------------------------

geometry_msgs::msg::PoseStamped CoveragePlannerNode::make_pose(
  double x, double y, double yaw, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  // Use time 0 so TF lookups use the latest available transform rather than
  // requiring a transform at a specific (possibly stale) timestamp.
  pose.header.stamp    = rclcpp::Time(0);
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

}  // namespace mowgli_coverage_planner
