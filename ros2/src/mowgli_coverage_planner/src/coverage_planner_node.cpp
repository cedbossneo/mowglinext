// Copyright 2026 Mowgli Project
//
// Licensed under the GNU General Public License, version 3 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.html

/**
 * @file coverage_planner_node.cpp
 * @brief Coverage path planner using Fields2Cover v2.
 *
 * F2C pipeline:
 *   1. Validate the input boundary polygon.
 *   2. Convert ROS Polygon to f2c::types::Field / Cells.
 *   3. Generate headlands via f2c::hg::ConstHL.
 *   4. Decompose non-convex cells via f2c::decomp::BoustrophedonDecomp (v2).
 *   5. Generate swaths per cell via f2c::sg::BruteForce.
 *   6. Order swaths via f2c::rp::BoustrophedonOrder.
 *   7. Generate smooth turns via f2c::pp::ReedsSheppCurvesHC.
 *   8. Convert F2CPath to nav_msgs::Path with swath endpoint metadata.
 */

#include "mowgli_coverage_planner/coverage_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "fields2cover.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "mowgli_coverage_planner/polygon_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ogr_geometry.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_coverage_planner
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CoveragePlannerNode::CoveragePlannerNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("coverage_planner_node", options)
{
  tool_width_ = declare_parameter<double>("tool_width", 0.18);
  headland_passes_ = declare_parameter<int>("headland_passes", 2);
  headland_width_ = declare_parameter<double>("headland_width", 0.18);
  default_mow_angle_ = declare_parameter<double>("default_mow_angle", -1.0);
  path_spacing_ = declare_parameter<double>("path_spacing", 0.18);
  min_turning_radius_ = declare_parameter<double>("min_turning_radius", 0.01);
  decompose_cells_ = declare_parameter<bool>("decompose_cells", true);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  mowing_speed_ = declare_parameter<double>("mowing_speed", 0.15);
  transit_speed_ = declare_parameter<double>("transit_speed", 0.3);
  route_graph_filepath_ =
      declare_parameter<std::string>("route_graph_filepath", "/tmp/mowing_route.geojson");

  if (tool_width_ <= 0.0)
  {
    RCLCPP_WARN(get_logger(), "tool_width must be positive; clamping to 0.01 m");
    tool_width_ = 0.01;
  }
  if (headland_passes_ < 0)
  {
    RCLCPP_WARN(get_logger(), "headland_passes must be >= 0; setting to 0");
    headland_passes_ = 0;
  }
  if (headland_width_ <= 0.0)
  {
    RCLCPP_WARN(get_logger(), "headland_width must be positive; clamping to tool_width");
    headland_width_ = tool_width_;
  }
  if (path_spacing_ <= 0.0)
  {
    RCLCPP_WARN(get_logger(), "path_spacing must be positive; clamping to tool_width");
    path_spacing_ = tool_width_;
  }

  RCLCPP_INFO(get_logger(),
              "CoveragePlannerNode: tool_width=%.3f m, headland_passes=%d, "
              "headland_width=%.3f m, path_spacing=%.3f m, decompose=%s, frame='%s'",
              tool_width_,
              headland_passes_,
              headland_width_,
              path_spacing_,
              decompose_cells_ ? "true" : "false",
              map_frame_.c_str());

  // Publishers for visualization.
  path_pub_ =
      create_publisher<nav_msgs::msg::Path>("~/coverage_path", rclcpp::QoS(1).transient_local());
  outline_pub_ =
      create_publisher<nav_msgs::msg::Path>("~/coverage_outline", rclcpp::QoS(1).transient_local());
  route_graph_pub_ =
      create_publisher<std_msgs::msg::String>("~/route_graph", rclcpp::QoS(1).transient_local());

  costmap_min_cluster_size_ = declare_parameter<int>("costmap_min_cluster_size", 10);
  costmap_obstacle_inflation_ = declare_parameter<double>("costmap_obstacle_inflation", 0.10);
  use_costmap_obstacles_ = declare_parameter<bool>("use_costmap_obstacles", false);

  // Subscribe to global costmap for obstacle extraction.
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap",
      rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        latest_costmap_ = msg;
      });

  // Subscribe to tracked obstacles from obstacle_tracker.
  // Persistent obstacles are merged as holes in the F2C cell so
  // coverage paths are planned around them.
  obstacle_sub_ = create_subscription<mowgli_interfaces::msg::ObstacleArray>(
      "/obstacle_tracker/obstacles",
      rclcpp::QoS(10),
      [this](mowgli_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        tracked_obstacles_.clear();
        for (const auto &obs : msg->obstacles)
        {
          // Only use persistent obstacles (not transient detections)
          if (obs.status == mowgli_interfaces::msg::TrackedObstacle::PERSISTENT &&
              obs.polygon.points.size() >= 3)
          {
            tracked_obstacles_.push_back(obs.polygon);
          }
        }
      });

  // Action server.
  action_server_ = rclcpp_action::create_server<PlanCoverageAction>(
      this,
      "~/plan_coverage",
      std::bind(
          &CoveragePlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CoveragePlannerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&CoveragePlannerNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "CoveragePlannerNode ready — action server on ~/plan_coverage");
}

// ---------------------------------------------------------------------------
// Action server callbacks
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CoveragePlannerNode::handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PlanCoverageAction::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CoveragePlannerNode::handle_cancel(
    const std::shared_ptr<GoalHandlePlanCoverage>)
{
  RCLCPP_INFO(get_logger(), "Coverage plan cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CoveragePlannerNode::handle_accepted(const std::shared_ptr<GoalHandlePlanCoverage> goal_handle)
{
  std::thread{std::bind(&CoveragePlannerNode::execute, this, goal_handle)}.detach();
}

// ---------------------------------------------------------------------------
// Feedback helper
// ---------------------------------------------------------------------------

static void publish_feedback_impl(
    const std::shared_ptr<CoveragePlannerNode::GoalHandlePlanCoverage> &goal_handle,
    float progress,
    const std::string &phase)
{
  auto fb = std::make_shared<CoveragePlannerNode::PlanCoverageAction::Feedback>();
  fb->progress_percent = progress;
  fb->phase = phase;
  goal_handle->publish_feedback(fb);
}

// ---------------------------------------------------------------------------
// Execute — main F2C pipeline
// ---------------------------------------------------------------------------

void CoveragePlannerNode::execute(const std::shared_ptr<GoalHandlePlanCoverage> goal_handle)
{
  auto result = std::make_shared<PlanCoverageAction::Result>();
  const auto goal = goal_handle->get_goal();

  auto publish_feedback = [&](float pct, const std::string &phase)
  {
    publish_feedback_impl(goal_handle, pct, phase);
  };

  // ---- Validate input -------------------------------------------------------
  const auto &boundary = goal->outer_boundary;
  if (boundary.points.size() < 3)
  {
    result->success = false;
    result->message = "outer_boundary must have at least 3 vertices";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  // Convert to internal polygon type for outline generation.
  Polygon2D outer;
  for (const auto &pt : boundary.points)
  {
    outer.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.y));
  }

  const double outer_area = std::abs(polygon_area(outer));
  if (outer_area < 1e-6)
  {
    result->success = false;
    result->message = "outer_boundary has zero or near-zero area";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  // ---- Headland outline (polygon_utils, for visualization) ------------------
  publish_feedback(5.0f, "headland");

  nav_msgs::msg::Path outline_path;
  if (!goal->skip_outline)
  {
    outline_path = generate_outline_path(outer, map_frame_);
  }

  if (goal_handle->is_canceling())
  {
    result->success = false;
    result->message = "Cancelled during headland outline generation";
    goal_handle->canceled(result);
    return;
  }

  // ---- Build Fields2Cover types ---------------------------------------------
  F2CLinearRing ring;
  for (const auto &pt : outer)
  {
    ring.addPoint(F2CPoint(pt.first, pt.second));
  }
  ring.addPoint(F2CPoint(outer.front().first, outer.front().second));

  F2CCell cell(ring);

  // Add obstacle polygons as interior rings (holes).
  // Sources: 1) explicit obstacles from action goal, 2) tracked obstacles
  // from obstacle_tracker (persistent LiDAR detections).
  auto add_obstacle_ring = [&cell, this](const geometry_msgs::msg::Polygon &obs_polygon)
  {
    if (obs_polygon.points.size() < 3)
    {
      return;
    }
    F2CLinearRing obs_ring;
    for (const auto &pt : obs_polygon.points)
    {
      obs_ring.addPoint(F2CPoint(static_cast<double>(pt.x), static_cast<double>(pt.y)));
    }
    obs_ring.addPoint(F2CPoint(static_cast<double>(obs_polygon.points.front().x),
                               static_cast<double>(obs_polygon.points.front().y)));
    cell.addRing(obs_ring);
  };

  // 1) Obstacles from action goal (passed by BT from map_server).
  for (const auto &obs : goal->obstacles)
  {
    add_obstacle_ring(obs);
  }

  // 2) Persistent obstacles from obstacle_tracker (LiDAR detections).
  size_t tracked_count = 0;
  {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    for (const auto &obs : tracked_obstacles_)
    {
      add_obstacle_ring(obs);
      ++tracked_count;
    }
  }

  // 3) Obstacles extracted from global costmap (lethal cells).
  //    Disabled by default — the collision_monitor handles real-time obstacle
  //    avoidance during navigation. Costmap obstacles from SLAM map walls
  //    fill the mowing area and collapse the coverage plan.
  std::vector<geometry_msgs::msg::Polygon> costmap_obstacles;
  if (use_costmap_obstacles_)
  {
    costmap_obstacles = extract_costmap_obstacles();
    for (const auto &obs : costmap_obstacles)
    {
      add_obstacle_ring(obs);
    }
  }

  RCLCPP_INFO(get_logger(),
              "F2C: %zu goal + %zu tracked + %zu costmap = %zu total obstacle holes",
              goal->obstacles.size(),
              tracked_count,
              costmap_obstacles.size(),
              goal->obstacles.size() + tracked_count + costmap_obstacles.size());

  F2CCells cells(cell);

  // Robot model: width and operational width (coverage width).
  F2CRobot robot(tool_width_, tool_width_);
  robot.setMinTurningRadius(min_turning_radius_);

  // ---- Phase: headland (F2C) ------------------------------------------------
  publish_feedback(15.0f, "headland");

  f2c::hg::ConstHL headland_gen;
  F2CCells inner_cells;

  try
  {
    const double total_headland = static_cast<double>(headland_passes_) * headland_width_;
    inner_cells = headland_gen.generateHeadlands(cells, total_headland);
  }
  catch (const std::exception &ex)
  {
    result->success = false;
    result->message = std::string("F2C headland generation failed: ") + ex.what();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  if (inner_cells.size() == 0)
  {
    result->success = false;
    result->message =
        "Headland contraction collapsed the inner area. "
        "Reduce headland_passes or headland_width.";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  if (goal_handle->is_canceling())
  {
    result->success = false;
    result->message = "Cancelled during F2C headland generation";
    goal_handle->canceled(result);
    return;
  }

  // Subtract obstacles from inner_cells using GDAL geometry difference.
  // This cleanly clips obstacle regions from the cell, even when obstacles
  // overlap with the cell boundary. The result may split a cell into
  // multiple sub-cells around the obstacles.
  const size_t num_obstacle_rings = (cell.size() > 1) ? cell.size() - 1 : 0;
  F2CCells work_cells = inner_cells;

  if (num_obstacle_rings > 0)
  {
    try
    {
      for (int ci = 0; ci < static_cast<int>(work_cells.size()); ++ci)
      {
        // Get the GDAL geometry handle for the inner cell.
        OGRGeometry *cell_geom = work_cells.getGeometry(ci).get()->clone();

        for (size_t ri = 0; ri < num_obstacle_rings; ++ri)
        {
          // Build an OGR polygon from the obstacle ring.
          OGRPolygon obs_poly;
          OGRLinearRing obs_lr;
          F2CLinearRing f2c_ring = cell.getInteriorRing(ri);
          for (size_t pi = 0; pi < f2c_ring.size(); ++pi)
          {
            obs_lr.addPoint(f2c_ring.getX(pi), f2c_ring.getY(pi));
          }
          obs_lr.closeRings();
          obs_poly.addRing(&obs_lr);

          // Buffer the obstacle slightly for clearance.
          OGRGeometry *buffered = obs_poly.Buffer(costmap_obstacle_inflation_);
          if (!buffered)
          {
            continue;
          }

          // Subtract obstacle from cell.
          OGRGeometry *result = cell_geom->Difference(buffered);
          OGRGeometryFactory::destroyGeometry(buffered);
          if (result)
          {
            OGRGeometryFactory::destroyGeometry(cell_geom);
            cell_geom = result;
          }
        }

        // Convert OGR result back to F2C cells.
        // Helper to build F2CCell from an OGRPolygon.
        auto ogr_poly_to_f2c = [](const OGRPolygon *poly) -> F2CCell
        {
          F2CLinearRing outer;
          const auto *ext = poly->getExteriorRing();
          for (int pi = 0; pi < ext->getNumPoints(); ++pi)
          {
            outer.addPoint(F2CPoint(ext->getX(pi), ext->getY(pi)));
          }
          F2CCell c(outer);
          // Add interior rings (remaining obstacles)
          for (int iri = 0; iri < poly->getNumInteriorRings(); ++iri)
          {
            const auto *ir = poly->getInteriorRing(iri);
            F2CLinearRing hole;
            for (int pi = 0; pi < ir->getNumPoints(); ++pi)
            {
              hole.addPoint(F2CPoint(ir->getX(pi), ir->getY(pi)));
            }
            c.addRing(hole);
          }
          return c;
        };

        F2CCells result_cells;
        if (cell_geom->getGeometryType() == wkbPolygon)
        {
          result_cells.addGeometry(ogr_poly_to_f2c(static_cast<const OGRPolygon *>(cell_geom)));
        }
        else if (cell_geom->getGeometryType() == wkbMultiPolygon)
        {
          auto *mp = static_cast<OGRMultiPolygon *>(cell_geom);
          for (int gi = 0; gi < mp->getNumGeometries(); ++gi)
          {
            result_cells.addGeometry(
                ogr_poly_to_f2c(static_cast<const OGRPolygon *>(mp->getGeometryRef(gi))));
          }
        }
        OGRGeometryFactory::destroyGeometry(cell_geom);

        // Replace work cell with the result (may now be multiple cells).
        if (result_cells.size() > 0)
        {
          work_cells.setGeometry(ci, result_cells.getGeometry(0));
          for (int ri2 = 1; ri2 < static_cast<int>(result_cells.size()); ++ri2)
          {
            work_cells.addGeometry(result_cells.getGeometry(ri2));
          }
        }
      }
      // Log ring counts for each work cell.
      for (int wci = 0; wci < static_cast<int>(work_cells.size()); ++wci)
      {
        RCLCPP_INFO(get_logger(),
                    "F2C: work cell %d has %zu rings (1 outer + %zu holes)",
                    wci,
                    work_cells.getGeometry(wci).size(),
                    work_cells.getGeometry(wci).size() > 0 ? work_cells.getGeometry(wci).size() - 1
                                                           : 0);
      }
      RCLCPP_INFO(get_logger(),
                  "F2C: subtracted %zu obstacles -> %d work cell(s)",
                  num_obstacle_rings,
                  static_cast<int>(work_cells.size()));
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(get_logger(),
                  "F2C: obstacle subtraction failed (%s), using cells without obstacles",
                  ex.what());
      work_cells = inner_cells;
    }
  }

  // ---- Phase: swaths --------------------------------------------------------
  publish_feedback(35.0f, "swaths");

  const double mow_angle_deg = goal->mow_angle_deg;
  double swath_angle_rad = 0.0;
  bool auto_angle = (mow_angle_deg < 0.0 && default_mow_angle_ < 0.0);
  if (!auto_angle)
  {
    const double deg = (mow_angle_deg >= 0.0) ? mow_angle_deg : default_mow_angle_;
    swath_angle_rad = deg * M_PI / 180.0;
    RCLCPP_INFO(get_logger(), "Using mowing angle: %.1f deg", deg);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "F2C BruteForce will auto-optimize mowing angle");
  }

  f2c::sg::BruteForce swath_gen;
  f2c::obj::NSwath n_swath_obj;

  // Generate swaths for each sub-cell after decomposition.
  F2CSwathsByCells all_swaths;

  try
  {
    for (int ci = 0; ci < static_cast<int>(work_cells.size()); ++ci)
    {
      F2CSwaths cell_swaths;
      if (auto_angle)
      {
        cell_swaths =
            swath_gen.generateBestSwaths(n_swath_obj, path_spacing_, work_cells.getGeometry(ci));
      }
      else
      {
        cell_swaths =
            swath_gen.generateSwaths(swath_angle_rad, path_spacing_, work_cells.getGeometry(ci));
      }
      all_swaths.push_back(cell_swaths);
    }
  }
  catch (const std::exception &ex)
  {
    result->success = false;
    result->message = std::string("F2C swath generation failed: ") + ex.what();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  // Count total swaths.
  size_t total_swaths = 0;
  for (const auto &cs : all_swaths)
  {
    total_swaths += cs.size();
  }

  if (total_swaths == 0)
  {
    result->success = false;
    result->message =
        "F2C swath generator produced no swaths — polygon may be too small "
        "relative to path_spacing.";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(get_logger(),
              "F2C: %zu swaths across %zu sub-cell(s)",
              total_swaths,
              all_swaths.size());

  if (goal_handle->is_canceling())
  {
    result->success = false;
    result->message = "Cancelled during swath generation";
    goal_handle->canceled(result);
    return;
  }

  // ---- Phase: routing -------------------------------------------------------
  publish_feedback(55.0f, "routing");

  // Flatten all swaths and sort with BoustrophedonOrder.
  // (OR-tools RoutePlannerBase requires the ortools library; use BoustrophedonOrder
  // as the robust default — can be upgraded when ortools is added to the Docker image.)
  F2CSwaths flat_swaths;
  for (const auto &cs : all_swaths)
  {
    for (const auto &s : cs)
    {
      flat_swaths.push_back(s);
    }
  }

  f2c::rp::BoustrophedonOrder route_planner;
  F2CSwaths route;

  try
  {
    route = route_planner.genSortedSwaths(flat_swaths);
  }
  catch (const std::exception &ex)
  {
    result->success = false;
    result->message = std::string("F2C route planning failed: ") + ex.what();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  if (goal_handle->is_canceling())
  {
    result->success = false;
    result->message = "Cancelled during route planning";
    goal_handle->canceled(result);
    return;
  }

  // ---- Phase: path planning -------------------------------------------------
  // Use F2C's PathPlanning to generate the complete coverage path including
  // tight ReedsShepp turns between swaths. With min_turning_radius ~0.05m,
  // the turns are nearly point-turns — appropriate for a diff-drive robot.
  // The result is a single continuous path: swaths + turns, followed via
  // a single FollowPath call (no separate transit phase).
  publish_feedback(75.0f, "path_planning");

  if (route.size() == 0)
  {
    result->success = false;
    result->message = "F2C route planner returned no swaths";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  // Dubins curves: forward-only arcs. Produces clean turns without the
  // spiral patterns that ReedsShepp creates for long obstacle transitions.
  // A diff-drive robot handles the required heading changes via in-place
  // rotation (RPP's RotationShimController).
  f2c::pp::DubinsCurves turn_planner;
  turn_planner.setDiscretization(0.10);  // 10cm waypoint spacing in turns

  f2c::pp::PathPlanning path_planner;
  F2CPath f2c_path;

  try
  {
    f2c_path = path_planner.planPath(robot, route, turn_planner);
  }
  catch (const std::exception &ex)
  {
    result->success = false;
    result->message = std::string("F2C path planning failed: ") + ex.what();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  // Convert F2CPath to nav_msgs::Path + extract swath endpoints.
  nav_msgs::msg::Path swath_path;
  swath_path.header.frame_id = map_frame_;
  swath_path.header.stamp = this->now();

  std::vector<geometry_msgs::msg::Point> swath_starts;
  std::vector<geometry_msgs::msg::Point> swath_ends;

  bool in_swath = false;
  geometry_msgs::msg::Point current_swath_start;

  for (size_t si = 0; si < f2c_path.getStates().size(); ++si)
  {
    const auto &state = f2c_path.getStates()[si];
    const double x = state.point.getX();
    const double y = state.point.getY();
    const double yaw = state.angle;

    swath_path.poses.push_back(make_pose(x, y, yaw, map_frame_));

    // Track swath/turn transitions for blade on/off control.
    if (state.type == f2c::types::PathSectionType::SWATH && !in_swath)
    {
      in_swath = true;
      current_swath_start.x = x;
      current_swath_start.y = y;
    }
    else if (state.type != f2c::types::PathSectionType::SWATH && in_swath)
    {
      in_swath = false;
      swath_starts.push_back(current_swath_start);
      geometry_msgs::msg::Point end_pt;
      if (si > 0)
      {
        end_pt.x = f2c_path.getStates()[si - 1].point.getX();
        end_pt.y = f2c_path.getStates()[si - 1].point.getY();
      }
      else
      {
        end_pt.x = x;
        end_pt.y = y;
      }
      swath_ends.push_back(end_pt);
    }
  }
  // Handle path ending during a swath.
  if (in_swath && !f2c_path.getStates().empty())
  {
    swath_starts.push_back(current_swath_start);
    geometry_msgs::msg::Point end_pt;
    end_pt.x = f2c_path.getStates().back().point.getX();
    end_pt.y = f2c_path.getStates().back().point.getY();
    swath_ends.push_back(end_pt);
  }

  RCLCPP_INFO(get_logger(),
              "Coverage path: %zu waypoints (%zu swaths, %.1f m total, turns: Dubins r=%.3fm)",
              swath_path.poses.size(),
              swath_starts.size(),
              f2c_path.length(),
              min_turning_radius_);

  // ---- Coverage area --------------------------------------------------------
  double coverage_area = 0.0;
  for (size_t ci = 0; ci < static_cast<size_t>(work_cells.size()); ++ci)
  {
    coverage_area += std::abs(work_cells.getGeometry(static_cast<int>(ci)).area());
  }

  // ---- Populate result ------------------------------------------------------
  result->success = true;
  result->message = "Coverage path generated via Fields2Cover v2";
  result->path = swath_path;
  result->outline_path = outline_path;
  result->total_distance = f2c_path.length() + compute_path_length(outline_path);
  result->coverage_area = coverage_area;
  result->swath_starts = swath_starts;
  result->swath_ends = swath_ends;

  path_pub_->publish(swath_path);
  outline_pub_->publish(outline_path);

  // Generate and publish GeoJSON route graph for Nav2 route_server.
  publish_route_graph(swath_starts, swath_ends, mowing_speed_, transit_speed_);

  RCLCPP_INFO(get_logger(),
              "Coverage plan: %zu path states, %zu swaths, %.1f m total, %.2f m2 area",
              swath_path.poses.size(),
              swath_starts.size(),
              result->total_distance,
              result->coverage_area);

  publish_feedback(100.0f, "path_planning");
  goal_handle->succeed(result);
}

// ---------------------------------------------------------------------------
// publish_route_graph — convert swaths to GeoJSON for Nav2 route_server
// ---------------------------------------------------------------------------

void CoveragePlannerNode::publish_route_graph(
    const std::vector<geometry_msgs::msg::Point> &swath_starts,
    const std::vector<geometry_msgs::msg::Point> &swath_ends,
    double mowing_speed,
    double transit_speed)
{
  if (swath_starts.empty() || swath_starts.size() != swath_ends.size())
  {
    RCLCPP_WARN(get_logger(), "No swaths to convert to route graph");
    return;
  }

  // Build node list: for N swaths we have 2N unique endpoints.
  // Node IDs: swath i start = 2*i, swath i end = 2*i+1.
  //
  // Edge types:
  //   - Swath edge: connects start[i] -> end[i]  (blade_on, mowing_speed)
  //   - Turn edge:  connects end[i] -> start[i+1] (blade_off, transit_speed)

  std::ostringstream json;
  json << std::fixed;
  json.precision(6);

  json << "{\n";
  json << "  \"type\": \"FeatureCollection\",\n";
  json << "  \"features\": [\n";

  const size_t n_swaths = swath_starts.size();
  const size_t n_nodes = 2 * n_swaths;
  const size_t n_edges = n_swaths + (n_swaths > 0 ? n_swaths - 1 : 0);
  bool first = true;

  // --- Point nodes ---
  for (size_t i = 0; i < n_swaths; ++i)
  {
    // Start node
    if (!first)
    {
      json << ",\n";
    }
    first = false;
    json << "    {\"type\": \"Feature\", \"geometry\": {\"type\": \"Point\", "
         << "\"coordinates\": [" << swath_starts[i].x << ", " << swath_starts[i].y
         << "]}, \"properties\": {\"id\": " << (2 * i) << "}}";

    // End node
    json << ",\n";
    json << "    {\"type\": \"Feature\", \"geometry\": {\"type\": \"Point\", "
         << "\"coordinates\": [" << swath_ends[i].x << ", " << swath_ends[i].y
         << "]}, \"properties\": {\"id\": " << (2 * i + 1) << "}}";
  }

  // --- Swath edges (blade_on) ---
  size_t edge_id = 0;
  for (size_t i = 0; i < n_swaths; ++i)
  {
    const size_t start_id = 2 * i;
    const size_t end_id = 2 * i + 1;
    const double dx = swath_ends[i].x - swath_starts[i].x;
    const double dy = swath_ends[i].y - swath_starts[i].y;
    const double cost = std::hypot(dx, dy);

    json << ",\n";
    json << "    {\"type\": \"Feature\", \"geometry\": {\"type\": \"MultiLineString\", "
         << "\"coordinates\": [[[" << swath_starts[i].x << ", " << swath_starts[i].y << "], ["
         << swath_ends[i].x << ", " << swath_ends[i].y << "]]]}, \"properties\": {"
         << "\"id\": " << edge_id++ << ", \"startid\": " << start_id << ", \"endid\": " << end_id
         << ", \"cost\": " << cost << ", \"speed_limit\": " << mowing_speed
         << ", \"operation\": \"blade_on\""
         << "}}";
  }

  // --- Turn edges (blade_off) ---
  for (size_t i = 0; i + 1 < n_swaths; ++i)
  {
    const size_t start_id = 2 * i + 1;  // end of swath i
    const size_t end_id = 2 * (i + 1);  // start of swath i+1
    const double dx = swath_starts[i + 1].x - swath_ends[i].x;
    const double dy = swath_starts[i + 1].y - swath_ends[i].y;
    const double cost = std::hypot(dx, dy);

    json << ",\n";
    json << "    {\"type\": \"Feature\", \"geometry\": {\"type\": \"MultiLineString\", "
         << "\"coordinates\": [[[" << swath_ends[i].x << ", " << swath_ends[i].y << "], ["
         << swath_starts[i + 1].x << ", " << swath_starts[i + 1].y << "]]]}, \"properties\": {"
         << "\"id\": " << edge_id++ << ", \"startid\": " << start_id << ", \"endid\": " << end_id
         << ", \"cost\": " << cost << ", \"speed_limit\": " << transit_speed
         << ", \"operation\": \"blade_off\""
         << "}}";
  }

  json << "\n  ]\n";
  json << "}\n";

  const std::string geojson = json.str();

  // Write to file.
  {
    std::ofstream ofs(route_graph_filepath_);
    if (ofs.is_open())
    {
      ofs << geojson;
      ofs.close();
      RCLCPP_INFO(get_logger(),
                  "Route graph: %zu nodes, %zu edges (%zu swaths + %zu turns) -> %s",
                  n_nodes,
                  n_edges,
                  n_swaths,
                  n_swaths > 0 ? n_swaths - 1 : size_t(0),
                  route_graph_filepath_.c_str());
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Failed to write route graph to %s", route_graph_filepath_.c_str());
    }
  }

  // Publish on topic.
  auto msg = std_msgs::msg::String();
  msg.data = geojson;
  route_graph_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// generate_outline_path
// ---------------------------------------------------------------------------

nav_msgs::msg::Path CoveragePlannerNode::generate_outline_path(const Polygon2D &outer,
                                                               const std::string &frame) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = this->now();

  for (int pass = 1; pass <= headland_passes_; ++pass)
  {
    const double inset = static_cast<double>(pass) * headland_width_;
    const Polygon2D ring = (pass == 1) ? offset_polygon(outer, headland_width_ * 0.5)
                                       : offset_polygon(outer, inset - headland_width_ * 0.5);

    if (ring.size() < 3)
    {
      break;
    }

    for (std::size_t i = 0; i < ring.size(); ++i)
    {
      const Point2D &curr = ring[i];
      const Point2D &next = ring[(i + 1) % ring.size()];
      const double yaw = std::atan2(next.second - curr.second, next.first - curr.first);
      path.poses.push_back(make_pose(curr.first, curr.second, yaw, frame));
    }
    if (!ring.empty())
    {
      const Point2D &last = ring.back();
      const Point2D &first = ring.front();
      const double yaw = std::atan2(first.second - last.second, first.first - last.first);
      path.poses.push_back(make_pose(last.first, last.second, yaw, frame));
    }
  }

  return path;
}

// ---------------------------------------------------------------------------
// compute_path_length
// ---------------------------------------------------------------------------

double CoveragePlannerNode::compute_path_length(const nav_msgs::msg::Path &path)
{
  double total = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i)
  {
    const auto &a = path.poses[i - 1].pose.position;
    const auto &b = path.poses[i].pose.position;
    total += std::hypot(b.x - a.x, b.y - a.y);
  }
  return total;
}

// ---------------------------------------------------------------------------
// make_pose
// ---------------------------------------------------------------------------

geometry_msgs::msg::PoseStamped CoveragePlannerNode::make_pose(double x,
                                                               double y,
                                                               double yaw,
                                                               const std::string &frame)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = rclcpp::Time(0);
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

// ---------------------------------------------------------------------------
// extract_costmap_obstacles — convert lethal costmap cells to polygons
// ---------------------------------------------------------------------------

std::vector<geometry_msgs::msg::Polygon> CoveragePlannerNode::extract_costmap_obstacles() const
{
  std::vector<geometry_msgs::msg::Polygon> obstacles;

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap = latest_costmap_;
  }

  if (!costmap || costmap->data.empty())
  {
    return obstacles;
  }

  const auto &info = costmap->info;
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);
  const double res = info.resolution;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;

  // Mark cells that are unsafe for the robot to traverse.
  // OccupancyGrid values: 0=free, 100=lethal, -1=unknown.
  // Use threshold of 50 to capture both lethal cells AND the inner
  // inflation zone — this gives F2C obstacle holes that include the
  // full inflated area the robot must avoid.
  std::vector<bool> visited(costmap->data.size(), false);
  const int8_t OBSTACLE_THRESHOLD = 50;

  auto idx = [width](int x, int y)
  {
    return y * width + x;
  };

  for (int sy = 0; sy < height; ++sy)
  {
    for (int sx = 0; sx < width; ++sx)
    {
      const int i = idx(sx, sy);
      if (visited[i] || costmap->data[i] < OBSTACLE_THRESHOLD)
      {
        continue;
      }

      // BFS flood fill to collect connected lethal cells.
      std::vector<std::pair<int, int>> cluster;
      std::vector<std::pair<int, int>> queue;
      queue.push_back({sx, sy});
      visited[i] = true;

      while (!queue.empty())
      {
        auto [cx, cy] = queue.back();
        queue.pop_back();
        cluster.push_back({cx, cy});

        // 4-connected neighbors
        for (auto [dx, dy] : std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
        {
          int nx = cx + dx;
          int ny = cy + dy;
          if (nx >= 0 && nx < width && ny >= 0 && ny < height)
          {
            int ni = idx(nx, ny);
            if (!visited[ni] && costmap->data[ni] >= OBSTACLE_THRESHOLD)
            {
              visited[ni] = true;
              queue.push_back({nx, ny});
            }
          }
        }
      }

      if (static_cast<int>(cluster.size()) < costmap_min_cluster_size_)
      {
        continue;  // Too small — noise
      }

      // Skip clusters that are too large — they're likely boundary walls.
      // Max individual obstacle area: 16m² (4m x 4m). Walls are typically
      // much larger (10m+ long). With inflated threshold, obstacles capture
      // the full inflation zone so they appear larger than the physical object.
      const int max_cluster_cells = static_cast<int>(16.0 / (res * res));
      if (static_cast<int>(cluster.size()) > max_cluster_cells)
      {
        continue;  // Too large — boundary wall or merged structure
      }

      // Compute axis-aligned bounding box of the cluster and inflate.
      double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
      for (const auto &[cx, cy] : cluster)
      {
        double wx = ox + (cx + 0.5) * res;
        double wy = oy + (cy + 0.5) * res;
        min_x = std::min(min_x, wx);
        max_x = std::max(max_x, wx);
        min_y = std::min(min_y, wy);
        max_y = std::max(max_y, wy);
      }

      // Inflate by obstacle_inflation
      min_x -= costmap_obstacle_inflation_;
      max_x += costmap_obstacle_inflation_;
      min_y -= costmap_obstacle_inflation_;
      max_y += costmap_obstacle_inflation_;

      // Create a rectangular polygon for this obstacle cluster.
      geometry_msgs::msg::Polygon poly;
      geometry_msgs::msg::Point32 p;
      p.z = 0.0f;

      p.x = static_cast<float>(min_x);
      p.y = static_cast<float>(min_y);
      poly.points.push_back(p);
      p.x = static_cast<float>(max_x);
      p.y = static_cast<float>(min_y);
      poly.points.push_back(p);
      p.x = static_cast<float>(max_x);
      p.y = static_cast<float>(max_y);
      poly.points.push_back(p);
      p.x = static_cast<float>(min_x);
      p.y = static_cast<float>(max_y);
      poly.points.push_back(p);

      obstacles.push_back(poly);
    }
  }

  return obstacles;
}

}  // namespace mowgli_coverage_planner
