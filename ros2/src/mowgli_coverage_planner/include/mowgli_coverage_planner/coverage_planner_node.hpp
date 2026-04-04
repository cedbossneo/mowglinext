// Copyright (C) 2024 Cedric
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * @file coverage_planner_node.hpp
 * @brief ROS2 node that provides coverage path planning via a ROS2 action interface.
 *
 * The node accepts an area boundary and a list of obstacle polygons, then
 * generates an efficient boustrophedon mowing path using the Fields2Cover v2
 * library:
 *   1. Headland generation via f2c::hg::ConstHL (configurable passes).
 *   2. Swath generation via f2c::sg::BruteForce (optimal angle search).
 *   3. Route planning via f2c::rp::BoustrophedonOrder.
 *   4. Smooth path via f2c::pp::DubinsCurves (respects robot turning radius).
 *
 * The headland outline path is still generated internally using polygon_utils
 * for visualisation purposes.
 */

#ifndef MOWGLI_COVERAGE_PLANNER__COVERAGE_PLANNER_NODE_HPP_
#define MOWGLI_COVERAGE_PLANNER__COVERAGE_PLANNER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_coverage_planner/polygon_utils.hpp"
#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "mowgli_interfaces/msg/obstacle_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_coverage_planner
{

/**
 * @brief Coverage path planner node.
 *
 * Advertises:
 *   - `~/plan_coverage` action server (PlanCoverage action)
 *   - `~/coverage_path` publisher (nav_msgs::Path, transient-local)
 *   - `~/coverage_outline` publisher (nav_msgs::Path, transient-local)
 */
class CoveragePlannerNode : public rclcpp::Node
{
public:
  using PlanCoverageAction = mowgli_interfaces::action::PlanCoverage;
  using GoalHandlePlanCoverage = rclcpp_action::ServerGoalHandle<PlanCoverageAction>;

  /**
   * @brief Construct the node, declare parameters, create publishers and
   *        action server.
   *
   * @param options Node options forwarded to rclcpp::Node.
   */
  explicit CoveragePlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // -------------------------------------------------------------------------
  // Action server callbacks
  // -------------------------------------------------------------------------

  /**
   * @brief Handle a new goal request.
   *
   * Always accepts the goal so that the BT node does not have to retry.
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const PlanCoverageAction::Goal> goal);

  /**
   * @brief Handle a cancel request.
   *
   * Accepts the cancellation — the execute thread will check for it.
   */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandlePlanCoverage> goal_handle);

  /**
   * @brief Spawn the execute callback on a detached thread.
   *
   * Using a detached thread keeps the action server callbacks non-blocking
   * while the F2C pipeline runs.
   */
  void handle_accepted(const std::shared_ptr<GoalHandlePlanCoverage> goal_handle);

  /**
   * @brief Execute the Fields2Cover planning pipeline and fill the result.
   *
   * Publishes feedback for each phase: "headland", "swaths", "routing",
   * and "path_planning". On completion, publishes the paths and fills
   * the action result.
   *
   * @param goal_handle Active goal handle for feedback / result publication.
   */
  void execute(const std::shared_ptr<GoalHandlePlanCoverage> goal_handle);

  // -------------------------------------------------------------------------
  // Internal planning helpers
  // -------------------------------------------------------------------------

  /**
   * @brief Generate the headland outline path using polygon_utils.
   *
   * Contracts the outer boundary by successive headland offsets and returns a
   * nav_msgs::Path tracing all headland contour rings. Used for visualisation
   * and for the outline component of the planner response.
   *
   * @param outer   Outer boundary polygon.
   * @param frame   Coordinate frame for the header.
   * @return Path tracing the headland contour(s).
   */
  nav_msgs::msg::Path generate_outline_path(const Polygon2D& outer, const std::string& frame) const;

  /**
   * @brief Compute the total Euclidean path length.
   *
   * @param path Input path (pose sequence).
   * @return Sum of distances between consecutive poses in metres.
   */
  static double compute_path_length(const nav_msgs::msg::Path& path);

  /**
   * @brief Build a PoseStamped with the given position and heading.
   *
   * @param x     X position.
   * @param y     Y position.
   * @param yaw   Heading angle in radians.
   * @param frame Coordinate frame id.
   * @return Fully populated PoseStamped.
   */
  static geometry_msgs::msg::PoseStamped make_pose(double x,
                                                   double y,
                                                   double yaw,
                                                   const std::string& frame);

  // -------------------------------------------------------------------------
  // ROS interfaces
  // -------------------------------------------------------------------------

  rclcpp_action::Server<PlanCoverageAction>::SharedPtr action_server_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr outline_pub_;
  rclcpp::Subscription<mowgli_interfaces::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  /// Latest tracked obstacles from obstacle_tracker (guarded by mutex).
  std::mutex obstacle_mutex_;
  std::vector<geometry_msgs::msg::Polygon> tracked_obstacles_;

  /// Latest global costmap (guarded by mutex).
  mutable std::mutex costmap_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

  /// Extract obstacle polygons from lethal cells in the costmap.
  /// Groups connected lethal cells into clusters and returns their
  /// convex hulls as polygons suitable for F2C cell holes.
  std::vector<geometry_msgs::msg::Polygon> extract_costmap_obstacles() const;

  /// Minimum number of connected lethal cells to form an obstacle.
  int costmap_min_cluster_size_{3};
  /// Inflation radius around costmap obstacles (metres).
  double costmap_obstacle_inflation_{0.10};

  // -------------------------------------------------------------------------
  // Parameters
  // -------------------------------------------------------------------------

  double tool_width_;  ///< Mowing blade / disc width [m].
  int headland_passes_;  ///< Number of headland perimeter passes.
  double headland_width_;  ///< Width of one headland pass [m].
  double default_mow_angle_;  ///< Default mowing angle [deg]; -1 = auto.
  double path_spacing_;  ///< Distance between parallel swath centrelines [m].
  double min_turning_radius_;  ///< Minimum robot turning radius [m].
  bool decompose_cells_;  ///< Enable cell decomposition for irregular polygons.
  std::string map_frame_;  ///< TF frame for output paths.
};

}  // namespace mowgli_coverage_planner

#endif  // MOWGLI_COVERAGE_PLANNER__COVERAGE_PLANNER_NODE_HPP_
