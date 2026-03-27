// Copyright (c) 2024, Cedric - Mowgli ROS2 port.
// Ported from ftc_local_planner (ROS1 / move_base_flex) to Nav2 controller plugin.
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef MOWGLI_NAV2_PLUGINS__FTC_CONTROLLER_HPP_
#define MOWGLI_NAV2_PLUGINS__FTC_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2/LinearMath/Quaternion.h>  // No .hpp equivalent for LinearMath
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>

#include "mowgli_nav2_plugins/oscillation_detector.hpp"

namespace mowgli_nav2_plugins
{

/**
 * @class FTCController
 * @brief Nav2 controller plugin implementing the Follow-The-Carrot (FTC) algorithm.
 *
 * The controller advances a virtual carrot point along the global path and drives
 * the robot towards it using three decoupled PID channels (longitudinal, lateral,
 * angular).  A five-state machine manages the full trajectory lifecycle:
 *
 *   PRE_ROTATE -> FOLLOWING -> WAITING_FOR_GOAL_APPROACH -> POST_ROTATE -> FINISHED
 *
 * Ported from ftc_local_planner (mbf_costmap_core::CostmapController, ROS1).
 */
class FTCController : public nav2_core::Controller
{
public:
  FTCController() = default;
  ~FTCController() override = default;

  // ── nav2_core::Controller interface ──────────────────────────────────────

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // ── State machine ─────────────────────────────────────────────────────────

  enum class PlannerState
  {
    PRE_ROTATE,
    FOLLOWING,
    WAITING_FOR_GOAL_APPROACH,
    POST_ROTATE,
    FINISHED
  };

  PlannerState current_state_{PlannerState::PRE_ROTATE};
  rclcpp::Time state_entered_time_;
  bool is_crashed_{false};

  /// Seconds elapsed since the current state was entered.
  double time_in_current_state() const;

  PlannerState update_planner_state();

  // ── Control point / path tracking ─────────────────────────────────────────

  /// Advance the virtual carrot and project it into base_link.
  void update_control_point(double dt);

  /// Compute the look-ahead distance along the remaining straight path.
  double distanceLookahead() const;

  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
  Eigen::Affine3d current_control_point_;  ///< Carrot pose in map frame.
  Eigen::Affine3d local_control_point_;    ///< Carrot pose in base_link frame.

  uint32_t current_index_{0};
  double current_progress_{0.0};
  double current_movement_speed_{0.0};

  // ── PID state ────────────────────────────────────────────────────────────

  void calculate_velocity_commands(double dt, geometry_msgs::msg::TwistStamped & cmd_vel);

  double lat_error_{0.0};
  double lon_error_{0.0};
  double angle_error_{0.0};
  double last_lat_error_{0.0};
  double last_lon_error_{0.0};
  double last_angle_error_{0.0};
  double i_lat_error_{0.0};
  double i_lon_error_{0.0};
  double i_angle_error_{0.0};

  rclcpp::Time last_time_;

  // ── Collision checking ────────────────────────────────────────────────────

  bool checkCollision(int max_points);
  void debugObstacle(
    visualization_msgs::msg::Marker & obstacle_points,
    double x, double y, unsigned char cost, int max_ids);

  // ── Oscillation detection ─────────────────────────────────────────────────

  bool checkOscillation(const geometry_msgs::msg::TwistStamped & cmd_vel);

  FailureDetector failure_detector_;
  rclcpp::Time time_last_oscillation_;
  bool oscillation_detected_{false};
  bool oscillation_warning_{false};

  // ── ROS2 infrastructure ───────────────────────────────────────────────────

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("FTCController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_map_{nullptr};

  std::string plugin_name_;

  // Publishers (lifecycle-aware)
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_point_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;

  // ── Parameters ────────────────────────────────────────────────────────────

  /// Declare all ROS2 parameters and populate the local config struct.
  void declareParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /// Parameter-change callback registered with the node.
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  struct Config
  {
    // Control point speed
    double speed_fast{0.5};
    double speed_fast_threshold{1.5};
    double speed_fast_threshold_angle{5.0};
    double speed_slow{0.2};
    double speed_angular{20.0};
    double acceleration{1.0};

    // PID longitudinal
    double kp_lon{1.0};
    double ki_lon{0.0};
    double ki_lon_max{10.0};
    double kd_lon{0.0};

    // PID lateral
    double kp_lat{1.0};
    double ki_lat{0.0};
    double ki_lat_max{10.0};
    double kd_lat{0.0};

    // PID angular
    double kp_ang{1.0};
    double ki_ang{0.0};
    double ki_ang_max{10.0};
    double kd_ang{0.0};

    // Robot limits
    double max_cmd_vel_speed{2.0};
    double max_cmd_vel_ang{2.0};
    double max_goal_distance_error{1.0};
    double max_goal_angle_error{10.0};
    double goal_timeout{5.0};
    double max_follow_distance{1.0};

    // Options
    bool forward_only{true};
    bool debug_pid{false};
    bool debug_obstacle{false};

    // Recovery
    bool oscillation_recovery{true};
    double oscillation_v_eps{5.0};
    double oscillation_omega_eps{5.0};
    double oscillation_recovery_min_duration{5.0};

    // Obstacles
    bool check_obstacles{true};
    int obstacle_lookahead{5};
    bool obstacle_footprint{true};
  };

  Config config_;

  /// Speed limit applied via setSpeedLimit(). -1.0 means "no external limit".
  double speed_limit_{-1.0};
  bool speed_limit_is_percentage_{false};
};

}  // namespace mowgli_nav2_plugins

#endif  // MOWGLI_NAV2_PLUGINS__FTC_CONTROLLER_HPP_
