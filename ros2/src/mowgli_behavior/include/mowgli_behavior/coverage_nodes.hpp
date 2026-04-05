#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// ComputeCoverage
// ---------------------------------------------------------------------------

/// Calls the coverage_planner_node's PlanCoverage action server, then stores
/// the resulting swaths in BTContext::coverage_plan.
///
/// Input ports:
///   area_index (uint32_t, default "0") - mowing area index.
/// Output ports:
///   first_swath_start (string) - "x;y;yaw" of the first swath start point.
class ComputeCoverage : public BT::StatefulActionNode
{
public:
  using CoverageAction = mowgli_interfaces::action::PlanCoverage;
  using GoalHandle = rclcpp_action::ClientGoalHandle<CoverageAction>;

  ComputeCoverage(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index to plan"),
            BT::OutputPort<std::string>("first_swath_start", "First swath start as 'x;y;yaw'")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<CoverageAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_ptr<const CoverageAction::Result> latest_result_;
  bool result_received_{false};
};

// ---------------------------------------------------------------------------
// ExecuteSwathBySwath
// ---------------------------------------------------------------------------

/// Iterates over the swaths in BTContext::coverage_plan.
///
/// For each swath:
///   1. NavigateToPose (SmacPlanner2D) to swath start - obstacle-aware transit
///   2. FollowPath (RPP) along the swath - straight-line mowing
///
/// Blade control (SetMowerEnabled) is handled internally:
///   ON during swath following, OFF during transit.
class ExecuteSwathBySwath : public BT::StatefulActionNode
{
public:
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using Nav2FollowPath = nav2_msgs::action::FollowPath;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Nav2FollowPath>;

  ExecuteSwathBySwath(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    TRANSIT_TO_SWATH,
    MOWING_SWATH,
    REROUTING_AROUND_OBSTACLE,
    DONE
  };

  nav_msgs::msg::Path swathToPath(const BTContext::Swath& swath,
                                  const rclcpp::Node::SharedPtr& node) const;
  nav_msgs::msg::Path remainingSwathPath(const BTContext::Swath& swath,
                                         double from_x,
                                         double from_y,
                                         const rclcpp::Node::SharedPtr& node) const;
  void sendTransitGoal(const BTContext::Swath& swath);
  void sendSwathGoal(const BTContext::Swath& swath);
  void sendRerouteGoal(const BTContext::Swath& swath, double rx, double ry);
  void sendRemainingSwathGoal(const BTContext::Swath& swath, double from_x, double from_y);
  void setBladeEnabled(bool enabled);
  bool advanceToNextSwath();
  bool checkStuck(const std::shared_ptr<BTContext>& ctx);

  // State
  Phase phase_{Phase::TRANSIT_TO_SWATH};
  size_t swath_index_{0};
  size_t total_swaths_{0};
  size_t completed_swaths_{0};
  size_t skipped_swaths_{0};

  // Action clients
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  rclcpp_action::Client<Nav2FollowPath>::SharedPtr follow_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;

  // Goal handles
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;

  std::chrono::steady_clock::time_point transit_cooldown_until_{};

  /// True when the current transit uses FollowPath instead of NavigateToPose.
  bool use_follow_for_transit_{false};

  // Obstacle rerouting state
  size_t reroute_attempts_{0};
  static constexpr size_t max_reroute_attempts_{2};
  /// Fraction along the swath where the robot was blocked (0.0 = start, 1.0 = end).
  double blocked_swath_fraction_{0.0};

  // Stuck detection
  static constexpr double stuck_timeout_sec_{10.0};
  static constexpr double stuck_min_progress_{0.05};
  std::chrono::steady_clock::time_point last_progress_time_{};
  double last_progress_x_{0.0};
  double last_progress_y_{0.0};
};

// ---------------------------------------------------------------------------
// ExecuteFullCoveragePath
// ---------------------------------------------------------------------------

/// Sends the full F2C coverage path (swaths + Dubins turns) to Nav2's
/// FollowPath action as a single continuous path.  Much simpler than the
/// per-swath ExecuteSwathBySwath — no NavigateToPose transits, no per-swath
/// phase state machine.
///
/// Blade is enabled at start and disabled on completion/halt.
/// On retry (after stuck/abort), finds the closest pose on the path to the
/// robot's current position and sends the remaining sub-path.
class ExecuteFullCoveragePath : public BT::StatefulActionNode
{
public:
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using Nav2FollowPath = nav2_msgs::action::FollowPath;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Nav2FollowPath>;

  ExecuteFullCoveragePath(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    TRANSIT_TO_START,
    FOLLOWING_PATH,
    DONE
  };

  void setBladeEnabled(bool enabled);
  bool checkStuck(const std::shared_ptr<BTContext>& ctx);
  size_t findClosestPoseIndex(const nav_msgs::msg::Path& path, double rx, double ry) const;

  Phase phase_{Phase::TRANSIT_TO_START};

  // Action clients
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  rclcpp_action::Client<Nav2FollowPath>::SharedPtr follow_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;

  // Goal handles
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;

  // Stuck detection
  static constexpr double stuck_timeout_sec_{30.0};
  static constexpr double stuck_min_progress_{0.05};
  std::chrono::steady_clock::time_point last_progress_time_{};
  double last_progress_x_{0.0};
  double last_progress_y_{0.0};
};

}  // namespace mowgli_behavior
