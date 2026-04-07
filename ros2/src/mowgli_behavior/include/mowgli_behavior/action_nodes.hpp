#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/obstacle_array.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "opennav_docking_msgs/action/undock_robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// SetMowerEnabled
// ---------------------------------------------------------------------------

/// Calls the /hardware_bridge/mower_control service to enable or disable the
/// cutting blade motor.
///
/// Input ports:
///   enabled (bool) – true to start the blade, false to stop it.
class SetMowerEnabled : public BT::SyncActionNode
{
public:
  SetMowerEnabled(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("enabled", "Enable (true) or disable (false) the mow motor")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// StopMoving
// ---------------------------------------------------------------------------

/// Publishes a zero-velocity Twist to /cmd_vel to halt the robot immediately.
class StopMoving : public BT::SyncActionNode
{
public:
  StopMoving(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

// ---------------------------------------------------------------------------
// ClearCostmap
// ---------------------------------------------------------------------------

/// Calls the Nav2 clear_entirely service on both the global and local costmaps.
///
/// This is a synchronous fire-and-forget node: it sends both service requests
/// without waiting for responses (the node is already being spun by the main
/// executor), then returns SUCCESS immediately.  Useful after obstacle removal
/// to let the planner see a clean costmap before retrying coverage.
class ClearCostmap : public BT::SyncActionNode
{
public:
  ClearCostmap(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr local_client_;
};

// ---------------------------------------------------------------------------
// PublishHighLevelStatus
// ---------------------------------------------------------------------------

/// Publishes a HighLevelStatus message to ~/high_level_status.
///
/// Input ports:
///   state      (uint8_t) – HIGH_LEVEL_STATE_* constant.
///   state_name (string)  – Human-readable sub_state_name string.
class PublishHighLevelStatus : public BT::SyncActionNode
{
public:
  PublishHighLevelStatus(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<uint8_t>("state", "HIGH_LEVEL_STATE_* value"),
            BT::InputPort<std::string>("state_name", "Human-readable state label")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr pub_;
};

// ---------------------------------------------------------------------------
// WasRainingAtStart
// ---------------------------------------------------------------------------

/// Records whether it's currently raining into ctx->raining_at_mow_start.
/// Always returns SUCCESS. Called once at the start of a mowing session.
class WasRainingAtStart : public BT::SyncActionNode
{
public:
  WasRainingAtStart(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// RecordUndockStart — snapshot GPS position before undocking
// ---------------------------------------------------------------------------

class RecordUndockStart : public BT::SyncActionNode
{
public:
  RecordUndockStart(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// CalibrateHeadingFromUndock — compute heading from GPS displacement
// ---------------------------------------------------------------------------

/// After undocking (backward motion), computes heading from the difference
/// between the pre-undock and post-undock GPS positions. Since the robot
/// moved straight backward, the heading is opposite to the displacement vector.
class CalibrateHeadingFromUndock : public BT::SyncActionNode
{
public:
  CalibrateHeadingFromUndock(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// WaitForDuration
// ---------------------------------------------------------------------------

/// Stateful action that returns RUNNING until the requested duration has
/// elapsed, then returns SUCCESS.
///
/// Input ports:
///   duration_sec (double, default "1.0") – wait duration in seconds.
class WaitForDuration : public BT::StatefulActionNode
{
public:
  WaitForDuration(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("duration_sec", 1.0, "Duration to wait in seconds")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::duration<double> duration_;
};

// ---------------------------------------------------------------------------
// NavigateToPose
// ---------------------------------------------------------------------------

/// Stateful action that sends a goal to the Nav2 NavigateToPose action server
/// and waits for it to complete.
///
/// Input ports:
///   goal (string) – target pose encoded as "x;y;yaw" (metres / radians,
///                   frame_id = "map").
class NavigateToPose : public BT::StatefulActionNode
{
public:
  using Nav2Goal = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Goal>;

  NavigateToPose(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("goal", "Target pose as 'x;y;yaw'")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Goal>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;

  /// Lazily creates the action client once and reuses it across ticks.
  void ensureActionClient(const rclcpp::Node::SharedPtr& node);
};

// ---------------------------------------------------------------------------
// PlanCoveragePath
// ---------------------------------------------------------------------------

/// Calls the coverage planner action server asynchronously and waits for the
/// result.  Feedback progress is logged at DEBUG level.
///
/// Input ports:
///   area_index (uint32_t, default "0") – index of the mowing area to plan.
///   boundary   (string, default "")    – mowing boundary points (optional).
/// Output ports:
///   first_waypoint (string) – first waypoint as "x;y;yaw" for NavigateToPose.
class PlanCoveragePath : public BT::StatefulActionNode
{
public:
  using PlanCoverageAction = mowgli_interfaces::action::PlanCoverage;
  using GoalHandle = rclcpp_action::ClientGoalHandle<PlanCoverageAction>;

  PlanCoveragePath(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index to plan"),
            BT::OutputPort<std::string>("first_waypoint", "First waypoint as 'x;y;yaw'")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<PlanCoverageAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;

  /// Latest result received from the action server (set in result callback).
  std::shared_ptr<const PlanCoverageAction::Result> latest_result_;
  bool result_received_{false};

  uint32_t area_index_{0};
};

// ---------------------------------------------------------------------------
// FollowCoveragePath
// ---------------------------------------------------------------------------

/// Subscribes to the coverage path topic, sends it to Nav2 FollowPath action
/// using the MPPI controller, and returns SUCCESS when the robot has traversed
/// the entire path.  MPPI handles obstacle avoidance within the costmap
/// (keepout boundary + LiDAR obstacles) — no detour logic needed.
///
/// Input ports:
///   path_topic (string, default "/coverage_planner_node/coverage_path")
class FollowCoveragePath : public BT::StatefulActionNode
{
public:
  using FollowPathAction = nav2_msgs::action::FollowPath;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowPathAction>;

  FollowCoveragePath(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("path_topic",
                                       "/coverage_planner_node/coverage_path",
                                       "Topic with the coverage path to follow")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    WAIT_PATH,
    FOLLOWING
  };

  /// Send the full coverage path to the FollowPath action server.
  void sendFollowGoal();

  // FollowPath action (MPPI controller)
  rclcpp_action::Client<FollowPathAction>::SharedPtr follow_client_;
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;

  // Path and state
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path full_path_;
  bool path_received_{false};
  Phase phase_{Phase::WAIT_PATH};
  size_t current_path_index_{0};
};

// ---------------------------------------------------------------------------
// SaveSlamMap
// ---------------------------------------------------------------------------

/// Calls /slam_toolbox/serialize_map to persist the current pose graph to
/// disk.  Intended to run after mowing completes so the map survives container
/// restarts.
///
/// Input ports:
///   map_path (string, default "/ros2_ws/maps/garden_map") – destination path
///            without extension.  slam_toolbox appends .posegraph / .data.
///
/// Returns SUCCESS when the service call succeeds, FAILURE otherwise.  The
/// node is synchronous: it blocks for up to 5 s waiting for the service
/// response before declaring failure.
class SaveSlamMap : public BT::StatefulActionNode
{
public:
  using SerializeSrv = slam_toolbox::srv::SerializePoseGraph;

  SaveSlamMap(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>("map_path",
                                   "/ros2_ws/maps/garden_map",
                                   "Destination path without extension for the pose graph files")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Client<SerializeSrv>::SharedPtr client_;
  std::shared_future<SerializeSrv::Response::SharedPtr> response_future_;
};

// ---------------------------------------------------------------------------
// BackUp
// ---------------------------------------------------------------------------

/// Calls the Nav2 /backup action to reverse the robot by a given distance.
/// Used for undocking (reverse away from charger) and recovery (back away
/// from unseen obstacles).
///
/// Input ports:
///   backup_dist  (double, default "0.5") – distance to reverse in metres.
///   backup_speed (double, default "0.15") – reverse speed in m/s.
class BackUp : public BT::StatefulActionNode
{
public:
  using BackUpAction = nav2_msgs::action::BackUp;
  using GoalHandle = rclcpp_action::ClientGoalHandle<BackUpAction>;

  BackUp(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("backup_dist", 0.5, "Distance to reverse (m)"),
            BT::InputPort<double>("backup_speed", 0.15, "Reverse speed (m/s)")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using WrappedResult = rclcpp_action::ClientGoalHandle<BackUpAction>::WrappedResult;

  rclcpp_action::Client<BackUpAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<WrappedResult> result_future_;
  bool result_requested_{false};
};

// ---------------------------------------------------------------------------
// ReplanCoverage
// ---------------------------------------------------------------------------

/// Fetches the current mowing area + obstacles from the map server, then
/// calls the coverage planner to generate a new path.  On success, the new
/// path is published on the coverage_path topic (transient_local) and
/// FollowCoveragePath will pick it up on its next tick.
///
/// Output ports:
///   first_waypoint (string) – first waypoint as "x;y;yaw".
class ReplanCoverage : public BT::StatefulActionNode
{
public:
  using PlanCoverageAction = mowgli_interfaces::action::PlanCoverage;
  using GoalHandle = rclcpp_action::ClientGoalHandle<PlanCoverageAction>;

  ReplanCoverage(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<std::string>("first_waypoint", "First waypoint as 'x;y;yaw'")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<PlanCoverageAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_ptr<const PlanCoverageAction::Result> latest_result_;
  bool result_received_{false};
};

// ---------------------------------------------------------------------------
// SaveObstacles
// ---------------------------------------------------------------------------

/// Calls /obstacle_tracker/save_obstacles to persist the obstacle map to disk.
class SaveObstacles : public BT::SyncActionNode
{
public:
  SaveObstacles(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// SetNavMode
// ---------------------------------------------------------------------------

/// Dynamically adjusts Nav2 controller speed and costmap inflation based on
/// GPS quality.  "precise" = full speed, "degraded" = half speed + wider
/// inflation.
///
/// Input ports:
///   mode (string) – "precise" or "degraded"
class SetNavMode : public BT::SyncActionNode
{
public:
  SetNavMode(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("mode", "precise", "Navigation mode: precise or degraded")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// DockRobot
// ---------------------------------------------------------------------------

/// Calls the opennav_docking /dock_robot action to dock the robot.
///
/// Input ports:
///   dock_id   (string) – named dock instance (e.g. "home_dock")
///   dock_type (string) – dock plugin type (e.g. "simple_charging_dock")
class DockRobot : public BT::StatefulActionNode
{
public:
  using DockAction = opennav_docking_msgs::action::DockRobot;
  using GoalHandle = rclcpp_action::ClientGoalHandle<DockAction>;

  DockRobot(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_id", "home_dock", "Named dock instance"),
            BT::InputPort<std::string>("dock_type", "simple_charging_dock", "Dock plugin type")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<DockAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
};

// ---------------------------------------------------------------------------
// UndockRobot
// ---------------------------------------------------------------------------

/// Calls the opennav_docking /undock_robot action to undock the robot.
///
/// Input ports:
///   dock_type (string) – dock plugin type (e.g. "simple_charging_dock")
class UndockRobot : public BT::StatefulActionNode
{
public:
  using UndockAction = opennav_docking_msgs::action::UndockRobot;
  using GoalHandle = rclcpp_action::ClientGoalHandle<UndockAction>;

  UndockRobot(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_type", "simple_charging_dock", "Dock plugin type")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<UndockAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
};

// ---------------------------------------------------------------------------
// ClearCommand
// ---------------------------------------------------------------------------

/// Resets the current_command in BTContext to 0 (no command), preventing the
/// BT from re-entering a completed sequence.
class ClearCommand : public BT::SyncActionNode
{
public:
  ClearCommand(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// Free registration helper
// ---------------------------------------------------------------------------

/// Registers all mowgli_behavior BT nodes with the given factory.
void registerAllNodes(BT::BehaviorTreeFactory& factory);

}  // namespace mowgli_behavior
