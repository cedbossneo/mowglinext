#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace mowgli_behavior
{

/// Shared context passed to all BehaviorTree nodes via the blackboard.
///
/// The main node keeps this struct alive and updates it from ROS2 topic
/// callbacks before each tree tick.  BT nodes retrieve a shared_ptr to
/// this struct with:
///
///   auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
struct BTContext
{
  /// ROS2 node used by action/service nodes to create clients.
  rclcpp::Node::SharedPtr node;

  // -----------------------------------------------------------------------
  // Latest sensor state (updated by topic subscribers in the main node)
  // -----------------------------------------------------------------------

  mowgli_interfaces::msg::Status latest_status;
  mowgli_interfaces::msg::Emergency latest_emergency;
  mowgli_interfaces::msg::Power latest_power;

  /// Timestamp of the last emergency message received.
  std::chrono::steady_clock::time_point last_emergency_time{};

  // -----------------------------------------------------------------------
  // Thread safety
  // -----------------------------------------------------------------------

  /// Mutex protecting fields written by subscriber callbacks and read by
  /// BT condition/action nodes.  Use std::lock_guard for RAII locking.
  mutable std::mutex context_mutex;

  // -----------------------------------------------------------------------
  // Command state (set by HighLevelControl service handler)
  // -----------------------------------------------------------------------

  /// Last command received via the ~/high_level_control service.
  /// Constants match HighLevelControl.srv (COMMAND_START=1, COMMAND_HOME=2,
  /// COMMAND_S1=3, COMMAND_S2=4, COMMAND_RESET_EMERGENCY=254, …).
  uint8_t current_command{0};

  // -----------------------------------------------------------------------
  // Derived / convenience fields (computed from latest_* messages)
  // -----------------------------------------------------------------------

  float battery_percent{100.0f};
  float gps_quality{0.0f};

  /// Latest GPS position in map frame (from /gps/absolute_pose)
  double gps_x{0.0};
  double gps_y{0.0};

  // -----------------------------------------------------------------------
  // GPS quality classification (derived from gps_quality / fix_type)
  // -----------------------------------------------------------------------

  /// GPS fix type: 0=no fix, 1=autonomous, 2=DGPS, 4=RTK fixed, 5=RTK float
  uint8_t gps_fix_type{0};

  /// true when RTK fixed (fix_type >= 4 and gps_quality > 80%)
  bool gps_is_fixed{false};

  // -----------------------------------------------------------------------
  // Localization quality flags (set by boundary/replan monitors)
  // -----------------------------------------------------------------------

  /// Set to true when ObstacleTracker publishes updated obstacles that
  /// differ from the last coverage plan.
  bool replan_needed{false};

  /// Set to true when the robot is outside all allowed polygons.
  bool boundary_violation{false};

  /// Current navigation mode: "precise" or "degraded"
  std::string current_nav_mode{"precise"};

  /// True if it was raining when the current mowing session started.
  /// Set by WasRainingAtStart, checked by IsNewRain.
  bool raining_at_mow_start{false};

  // -----------------------------------------------------------------------
  // GPS snapshot for heading calibration during undock
  // -----------------------------------------------------------------------
  double undock_start_x{0.0};
  double undock_start_y{0.0};
  bool undock_start_recorded{false};

  // -----------------------------------------------------------------------
  // Docking point (set from parameter or service call)
  // -----------------------------------------------------------------------

  double dock_x{0.0};
  double dock_y{0.0};
  double dock_yaw{0.0};

  // -----------------------------------------------------------------------
  // Coverage path components (set by ComputeCoverage, consumed by
  // ExecuteSwathBySwath).  Using simple structs to avoid depending on
  // opennav_coverage_msgs in the context header.
  // -----------------------------------------------------------------------

  struct Swath
  {
    geometry_msgs::msg::Point32 start;
    geometry_msgs::msg::Point32 end;
  };

  struct CoveragePlan
  {
    std::vector<Swath> swaths;
    std::vector<nav_msgs::msg::Path> turns;  // N-1 turns for N swaths
    nav_msgs::msg::Path full_path;           // Full F2C discretized path (swaths + turns)
  };

  /// Populated by ComputeCoverage, consumed by ExecuteSwathBySwath.
  std::optional<CoveragePlan> coverage_plan;

  /// Progress tracking across charge cycles.
  size_t next_swath_index{0};

  /// Coverage progress (updated by ExecuteSwathBySwath, read by PublishHighLevelStatus).
  int current_area{-1};
  int total_swaths{0};
  int completed_swaths{0};
  int skipped_swaths{0};

  // -----------------------------------------------------------------------
  // TF buffer (shared across all BT nodes)
  // -----------------------------------------------------------------------
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  // -----------------------------------------------------------------------
  // Shared helper node for service calls (avoids creating/destroying DDS
  // participants on every call — the main node is in rclcpp::spin so it
  // cannot be used directly with spin_until_future_complete).
  // -----------------------------------------------------------------------
  rclcpp::Node::SharedPtr helper_node;
};

}  // namespace mowgli_behavior
