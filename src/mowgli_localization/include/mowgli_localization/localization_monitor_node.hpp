// SPDX-License-Identifier: GPL-3.0
/**
 * @file localization_monitor_node.hpp
 * @brief Localization quality monitor node.
 *
 * Watches the availability and quality of each localization source and
 * publishes a human-readable mode string that the behaviour tree and safety
 * monitors can use to gate mowing operations.
 *
 * Localization modes (ordered best to worst):
 *
 *   "RTK_SLAM"        – RTK fixed + LiDAR SLAM active.   ~2 cm absolute.
 *   "SLAM_DOMINANT"   – RTK float + LiDAR SLAM active.   ~5–10 cm.
 *   "SLAM_ODOM"       – No GPS + LiDAR SLAM active.       ~10–20 cm relative.
 *   "GPS_ODOM"        – RTK fixed + no LiDAR.             Obstacle avoidance unavailable.
 *   "DEAD_RECKONING"  – All sources degraded / timed out.  Return to dock.
 *
 * Subscribed topics:
 *   /wheel_odom               nav_msgs/msg/Odometry       (wheel odometry alive-check)
 *   /gps/absolute_pose        mowgli_interfaces/msg/AbsolutePose
 *   /slam_toolbox/pose        geometry_msgs/msg/PoseWithCovarianceStamped
 *
 * Published topics:
 *   /localization/mode        std_msgs/msg/String   (latched, 10 Hz)
 *   /localization/mode_id     std_msgs/msg/Int32    (integer enum for BT use)
 *
 * Parameters:
 *   gps_timeout    (double, default 2.0 s) – declare GPS stale after this gap.
 *   lidar_timeout  (double, default 1.0 s) – declare LiDAR SLAM stale.
 *   pose_timeout   (double, default 0.5 s) – declare wheel odom stale.
 *   publish_rate   (double, default 10.0 Hz)
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "mowgli_interfaces/msg/absolute_pose.hpp"

namespace mowgli_localization
{

/// Integer IDs for each localization mode (published on /localization/mode_id).
enum class LocalizationMode : int32_t
{
  DEAD_RECKONING = 0,
  GPS_ODOM       = 1,
  SLAM_ODOM      = 2,
  SLAM_DOMINANT  = 3,
  RTK_SLAM       = 4,
};

class LocalizationMonitorNode : public rclcpp::Node
{
public:
  explicit LocalizationMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalizationMonitorNode() override = default;

private:
  // ---------------------------------------------------------------------------
  // Initialisation helpers
  // ---------------------------------------------------------------------------
  void declare_parameters();
  void create_publishers();
  void create_subscribers();
  void create_timer();

  // ---------------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------------
  void on_wheel_odom (const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_absolute_pose(const mowgli_interfaces::msg::AbsolutePose::SharedPtr msg);
  void on_slam_pose   (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // ---------------------------------------------------------------------------
  // Timer callback – evaluates state and publishes
  // ---------------------------------------------------------------------------
  void on_publish_timer();

  // ---------------------------------------------------------------------------
  // Internal helpers
  // ---------------------------------------------------------------------------

  /**
   * @brief Evaluate the current localization mode from source freshness / quality.
   */
  LocalizationMode evaluate_mode() const;

  /**
   * @brief Convert a LocalizationMode enum to its string representation.
   */
  static std::string mode_to_string(LocalizationMode mode);

  /**
   * @brief Return true if the given time_point is within timeout seconds of now.
   */
  bool is_fresh(rclcpp::Time last_stamp, double timeout_sec) const;

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  double gps_timeout_{2.0};
  double lidar_timeout_{1.0};
  double pose_timeout_{0.5};
  double publish_rate_{10.0};

  // ---------------------------------------------------------------------------
  // Source state
  // ---------------------------------------------------------------------------
  rclcpp::Time last_wheel_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gps_stamp_       {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_slam_stamp_      {0, 0, RCL_ROS_TIME};

  /// True when the last GPS message carried an RTK-fixed or RTK-float flag.
  bool gps_rtk_active_{false};
  /// True when fixed (vs. float).
  bool gps_rtk_fixed_ {false};

  // ---------------------------------------------------------------------------
  // ROS handles
  // ---------------------------------------------------------------------------
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr  mode_id_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    wheel_odom_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr       abs_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mowgli_localization
