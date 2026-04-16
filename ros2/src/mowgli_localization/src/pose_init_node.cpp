// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// ---------------------------------------------------------------------------
// Pose Init Node
//
// Anchors RTAB-Map's map frame to known world coordinates by publishing to
// the standard ROS /initialpose topic. Two triggers:
//
//   1. Boot: after FusionCore + RTAB-Map are up, if the robot reports
//      charging=true, anchor the pose to (dock_pose_x, dock_pose_y, dock_pose_yaw).
//
//   2. Dock event: on every charging false→true transition during operation
//      (robot just parked), re-anchor to the dock pose. This corrects any
//      drift accumulated during mowing.
//
// Why: FusionCore's odom frame is anchored to a fixed datum (configured via
// mowgli_robot.yaml datum_lat/datum_lon), so dock_pose_x/y are stable ENU
// coordinates from that datum. Setting the initial pose in the map frame to
// dock_pose makes the map frame identical to the world ENU frame.
//
// Input:
//   /hardware_bridge/power (mowgli_interfaces/Power) — charger_status
//
// Output:
//   /initialpose (geometry_msgs/PoseWithCovarianceStamped) — consumed by
//                 RTAB-Map's CoreWrapper to set the initial map→robot pose.
// ---------------------------------------------------------------------------

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mowgli_interfaces/msg/power.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mowgli_localization
{

class PoseInitNode : public rclcpp::Node
{
public:
  PoseInitNode() : Node("pose_init_node")
  {
    declare_parameter<double>("dock_pose_x", 0.0);
    declare_parameter<double>("dock_pose_y", 0.0);
    declare_parameter<double>("dock_pose_yaw", 0.0);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<double>("readiness_check_interval_sec", 1.0);
    declare_parameter<double>("boot_grace_period_sec", 5.0);

    dock_x_ = get_parameter("dock_pose_x").as_double();
    dock_y_ = get_parameter("dock_pose_y").as_double();
    dock_yaw_ = get_parameter("dock_pose_yaw").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    const double boot_grace = get_parameter("boot_grace_period_sec").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(1).transient_local());

    power_sub_ = create_subscription<mowgli_interfaces::msg::Power>(
        "/hardware_bridge/power",
        rclcpp::QoS(10),
        [this](mowgli_interfaces::msg::Power::ConstSharedPtr msg) { on_power(msg); });

    boot_time_ = now();
    boot_grace_sec_ = boot_grace;

    RCLCPP_INFO(get_logger(),
                "PoseInit ready: dock=(%.2f, %.2f, %.2frad). Will anchor on boot-when-charging "
                "and on every dock event.",
                dock_x_, dock_y_, dock_yaw_);
  }

private:
  void on_power(mowgli_interfaces::msg::Power::ConstSharedPtr msg)
  {
    const bool is_charging = (msg->charger_status == "charging");

    // Ignore power updates during boot grace period — lets FusionCore and
    // RTAB-Map come up before we try to anchor.
    if ((now() - boot_time_).seconds() < boot_grace_sec_)
    {
      last_charging_ = is_charging;
      return;
    }

    // Wait until map→base_footprint TF chain is available (proves both
    // FusionCore and RTAB-Map are publishing their respective transforms).
    if (!tf_ready())
    {
      last_charging_ = is_charging;
      return;
    }

    bool should_anchor = false;
    std::string reason;

    if (!boot_anchored_ && is_charging)
    {
      should_anchor = true;
      reason = "boot-while-charging";
    }
    else if (last_charging_.has_value() && !*last_charging_ && is_charging)
    {
      should_anchor = true;
      reason = "just-docked";
    }

    if (should_anchor)
    {
      publish_dock_pose(reason);
      boot_anchored_ = true;
    }

    last_charging_ = is_charging;
  }

  bool tf_ready()
  {
    try
    {
      tf_buffer_->lookupTransform(map_frame_, "base_footprint", tf2::TimePointZero);
      return true;
    }
    catch (const tf2::TransformException&)
    {
      return false;
    }
  }

  void publish_dock_pose(const std::string& reason)
  {
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = dock_x_;
    msg.pose.pose.position.y = dock_y_;
    msg.pose.pose.position.z = 0.0;
    // Yaw → quaternion (only z rotation)
    msg.pose.pose.orientation.z = std::sin(dock_yaw_ / 2.0);
    msg.pose.pose.orientation.w = std::cos(dock_yaw_ / 2.0);

    // Low covariance: we're telling the filter "robot IS at dock, trust this"
    const double sigma_xy = 0.05;  // 5 cm
    const double sigma_yaw = 0.035;  // ~2°
    msg.pose.covariance[0] = sigma_xy * sigma_xy;        // x
    msg.pose.covariance[7] = sigma_xy * sigma_xy;        // y
    msg.pose.covariance[35] = sigma_yaw * sigma_yaw;     // yaw

    initial_pose_pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
                "Anchored map→robot pose to dock (%.3f, %.3f, %.3frad) — reason: %s",
                dock_x_, dock_y_, dock_yaw_, reason.c_str());
  }

  // Params
  double dock_x_, dock_y_, dock_yaw_;
  std::string map_frame_;
  double boot_grace_sec_;

  // State
  std::optional<bool> last_charging_;
  bool boot_anchored_ = false;
  rclcpp::Time boot_time_;

  // ROS
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Power>::SharedPtr power_sub_;
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::PoseInitNode>());
  rclcpp::shutdown();
  return 0;
}
