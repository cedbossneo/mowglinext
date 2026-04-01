// SPDX-License-Identifier: GPL-3.0
/**
 * @file gps_pose_converter_node.cpp
 * @brief GPS pose converter — always publishes, covariance reflects quality.
 *
 * Every GPS fix is forwarded to the EKF with a covariance that reflects the
 * actual fix quality.  The EKF and SLAM naturally handle the weighting:
 *   - RTK fixed  → tight covariance  → GPS dominates position
 *   - RTK float  → moderate covariance → GPS + SLAM blend
 *   - No RTK     → large covariance  → SLAM dominates, GPS is a hint
 *   - Dead reck. → very large covariance → SLAM fully in charge
 *
 * This avoids black-outs where the robot loses all position reference
 * because GPS was silently dropped.
 *
 * Covariance layout for robot_localization (6×6 row-major):
 *   index [0]  → x variance
 *   index [7]  → y variance
 *   index [35] → yaw variance  (set to a large constant; GPS yaw not fused)
 */

#include "mowgli_localization/gps_pose_converter_node.hpp"

#include <cmath>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_localization
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

GpsPoseConverterNode::GpsPoseConverterNode(const rclcpp::NodeOptions& options)
    : Node("gps_pose_converter", options)
{
  declare_parameters();
  create_publishers();
  create_subscribers();

  RCLCPP_INFO(get_logger(), "GpsPoseConverterNode started");
}

// ---------------------------------------------------------------------------
// Initialisation helpers
// ---------------------------------------------------------------------------

void GpsPoseConverterNode::declare_parameters()
{
  min_accuracy_threshold_ = declare_parameter<double>("min_accuracy_threshold", 0.5);
}

void GpsPoseConverterNode::create_publishers()
{
  pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gps/pose", rclcpp::QoS(10));
}

void GpsPoseConverterNode::create_subscribers()
{
  abs_pose_sub_ = create_subscription<mowgli_interfaces::msg::AbsolutePose>(
      "/gps/absolute_pose",
      rclcpp::QoS(10),
      [this](mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
      {
        on_absolute_pose(msg);
      });
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------

void GpsPoseConverterNode::on_absolute_pose(
    mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
{
  const double xy_variance = compute_xy_variance(*msg);

  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp = now();
  out.header.frame_id = "map";
  out.pose.pose = msg->pose.pose;

  out.pose.covariance[0] = xy_variance;   // x
  out.pose.covariance[7] = xy_variance;   // y
  out.pose.covariance[14] = 1e6;          // z  (2D mode)
  out.pose.covariance[21] = 1e6;          // roll
  out.pose.covariance[28] = 1e6;          // pitch
  out.pose.covariance[35] = 1e6;          // yaw (not fused from GPS)

  pose_pub_->publish(out);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

double GpsPoseConverterNode::compute_xy_variance(
    const mowgli_interfaces::msg::AbsolutePose& msg) const
{
  using Flags = mowgli_interfaces::msg::AbsolutePose;

  // Determine a quality multiplier from the fix-type flags.
  double multiplier = 1.0;

  if ((msg.flags & Flags::FLAG_GPS_RTK_FIXED) != 0u)
  {
    // RTK fixed — centimetre-level.
    multiplier = 1.0;
  }
  else if ((msg.flags & Flags::FLAG_GPS_RTK_FLOAT) != 0u)
  {
    // RTK float — decimetre-level.
    multiplier = 4.0;
  }
  else if ((msg.flags & Flags::FLAG_GPS_RTK) != 0u)
  {
    // Autonomous GPS fix (no RTK corrections).
    multiplier = 16.0;
  }
  else if ((msg.flags & Flags::FLAG_GPS_DEAD_RECKONING) != 0u)
  {
    // Dead reckoning — very loose.
    multiplier = 64.0;
  }
  else
  {
    // Unknown fix type — treat as low-quality autonomous fix.
    multiplier = 32.0;
  }

  // Reported accuracy (1-sigma metres).  Floor at 1 cm to avoid
  // unrealistically tight variance values.
  const double sigma = (msg.position_accuracy > 0.0f)
      ? std::max(static_cast<double>(msg.position_accuracy), 0.01)
      : 5.0;  // unknown accuracy → assume 5 m

  return sigma * sigma * multiplier;
}

}  // namespace mowgli_localization

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::GpsPoseConverterNode>());
  rclcpp::shutdown();
  return 0;
}
