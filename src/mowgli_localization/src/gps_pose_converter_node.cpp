// SPDX-License-Identifier: GPL-3.0
/**
 * @file gps_pose_converter_node.cpp
 * @brief GPS pose converter implementation.
 *
 * Fix-quality flag semantics from AbsolutePose.msg:
 *
 *   FLAG_GPS_RTK_FIXED   (0x0002) – centimetre-level, fully converged.
 *   FLAG_GPS_RTK_FLOAT   (0x0004) – decimetre-level, ambiguities not fixed.
 *   FLAG_GPS_DEAD_RECKONING (0x0008) – no satellite contact, pure inertial.
 *
 * If none of these flags is set the message originates from a source type
 * other than GPS (lighthouse, sensor fusion) or the GPS has no fix at all;
 * in that case we also accept RTK data from FLAG_GPS_RTK (0x0001) as a
 * generic "RTK capable" indicator alongside FIXED/FLOAT.
 *
 * Covariance layout for robot_localization (6×6 row-major):
 *   index [0]  → x variance
 *   index [7]  → y variance
 *   index [35] → yaw variance  (set to a large constant; we don't fuse GPS
 *                yaw into the map EKF via pose0_config)
 *
 * All other elements remain zero (uncorrelated, 2-D only).
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

GpsPoseConverterNode::GpsPoseConverterNode(const rclcpp::NodeOptions & options)
: Node("gps_pose_converter", options)
{
  declare_parameters();
  create_publishers();
  create_subscribers();

  RCLCPP_INFO(
    get_logger(),
    "GpsPoseConverterNode started — min_accuracy_threshold=%.2f m",
    min_accuracy_threshold_);
}

// ---------------------------------------------------------------------------
// Initialisation helpers
// ---------------------------------------------------------------------------

void GpsPoseConverterNode::declare_parameters()
{
  use_first_fix_as_datum_  = declare_parameter<bool>  ("use_first_fix_as_datum",  true);
  datum_lat_               = declare_parameter<double>("datum_lat",               0.0);
  datum_lon_               = declare_parameter<double>("datum_lon",               0.0);
  min_accuracy_threshold_  = declare_parameter<double>("min_accuracy_threshold",  0.5);
}

void GpsPoseConverterNode::create_publishers()
{
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/gps/pose", rclcpp::QoS(10));
}

void GpsPoseConverterNode::create_subscribers()
{
  abs_pose_sub_ = create_subscription<mowgli_interfaces::msg::AbsolutePose>(
    "/gps/absolute_pose",
    rclcpp::QoS(10),
    [this](mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg) {
      on_absolute_pose(msg);
    });
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------

void GpsPoseConverterNode::on_absolute_pose(
  mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
{
  // Compute the xy position variance; a negative return value means the fix
  // quality is insufficient and we should drop this measurement.
  const double xy_variance = compute_xy_variance(*msg);
  if (xy_variance < 0.0) {
    RCLCPP_DEBUG(get_logger(), "GPS fix quality too poor — discarding pose measurement");
    return;
  }

  // ------------------------------------------------------------------
  // Build PoseWithCovarianceStamped
  // ------------------------------------------------------------------
  geometry_msgs::msg::PoseWithCovarianceStamped out;

  // Use current ROS time so the EKF timestamp-matches correctly.
  out.header.stamp    = now();
  out.header.frame_id = "map";

  // Pass through the projected pose as-is; the xbot_positioning / firmware
  // subsystem already handles the geodetic → local ENU projection.
  out.pose.pose = msg->pose.pose;

  // Populate the 6×6 covariance (row-major).
  // robot_localization's pose0_config in localization.yaml fuses only x and y,
  // so we set those diagonals accurately and leave yaw very large.
  out.pose.covariance[0]  = xy_variance;  // x
  out.pose.covariance[7]  = xy_variance;  // y
  out.pose.covariance[14] = 1e6;           // z  (irrelevant, 2D)
  out.pose.covariance[21] = 1e6;           // roll
  out.pose.covariance[28] = 1e6;           // pitch
  out.pose.covariance[35] = 1e6;           // yaw (not fused from GPS pose)

  pose_pub_->publish(out);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

double GpsPoseConverterNode::compute_xy_variance(
  const mowgli_interfaces::msg::AbsolutePose & msg) const
{
  using Flags = mowgli_interfaces::msg::AbsolutePose;

  // Discard altogether if accuracy is reported as worse than the threshold.
  // position_accuracy of 0.0 means "unknown"; treat with caution.
  if (msg.position_accuracy > min_accuracy_threshold_ && msg.position_accuracy > 0.0f) {
    return -1.0;
  }

  // Determine a quality multiplier from the fix-type flags.
  // Check from best to worst quality.
  double multiplier = 0.0;

  if ((msg.flags & Flags::FLAG_GPS_RTK_FIXED) != 0u) {
    // RTK fixed — centimetre-level.  Trust the reported accuracy directly.
    multiplier = 1.0;
  } else if ((msg.flags & Flags::FLAG_GPS_RTK_FLOAT) != 0u) {
    // RTK float — decimetre-level.  Inflate variance by 4× (2× sigma).
    multiplier = 4.0;
  } else if ((msg.flags & Flags::FLAG_GPS_RTK) != 0u) {
    // Generic RTK indicator without fixed/float distinction.
    multiplier = 2.0;
  } else if ((msg.flags & Flags::FLAG_GPS_DEAD_RECKONING) != 0u) {
    // Dead reckoning — loose position estimate, large uncertainty.
    multiplier = 16.0;
  } else {
    // No recognisable GPS fix flag; discard.
    return -1.0;
  }

  // Variance = (1-sigma accuracy)² × multiplier.
  // Use a floor of 0.01 m² (10 cm sigma) to avoid feeding the EKF variance
  // values that are unrealistically tight.
  const double sigma      = std::max(static_cast<double>(msg.position_accuracy), 0.01);
  const double variance   = sigma * sigma * multiplier;

  return variance;
}

}  // namespace mowgli_localization

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::GpsPoseConverterNode>());
  rclcpp::shutdown();
  return 0;
}
