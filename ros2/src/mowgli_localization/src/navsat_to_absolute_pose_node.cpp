// SPDX-License-Identifier: GPL-3.0
/**
 * @file navsat_to_absolute_pose_node.cpp
 * @brief Converts sensor_msgs/NavSatFix to mowgli_interfaces/AbsolutePose.
 *
 * WGS84 → local ENU projection uses equirectangular approximation:
 *   east  = (lon - datum_lon) * cos(datum_lat) * METERS_PER_DEG
 *   north = (lat - datum_lat) * METERS_PER_DEG
 *
 * This is accurate to ~1 cm within 10 km of the datum, which is more than
 * sufficient for a garden robot mower operating within a few hundred metres.
 *
 * NavSatFix status mapping to AbsolutePose flags:
 *   STATUS_FIX              → FLAG_GPS_RTK (generic fix)
 *   STATUS_SBAS_FIX         → FLAG_GPS_RTK_FLOAT
 *   STATUS_GBAS_FIX         → FLAG_GPS_RTK_FIXED
 *   covariance_type UNKNOWN → FLAG_GPS_DEAD_RECKONING
 *
 * The ublox_gps driver maps u-blox carrSoln:
 *   carrSoln=0 (no RTK)     → STATUS_FIX
 *   carrSoln=1 (RTK float)  → STATUS_SBAS_FIX
 *   carrSoln=2 (RTK fixed)  → STATUS_GBAS_FIX
 */

#include "mowgli_localization/navsat_to_absolute_pose_node.hpp"

#include <cmath>

namespace mowgli_localization
{

/// WGS84 equatorial radius in metres.
static constexpr double EARTH_RADIUS_M = 6378137.0;

/// Degrees to radians.
static constexpr double DEG_TO_RAD = M_PI / 180.0;

/// Metres per degree of latitude (approximate, at the equator).
static constexpr double METERS_PER_DEG = EARTH_RADIUS_M * DEG_TO_RAD;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

NavSatToAbsolutePoseNode::NavSatToAbsolutePoseNode(
    const rclcpp::NodeOptions& options)
    : Node("navsat_to_absolute_pose", options)
{
  declare_parameters();
  cos_datum_lat_ = std::cos(datum_lat_ * DEG_TO_RAD);
  create_publishers();
  create_subscribers();

  RCLCPP_INFO(get_logger(),
              "NavSatToAbsolutePoseNode started — datum: [%.7f, %.7f]",
              datum_lat_, datum_lon_);
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

void NavSatToAbsolutePoseNode::declare_parameters()
{
  datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
  datum_lon_ = declare_parameter<double>("datum_lon", 0.0);
}

void NavSatToAbsolutePoseNode::create_publishers()
{
  pose_pub_ = create_publisher<mowgli_interfaces::msg::AbsolutePose>(
      "/gps/absolute_pose", rclcpp::QoS(10));
}

void NavSatToAbsolutePoseNode::create_subscribers()
{
  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix",
      rclcpp::QoS(10),
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
      {
        on_navsat_fix(msg);
      });
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------

void NavSatToAbsolutePoseNode::on_navsat_fix(
    sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  using AbsPose = mowgli_interfaces::msg::AbsolutePose;
  using NavSat = sensor_msgs::msg::NavSatFix;
  using NavStatus = sensor_msgs::msg::NavSatStatus;

  // Discard if no fix at all.
  if (msg->status.status == NavStatus::STATUS_NO_FIX)
  {
    return;
  }

  // Project WGS84 → local ENU.
  double east = 0.0;
  double north = 0.0;
  wgs84_to_enu(msg->latitude, msg->longitude, east, north);

  // Build AbsolutePose.
  AbsPose out;
  out.header.stamp = now();
  out.header.frame_id = "map";
  out.source = AbsPose::SOURCE_GPS;

  // Map NavSatFix status to AbsolutePose flags.
  // The ublox_gps driver maps u-blox carrSoln values as:
  //   carrSoln=0 → STATUS_FIX (no RTK)
  //   carrSoln=1 → STATUS_SBAS_FIX (RTK float)
  //   carrSoln=2 → STATUS_GBAS_FIX (RTK fixed)
  switch (msg->status.status)
  {
    case NavStatus::STATUS_GBAS_FIX:
      out.flags = AbsPose::FLAG_GPS_RTK | AbsPose::FLAG_GPS_RTK_FIXED;
      break;
    case NavStatus::STATUS_SBAS_FIX:
      out.flags = AbsPose::FLAG_GPS_RTK | AbsPose::FLAG_GPS_RTK_FLOAT;
      break;
    case NavStatus::STATUS_FIX:
      out.flags = AbsPose::FLAG_GPS_RTK;
      break;
    default:
      out.flags = AbsPose::FLAG_GPS_DEAD_RECKONING;
      break;
  }

  // Position in local ENU frame. Note: ROS REP-103 map frame is
  // x=east (or forward), y=north (or left). We store east→x, north→y
  // which matches the ENU convention used by robot_localization.
  out.pose.pose.position.x = east;
  out.pose.pose.position.y = north;
  out.pose.pose.position.z = msg->altitude;

  // No orientation from GPS fix alone.
  out.pose.pose.orientation.w = 1.0;
  out.orientation_valid = 0;
  out.motion_vector_valid = 0;

  // Position accuracy from covariance diagonal (metres, 1-sigma).
  // NavSatFix covariance is [lat, lon, alt] in m² (ENU if type is known).
  if (msg->position_covariance_type != NavSat::COVARIANCE_TYPE_UNKNOWN)
  {
    // Take the mean of lat/lon variance as horizontal accuracy.
    const double lat_var = msg->position_covariance[0];
    const double lon_var = msg->position_covariance[4];
    out.position_accuracy =
        static_cast<float>(std::sqrt((lat_var + lon_var) / 2.0));
  }
  else
  {
    out.position_accuracy = 10.0f;  // Unknown — large default.
    out.flags = AbsPose::FLAG_GPS_DEAD_RECKONING;
  }

  pose_pub_->publish(out);
}

// ---------------------------------------------------------------------------
// WGS84 → ENU projection
// ---------------------------------------------------------------------------

void NavSatToAbsolutePoseNode::wgs84_to_enu(
    double lat, double lon, double& east, double& north) const
{
  east = (lon - datum_lon_) * cos_datum_lat_ * METERS_PER_DEG;
  north = (lat - datum_lat_) * METERS_PER_DEG;
}

}  // namespace mowgli_localization

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<mowgli_localization::NavSatToAbsolutePoseNode>());
  rclcpp::shutdown();
  return 0;
}
