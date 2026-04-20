// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0

#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ublox_ubx_msgs/msg/carr_soln.hpp"
#include "ublox_ubx_msgs/msg/gps_fix.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_cov.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_pvt.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_status.hpp"

namespace mowgli_localization
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sensor_msgs::msg::NavSatFix;
using ublox_ubx_msgs::msg::CarrSoln;
using ublox_ubx_msgs::msg::GpsFix;
using ublox_ubx_msgs::msg::UBXNavCov;
using ublox_ubx_msgs::msg::UBXNavPVT;
using ublox_ubx_msgs::msg::UBXNavStatus;

class UbloxGnssAdapterNode : public rclcpp::Node
{
public:
  explicit UbloxGnssAdapterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ublox_gnss_adapter", options)
  {
    declare_parameters();
    create_publishers();
    create_subscriptions();
    create_timer();

    RCLCPP_INFO(
      get_logger(),
      "UbloxGnssAdapterNode started. vendor_fix=%s gnss_fix=%s gnss_diag=%s",
      vendor_fix_topic_.c_str(),
      gnss_fix_topic_.c_str(),
      gnss_diagnostics_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    vendor_fix_topic_ = declare_parameter<std::string>("vendor_fix_topic", "/ublox/fix");
    vendor_nav_status_topic_ =
      declare_parameter<std::string>("vendor_nav_status_topic", "/ublox/ubx_nav_status");
    vendor_nav_pvt_topic_ =
      declare_parameter<std::string>("vendor_nav_pvt_topic", "/ublox/ubx_nav_pvt");
    vendor_nav_cov_topic_ =
      declare_parameter<std::string>("vendor_nav_cov_topic", "/ublox/ubx_nav_cov");

    gnss_fix_topic_ = declare_parameter<std::string>("gnss_fix_topic", "/gnss/fix");
    gps_fix_compat_topic_ = declare_parameter<std::string>("gps_fix_topic", "/gps/fix");
    gnss_diagnostics_topic_ =
      declare_parameter<std::string>("gnss_diagnostics_topic", "/gnss/diagnostics");

    publish_gps_compat_topic_ =
      declare_parameter<bool>("publish_gps_compat_topic", true);
    diagnostics_rate_hz_ =
      declare_parameter<double>("diagnostics_rate_hz", 1.0);
    fix_timeout_sec_ =
      declare_parameter<double>("fix_timeout_sec", 2.0);

    backend_name_ = declare_parameter<std::string>("backend_name", "ublox");
    device_family_ = declare_parameter<std::string>("device_family", "F9P");
    frame_id_hint_ = declare_parameter<std::string>("frame_id_hint", "gps_link");

    if (diagnostics_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "diagnostics_rate_hz must be > 0. Falling back to 1.0 Hz.");
      diagnostics_rate_hz_ = 1.0;
    }
    if (fix_timeout_sec_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "fix_timeout_sec must be > 0. Falling back to 2.0 sec.");
      fix_timeout_sec_ = 2.0;
    }
  }

  void create_publishers()
  {
    const auto qos = rclcpp::SensorDataQoS();
    gnss_fix_pub_ = create_publisher<NavSatFix>(gnss_fix_topic_, qos);
    if (publish_gps_compat_topic_) {
      gps_fix_compat_pub_ = create_publisher<NavSatFix>(gps_fix_compat_topic_, qos);
    }
    diagnostics_pub_ = create_publisher<DiagnosticArray>(gnss_diagnostics_topic_, 10);
  }

  void create_subscriptions()
  {
    const auto qos = rclcpp::SensorDataQoS();

    vendor_fix_sub_ = create_subscription<NavSatFix>(
      vendor_fix_topic_, qos,
      [this](NavSatFix::ConstSharedPtr msg) {on_fix(msg);});

    vendor_nav_status_sub_ = create_subscription<UBXNavStatus>(
      vendor_nav_status_topic_, qos,
      [this](UBXNavStatus::ConstSharedPtr msg) {last_nav_status_ = *msg;});

    vendor_nav_pvt_sub_ = create_subscription<UBXNavPVT>(
      vendor_nav_pvt_topic_, qos,
      [this](UBXNavPVT::ConstSharedPtr msg) {last_nav_pvt_ = *msg;});

    vendor_nav_cov_sub_ = create_subscription<UBXNavCov>(
      vendor_nav_cov_topic_, qos,
      [this](UBXNavCov::ConstSharedPtr msg) {last_nav_cov_ = *msg;});
  }

  void create_timer()
  {
    const auto period = std::chrono::duration<double>(1.0 / diagnostics_rate_hz_);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() {publish_diagnostics();});
  }

  void on_fix(const NavSatFix::ConstSharedPtr & msg)
  {
    last_fix_ = *msg;
    last_fix_received_at_ = now();

    gnss_fix_pub_->publish(*msg);
    if (gps_fix_compat_pub_) {
      gps_fix_compat_pub_->publish(*msg);
    }
  }

  void publish_diagnostics()
  {
    DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(build_backend_status(array.header.stamp));
    diagnostics_pub_->publish(array);
  }

  DiagnosticStatus build_backend_status(const rclcpp::Time & stamp) const
  {
    DiagnosticStatus status;
    status.name = "gnss/" + backend_name_;
    status.hardware_id = backend_name_ + ":" + device_family_;
    status.level = DiagnosticStatus::ERROR;
    status.message = "No GNSS fix received";

    status.values.push_back(kv("backend", backend_name_));
    status.values.push_back(kv("device_family", device_family_));
    status.values.push_back(kv("vendor_fix_topic", vendor_fix_topic_));
    status.values.push_back(kv("gnss_fix_topic", gnss_fix_topic_));
    status.values.push_back(kv("frame_id_hint", frame_id_hint_));

    if (!last_fix_) {
      status.values.push_back(kv("fix_received", "false"));
      return status;
    }

    const double age_sec = (stamp - last_fix_received_at_).seconds();
    status.values.push_back(kv("fix_received", "true"));
    status.values.push_back(kv("fix_age_sec", format_float(age_sec, 3)));

    if (age_sec > fix_timeout_sec_) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "GNSS fix stale";
      status.values.push_back(kv("backend_ready", "false"));
    } else {
      status.values.push_back(kv("backend_ready", "true"));
      classify_fix_state(status);
    }

    append_fix_values(status);
    append_nav_status_values(status);
    append_nav_pvt_values(status);
    append_nav_cov_values(status);
    return status;
  }

  void classify_fix_state(DiagnosticStatus & status) const
  {
    const auto & fix = *last_fix_;
    const auto ros_status = fix.status.status;

    if (ros_status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
      status.level = DiagnosticStatus::WARN;
      status.message = "GNSS backend connected, no fix";
      return;
    }

    status.level = DiagnosticStatus::OK;
    if (ros_status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
      status.message = "GNSS RTK fixed";
    } else if (ros_status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) {
      status.message = "GNSS RTK float / differential";
    } else {
      status.message = "GNSS fix OK";
    }
  }

  void append_fix_values(DiagnosticStatus & status) const
  {
    const auto & fix = *last_fix_;
    status.values.push_back(kv("frame_id", fix.header.frame_id));
    status.values.push_back(kv("latitude", format_float(fix.latitude, 8)));
    status.values.push_back(kv("longitude", format_float(fix.longitude, 8)));
    status.values.push_back(kv("altitude_m", format_float(fix.altitude, 3)));
    status.values.push_back(kv("fix_status", std::to_string(fix.status.status)));
    status.values.push_back(kv("fix_service", std::to_string(fix.status.service)));
    status.values.push_back(
      kv("covariance_type", std::to_string(fix.position_covariance_type)));

    if (fix.position_covariance_type != NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      const double horiz_var =
        std::max(0.0, (fix.position_covariance[0] + fix.position_covariance[4]) / 2.0);
      const double vert_var = std::max(0.0, fix.position_covariance[8]);
      status.values.push_back(kv("horizontal_sigma_m", format_float(std::sqrt(horiz_var), 3)));
      status.values.push_back(kv("vertical_sigma_m", format_float(std::sqrt(vert_var), 3)));
    }
  }

  void append_nav_status_values(DiagnosticStatus & status) const
  {
    if (!last_nav_status_) {
      return;
    }

    const auto & nav = *last_nav_status_;
    status.values.push_back(kv("ubx_nav_status.gps_fix", gps_fix_to_string(nav.gps_fix.fix_type)));
    status.values.push_back(kv("ubx_nav_status.gps_fix_ok", bool_to_string(nav.gps_fix_ok)));
    status.values.push_back(kv("ubx_nav_status.diff_soln", bool_to_string(nav.diff_soln)));
    status.values.push_back(kv("ubx_nav_status.diff_corr", bool_to_string(nav.diff_corr)));
    status.values.push_back(
      kv("ubx_nav_status.carr_soln", carrier_solution_to_string(nav.carr_soln.status)));
  }

  void append_nav_pvt_values(DiagnosticStatus & status) const
  {
    if (!last_nav_pvt_) {
      return;
    }

    const auto & pvt = *last_nav_pvt_;
    status.values.push_back(kv("ubx_nav_pvt.num_sv", std::to_string(pvt.num_sv)));
    status.values.push_back(kv("ubx_nav_pvt.h_acc_m", format_float(pvt.h_acc / 1000.0, 3)));
    status.values.push_back(kv("ubx_nav_pvt.v_acc_m", format_float(pvt.v_acc / 1000.0, 3)));
    status.values.push_back(kv("ubx_nav_pvt.p_dop", format_float(pvt.p_dop / 100.0, 2)));
    status.values.push_back(kv("ubx_nav_pvt.diff_soln", bool_to_string(pvt.diff_soln)));
    status.values.push_back(
      kv("ubx_nav_pvt.carr_soln", carrier_solution_to_string(pvt.carr_soln.status)));
  }

  void append_nav_cov_values(DiagnosticStatus & status) const
  {
    if (!last_nav_cov_) {
      return;
    }

    const auto & cov = *last_nav_cov_;
    status.values.push_back(kv("ubx_nav_cov.pos_cov_valid", bool_to_string(cov.pos_cor_valid)));
    status.values.push_back(kv("ubx_nav_cov.vel_cov_valid", bool_to_string(cov.vel_cor_valid)));
  }

  static KeyValue kv(const std::string & key, const std::string & value)
  {
    KeyValue pair;
    pair.key = key;
    pair.value = value;
    return pair;
  }

  static std::string bool_to_string(bool value)
  {
    return value ? "true" : "false";
  }

  static std::string format_float(double value, int precision)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }

  static std::string gps_fix_to_string(uint8_t fix_type)
  {
    switch (fix_type) {
      case GpsFix::GPS_NO_FIX:
        return "NO_FIX";
      case GpsFix::GPS_DEAD_RECKONING_ONLY:
        return "DEAD_RECKONING_ONLY";
      case GpsFix::GPS_FIX_2D:
        return "FIX_2D";
      case GpsFix::GPS_FIX_3D:
        return "FIX_3D";
      case GpsFix::GPS_PLUS_DEAD_RECKONING:
        return "GPS_PLUS_DR";
      case GpsFix::GPS_TIME_ONLY:
        return "TIME_ONLY";
      default:
        return "UNKNOWN(" + std::to_string(fix_type) + ")";
    }
  }

  static std::string carrier_solution_to_string(uint8_t status)
  {
    switch (status) {
      case CarrSoln::CARRIER_SOLUTION_NO_CARRIER_RANGE_SOLUTION:
        return "NONE";
      case CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FLOATING_AMBIGUITIES:
        return "RTK_FLOAT";
      case CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES:
        return "RTK_FIXED";
      default:
        return "UNKNOWN(" + std::to_string(status) + ")";
    }
  }

  std::string vendor_fix_topic_;
  std::string vendor_nav_status_topic_;
  std::string vendor_nav_pvt_topic_;
  std::string vendor_nav_cov_topic_;
  std::string gnss_fix_topic_;
  std::string gps_fix_compat_topic_;
  std::string gnss_diagnostics_topic_;
  bool publish_gps_compat_topic_{true};
  double diagnostics_rate_hz_{1.0};
  double fix_timeout_sec_{2.0};
  std::string backend_name_;
  std::string device_family_;
  std::string frame_id_hint_;

  rclcpp::Publisher<NavSatFix>::SharedPtr gnss_fix_pub_;
  rclcpp::Publisher<NavSatFix>::SharedPtr gps_fix_compat_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::Subscription<NavSatFix>::SharedPtr vendor_fix_sub_;
  rclcpp::Subscription<UBXNavStatus>::SharedPtr vendor_nav_status_sub_;
  rclcpp::Subscription<UBXNavPVT>::SharedPtr vendor_nav_pvt_sub_;
  rclcpp::Subscription<UBXNavCov>::SharedPtr vendor_nav_cov_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::optional<NavSatFix> last_fix_;
  std::optional<UBXNavStatus> last_nav_status_;
  std::optional<UBXNavPVT> last_nav_pvt_;
  std::optional<UBXNavCov> last_nav_cov_;
  rclcpp::Time last_fix_received_at_{0, 0, RCL_ROS_TIME};
};

}  // namespace mowgli_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::UbloxGnssAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
