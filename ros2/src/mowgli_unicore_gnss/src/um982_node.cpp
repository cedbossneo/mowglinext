// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_unicore_gnss/serial_port.hpp"
#include "mowgli_unicore_gnss/um982_parser.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>

#include <compass_msgs/msg/azimuth.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

namespace mowgli_unicore_gnss
{

namespace
{

using SteadyTime = std::chrono::steady_clock::time_point;

template<typename T>
struct TimedData
{
  T data;
  SteadyTime received_at;
};

diagnostic_msgs::msg::KeyValue kv(const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string to_string_or_nan(double value)
{
  if (!std::isfinite(value))
  {
    return "nan";
  }
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  oss << value;
  return oss.str();
}

std::string fix_source_name(FixSource source)
{
  switch (source)
  {
    case FixSource::kGga:
      return "GGA";
    case FixSource::kPvtslna:
      return "PVTSLNA";
  }
  return "unknown";
}

std::string heading_source_name(HeadingSource source)
{
  switch (source)
  {
    case HeadingSource::kHdt:
      return "HDT";
    case HeadingSource::kHpr:
      return "HPR";
  }
  return "unknown";
}

}  // namespace

class Um982Node : public rclcpp::Node
{
public:
  Um982Node()
  : rclcpp::Node("um982_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 921600);
    frame_id_ = declare_parameter<std::string>("frame_id", "gnss");
    data_timeout_sec_ = declare_parameter<double>("data_timeout_sec", 1.0);
    reconnect_interval_sec_ = declare_parameter<double>("reconnect_interval_sec", 1.0);
    read_poll_hz_ = declare_parameter<double>("read_poll_hz", 200.0);
    fix_topic_ = declare_parameter<std::string>("fix_topic", "/gnss/fix");
    heading_topic_ = declare_parameter<std::string>("heading_topic", "/gnss/azimuth");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/gnss/diagnostics");

    serial_.configure(port_, baudrate_);

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic_, rclcpp::QoS(10));
    heading_pub_ = create_publisher<compass_msgs::msg::Azimuth>(heading_topic_, rclcpp::QoS(10));
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, rclcpp::QoS(10));

    const auto poll_period = std::chrono::duration<double>(1.0 / std::max(1.0, read_poll_hz_));
    poll_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(poll_period),
      std::bind(&Um982Node::poll_serial, this));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&Um982Node::publish_diagnostics, this));

    RCLCPP_INFO(get_logger(),
      "UM982 node configured: port=%s baudrate=%d fix_topic=%s heading_topic=%s",
      port_.c_str(), baudrate_, fix_topic_.c_str(), heading_topic_.c_str());
  }

private:
  void poll_serial()
  {
    ensure_serial_open();
    if (!serial_.is_open())
    {
      return;
    }

    std::uint8_t buffer[1024];
    while (true)
    {
      const ssize_t bytes_read = serial_.read(buffer, sizeof(buffer));
      if (bytes_read > 0)
      {
        rx_buffer_.append(reinterpret_cast<const char*>(buffer), static_cast<std::size_t>(bytes_read));
        drain_lines();
        continue;
      }

      if (bytes_read == 0)
      {
        break;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        break;
      }

      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Serial read failed on %s: %s", port_.c_str(), std::strerror(errno));
      serial_.close();
      break;
    }

    if (rx_buffer_.size() > 8192U)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Dropping oversized UM982 receive buffer");
      rx_buffer_.clear();
    }
  }

  void ensure_serial_open()
  {
    const auto now = std::chrono::steady_clock::now();
    if (serial_.is_open())
    {
      return;
    }
    if (last_open_attempt_.has_value() &&
        std::chrono::duration<double>(now - *last_open_attempt_).count() < reconnect_interval_sec_)
    {
      return;
    }

    last_open_attempt_ = now;
    if (serial_.open())
    {
      RCLCPP_INFO(get_logger(), "Opened UM982 serial port %s at %d baud", port_.c_str(), baudrate_);
      rx_buffer_.clear();
    }
    else
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Unable to open UM982 serial port %s: %s", port_.c_str(), std::strerror(errno));
    }
  }

  void drain_lines()
  {
    std::size_t newline = rx_buffer_.find('\n');
    while (newline != std::string::npos)
    {
      std::string line = rx_buffer_.substr(0U, newline);
      rx_buffer_.erase(0U, newline + 1U);

      if (!line.empty() && line.back() == '\r')
      {
        line.pop_back();
      }

      process_line(line);
      newline = rx_buffer_.find('\n');
    }
  }

  void process_line(const std::string& line)
  {
    auto parsed = parser_.parse_line(line);
    if (!parsed.has_value())
    {
      return;
    }

    ++sentence_counts_[parsed->sentence_type];
    const auto received_at = std::chrono::steady_clock::now();

    if (parsed->fix.has_value())
    {
      if (parsed->fix->source == FixSource::kPvtslna)
      {
        latest_pvtslna_fix_ = TimedData<FixData>{*parsed->fix, received_at};
      }
      else
      {
        latest_gga_fix_ = TimedData<FixData>{*parsed->fix, received_at};
      }
      publish_active_fix();
    }

    if (parsed->heading.has_value())
    {
      if (parsed->heading->source == HeadingSource::kHpr)
      {
        latest_hpr_heading_ = TimedData<HeadingData>{*parsed->heading, received_at};
      }
      else
      {
        latest_hdt_heading_ = TimedData<HeadingData>{*parsed->heading, received_at};
      }
      publish_active_heading();
    }

    if (parsed->velocity.has_value())
    {
      latest_velocity_ = TimedData<VelocityData>{*parsed->velocity, received_at};
    }
  }

  bool is_fresh(const SteadyTime& stamp) const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count() <= data_timeout_sec_;
  }

  std::optional<FixData> active_fix() const
  {
    if (latest_pvtslna_fix_.has_value() && is_fresh(latest_pvtslna_fix_->received_at))
    {
      return latest_pvtslna_fix_->data;
    }
    if (latest_gga_fix_.has_value() && latest_gga_fix_->data.valid_fix && is_fresh(latest_gga_fix_->received_at))
    {
      return latest_gga_fix_->data;
    }
    return std::nullopt;
  }

  std::optional<HeadingData> active_heading() const
  {
    if (latest_hpr_heading_.has_value() && is_fresh(latest_hpr_heading_->received_at))
    {
      return latest_hpr_heading_->data;
    }
    if (latest_hdt_heading_.has_value() && is_fresh(latest_hdt_heading_->received_at))
    {
      return latest_hdt_heading_->data;
    }
    return std::nullopt;
  }

  sensor_msgs::msg::NavSatStatus build_nav_status(const FixData& fix) const
  {
    sensor_msgs::msg::NavSatStatus status;
    status.service =
      sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
      sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

    int quality = fix.fix_quality;
    if (quality <= 0 && latest_gga_fix_.has_value() && is_fresh(latest_gga_fix_->received_at))
    {
      quality = latest_gga_fix_->data.fix_quality;
    }

    switch (quality)
    {
      case 0:
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case 2:
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        break;
      case 4:
      case 5:
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        break;
      default:
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        break;
    }

    return status;
  }

  void publish_active_fix()
  {
    const auto fix = active_fix();
    if (!fix.has_value())
    {
      return;
    }

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.status = build_nav_status(*fix);
    msg.latitude = fix->latitude_deg;
    msg.longitude = fix->longitude_deg;
    msg.altitude = fix->altitude_m;

    if (fix->has_covariance)
    {
      msg.position_covariance = fix->covariance;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
    else
    {
      msg.position_covariance.fill(0.0);
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    fix_pub_->publish(msg);
  }

  void publish_active_heading()
  {
    const auto heading = active_heading();
    if (!heading.has_value())
    {
      return;
    }

    compass_msgs::msg::Azimuth msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.azimuth = heading->heading_deg;
    msg.variance = heading->variance_deg2;
    msg.unit = compass_msgs::msg::Azimuth::UNIT_DEG;
    msg.orientation = compass_msgs::msg::Azimuth::ORIENTATION_NED;
    msg.reference = compass_msgs::msg::Azimuth::REFERENCE_GEOGRAPHIC;
    heading_pub_->publish(msg);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "um982";
    status.hardware_id = port_;

    const auto fix = active_fix();
    const auto heading = active_heading();

    if (!serial_.is_open())
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "serial disconnected";
    }
    else if (!fix.has_value())
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for valid GNSS fix";
    }
    else if (!heading.has_value())
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "fix available, heading not yet available";
    }
    else
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "receiving UM982 data";
    }

    status.values.push_back(kv("serial_open", serial_.is_open() ? "true" : "false"));
    status.values.push_back(kv("fix_source", fix.has_value() ? fix_source_name(fix->source) : "none"));
    status.values.push_back(kv("heading_source", heading.has_value() ? heading_source_name(heading->source) : "none"));

    if (fix.has_value())
    {
      status.values.push_back(kv("latitude_deg", to_string_or_nan(fix->latitude_deg)));
      status.values.push_back(kv("longitude_deg", to_string_or_nan(fix->longitude_deg)));
      status.values.push_back(kv("altitude_m", to_string_or_nan(fix->altitude_m)));
      status.values.push_back(kv("fix_quality", std::to_string(fix->fix_quality)));
      status.values.push_back(kv("satellites", std::to_string(fix->satellites)));
      status.values.push_back(kv("hdop", to_string_or_nan(fix->hdop)));
    }

    if (heading.has_value())
    {
      status.values.push_back(kv("heading_deg", to_string_or_nan(heading->heading_deg)));
      status.values.push_back(kv("pitch_deg",
        heading->pitch_deg.has_value() ? to_string_or_nan(*heading->pitch_deg) : "n/a"));
      status.values.push_back(kv("roll_deg",
        heading->roll_deg.has_value() ? to_string_or_nan(*heading->roll_deg) : "n/a"));
    }

    if (latest_velocity_.has_value() && is_fresh(latest_velocity_->received_at))
    {
      status.values.push_back(kv("velocity_east_mps", to_string_or_nan(latest_velocity_->data.east_mps)));
      status.values.push_back(kv("velocity_north_mps", to_string_or_nan(latest_velocity_->data.north_mps)));
      status.values.push_back(kv("velocity_up_mps", to_string_or_nan(latest_velocity_->data.up_mps)));
    }

    for (const auto& [sentence_type, count] : sentence_counts_)
    {
      status.values.push_back(kv("count_" + sentence_type, std::to_string(count)));
    }

    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string port_;
  int baudrate_{921600};
  std::string frame_id_;
  double data_timeout_sec_{1.0};
  double reconnect_interval_sec_{1.0};
  double read_poll_hz_{200.0};
  std::string fix_topic_;
  std::string heading_topic_;
  std::string diagnostics_topic_;

  SerialPort serial_;
  Um982Parser parser_;
  std::string rx_buffer_;
  std::optional<SteadyTime> last_open_attempt_;

  std::optional<TimedData<FixData>> latest_gga_fix_;
  std::optional<TimedData<FixData>> latest_pvtslna_fix_;
  std::optional<TimedData<HeadingData>> latest_hdt_heading_;
  std::optional<TimedData<HeadingData>> latest_hpr_heading_;
  std::optional<TimedData<VelocityData>> latest_velocity_;

  std::unordered_map<std::string, std::size_t> sentence_counts_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<compass_msgs::msg::Azimuth>::SharedPtr heading_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace mowgli_unicore_gnss

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_unicore_gnss::Um982Node>());
  rclcpp::shutdown();
  return 0;
}
