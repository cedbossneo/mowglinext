#include "mowgli_ntrip_client/ntrip_client_node.hpp"

#include <stdexcept>
#include <utility>

namespace mowgli_ntrip_client
{

NtripClientNode::NtripClientNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mowgli_ntrip_client", options)
{
  declare_parameters();

  frame_id_ = this->get_parameter("frame_id").as_string();
  rtcm_publisher_ = this->create_publisher<mavros_msgs::msg::RTCM>("/rtcm", rclcpp::QoS(10));

  setup_connection();

  const auto status_period_ms = this->get_parameter("status_log_period_ms").as_int();
  if (status_period_ms > 0)
  {
    status_timer_ = this->create_wall_timer(std::chrono::milliseconds(status_period_ms),
                                            std::bind(&NtripClientNode::log_status, this));
  }

  const auto port = this->get_parameter("port").as_int();
  RCLCPP_INFO(this->get_logger(),
              "Configured NTRIP client for %s:%ld/%s",
              this->get_parameter("host").as_string().c_str(),
              port,
              this->get_parameter("mountpoint").as_string().c_str());
}

NtripClientNode::~NtripClientNode()
{
  if (connection_)
  {
    connection_->stop();
  }
}

void NtripClientNode::declare_parameters()
{
  this->declare_parameter<std::string>("host", "127.0.0.1");
  this->declare_parameter<int>("port", 2101);
  this->declare_parameter<std::string>("mountpoint", "");
  this->declare_parameter<std::string>("username", "");
  this->declare_parameter<std::string>("password", "");
  this->declare_parameter<std::string>("frame_id", "gps");
  this->declare_parameter<std::string>("user_agent", "mowgli_ntrip_client/0.1");
  this->declare_parameter<int>("reconnect_delay_ms", 5000);
  this->declare_parameter<int>("connect_timeout_ms", 5000);
  this->declare_parameter<int>("read_timeout_ms", 15000);
  this->declare_parameter<int>("status_log_period_ms", 10000);
}

ConnectionOptions NtripClientNode::load_connection_options() const
{
  ConnectionOptions options;
  options.host = this->get_parameter("host").as_string();
  options.port = static_cast<std::uint16_t>(this->get_parameter("port").as_int());
  options.mountpoint = this->get_parameter("mountpoint").as_string();
  options.username = this->get_parameter("username").as_string();
  options.password = this->get_parameter("password").as_string();
  options.user_agent = this->get_parameter("user_agent").as_string();
  options.reconnect_delay_ms =
      static_cast<std::size_t>(this->get_parameter("reconnect_delay_ms").as_int());
  options.connect_timeout_ms =
      static_cast<std::size_t>(this->get_parameter("connect_timeout_ms").as_int());
  options.read_timeout_ms =
      static_cast<std::size_t>(this->get_parameter("read_timeout_ms").as_int());
  return options;
}

void NtripClientNode::setup_connection()
{
  auto options = load_connection_options();
  if (options.host.empty())
  {
    throw std::runtime_error("Parameter 'host' must not be empty");
  }

  if (options.mountpoint.empty())
  {
    throw std::runtime_error("Parameter 'mountpoint' must not be empty");
  }

  connection_ = std::make_shared<NtripConnection>(std::move(options));
  connection_->set_data_callback(
      [this](const std::vector<std::uint8_t>& data)
      {
        publish_rtcm(data);
      });
  connection_->set_state_callback(
      [this](bool connected, const std::string& state)
      {
        handle_connection_state(connected, state);
      });
  connection_->start();
}

void NtripClientNode::publish_rtcm(const std::vector<std::uint8_t>& data)
{
  if (data.empty())
  {
    return;
  }

  mavros_msgs::msg::RTCM msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;
  msg.data = data;
  rtcm_publisher_->publish(msg);
}

void NtripClientNode::handle_connection_state(bool connected, const std::string& state)
{
  if (connected)
  {
    RCLCPP_INFO(this->get_logger(), "NTRIP stream state: %s", state.c_str());
    return;
  }

  if (state == "reconnecting" || state == "resolve_failed" || state == "connect_failed" ||
      state == "handshake_write_failed" || state == "handshake_read_failed" ||
      state == "caster_rejected" || state == "stream_failed" || state == "read_timeout" ||
      state == "connect_timeout")
  {
    RCLCPP_WARN(this->get_logger(), "NTRIP stream state: %s", state.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "NTRIP stream state: %s", state.c_str());
}

void NtripClientNode::log_status()
{
  if (!connection_)
  {
    return;
  }

  const auto stats = connection_->stats();
  RCLCPP_INFO(
      this->get_logger(),
      "NTRIP status connected=%s state=%s bytes_received=%llu reconnects=%llu last_error=%s",
      stats.connected ? "true" : "false",
      stats.last_state.c_str(),
      static_cast<unsigned long long>(stats.bytes_received),
      static_cast<unsigned long long>(stats.reconnect_count),
      stats.last_error.empty() ? "<none>" : stats.last_error.c_str());
}

}  // namespace mowgli_ntrip_client
