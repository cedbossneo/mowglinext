#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mowgli_ntrip_client/ntrip_connection.hpp"
#include <mavros_msgs/msg/rtcm.hpp>

namespace mowgli_ntrip_client
{

class NtripClientNode : public rclcpp::Node
{
public:
  explicit NtripClientNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~NtripClientNode() override;

private:
  void declare_parameters();
  ConnectionOptions load_connection_options() const;
  void setup_connection();
  void publish_rtcm(const std::vector<std::uint8_t>& data);
  void handle_connection_state(bool connected, const std::string& state);
  void log_status();

  std::shared_ptr<NtripConnection> connection_;
  rclcpp::Publisher<mavros_msgs::msg::RTCM>::SharedPtr rtcm_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::string frame_id_;
};

}  // namespace mowgli_ntrip_client
