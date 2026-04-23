#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mowgli_ntrip_client/ntrip_client_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mowgli_ntrip_client::NtripClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
