// Copyright (C) 2024 Cedric
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * @file fake_hardware_bridge_node.cpp
 * @brief Fake hardware bridge for simulation.
 *
 * Provides stub services and topics that the behavior tree expects from the
 * real hardware_bridge_node, so simulation runs without "service unavailable"
 * warnings.
 *
 * Services:
 *   - /hardware_bridge/mower_control (MowerControl) — always succeeds
 *
 * Publishers:
 *   - /hardware_bridge/status   (mowgli_interfaces/Status)   — simulated idle
 *   - /hardware_bridge/power    (mowgli_interfaces/Power)     — simulated full battery
 *   - /hardware_bridge/emergency (mowgli_interfaces/Emergency) — no emergency
 */

#include <memory>

#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "rclcpp/rclcpp.hpp"

class FakeHardwareBridgeNode : public rclcpp::Node
{
public:
  FakeHardwareBridgeNode() : Node("fake_hardware_bridge")
  {
    // Service: mower_control
    mower_control_srv_ = create_service<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control",
        [this](const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
               std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res)
        {
          (void)req;
          res->success = true;
          RCLCPP_DEBUG(get_logger(), "Fake mower_control: mow_enabled=%u", req->mow_enabled);
        });

    // Publishers
    status_pub_ = create_publisher<mowgli_interfaces::msg::Status>("/hardware_bridge/status",
                                                                   rclcpp::QoS(10));
    power_pub_ =
        create_publisher<mowgli_interfaces::msg::Power>("/hardware_bridge/power", rclcpp::QoS(10));
    emergency_pub_ =
        create_publisher<mowgli_interfaces::msg::Emergency>("/hardware_bridge/emergency",
                                                            rclcpp::QoS(10));

    // Publish at 1 Hz
    timer_ = create_wall_timer(std::chrono::seconds(1),
                               [this]()
                               {
                                 publish_fake_data();
                               });

    RCLCPP_INFO(get_logger(), "Fake hardware bridge started (simulation mode)");
  }

private:
  void publish_fake_data()
  {
    auto now = this->now();

    mowgli_interfaces::msg::Status status;
    status.stamp = now;
    status_pub_->publish(status);

    mowgli_interfaces::msg::Power power;
    power.stamp = now;
    power.v_battery = 28.0;
    power.v_charge = 0.0;
    power.charge_current = 0.0;
    power_pub_->publish(power);

    mowgli_interfaces::msg::Emergency emergency;
    emergency.stamp = now;
    emergency.active_emergency = false;
    emergency.latched_emergency = false;
    emergency_pub_->publish(emergency);
  }

  rclcpp::Service<mowgli_interfaces::srv::MowerControl>::SharedPtr mower_control_srv_;
  rclcpp::Publisher<mowgli_interfaces::msg::Status>::SharedPtr status_pub_;
  rclcpp::Publisher<mowgli_interfaces::msg::Power>::SharedPtr power_pub_;
  rclcpp::Publisher<mowgli_interfaces::msg::Emergency>::SharedPtr emergency_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeHardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
