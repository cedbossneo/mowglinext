// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

// The bridge is intentionally a single executable. Rename its main while
// including the implementation so this test exercises the real parameter
// descriptors instead of duplicating their behaviour in a helper.
#define main hardware_bridge_node_main_for_test
#include "../src/hardware_bridge_node.cpp"
#undef main

namespace mowgli_hardware
{

struct HardwareBridgeBladeStatusTestPeer
{
  static std::string requested(const HardwareBridgeNode& node)
  {
    return node.blade_requested_direction_;
  }
  static void send(HardwareBridgeNode& node, uint8_t on, uint8_t dir)
  {
    node.send_blade_command(on, dir);
  }
  static void disconnect(HardwareBridgeNode& node)
  {
    node.close_serial_for_reconnect();
  }
};

class HardwareBridgeParametersTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(HardwareBridgeParametersTest, PublishRateRejectsRuntimeUpdate)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("serial_port", "/definitely/not/a/serial/device")});
  auto node = std::make_shared<HardwareBridgeNode>(options);

  const auto result = node->set_parameter(rclcpp::Parameter("publish_rate", 5.0));

  EXPECT_FALSE(result.successful);
}

TEST_F(HardwareBridgeParametersTest, UsableOutOfRangePublishRateStarts)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("serial_port", "/definitely/not/a/serial/device"),
                               rclcpp::Parameter("publish_rate", 5.0)});

  EXPECT_NO_THROW(std::make_shared<HardwareBridgeNode>(options));
}

TEST_F(HardwareBridgeParametersTest, BladeDirectionTracksWrittenCommandsAndClearsOnDisconnect)
{
  // Real serial writes into a pseudo-terminal, without physical hardware.
  const int master = posix_openpt(O_RDWR | O_NOCTTY);
  ASSERT_GE(master, 0);
  struct CloseFd
  {
    int fd;
    ~CloseFd()
    {
      close(fd);
    }
  } close_fd{master};
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  ASSERT_NE(ptsname(master), nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("serial_port", ptsname(master))});
  auto node = std::make_shared<HardwareBridgeNode>(options);
  using Peer = HardwareBridgeBladeStatusTestPeer;
  EXPECT_EQ(Peer::requested(*node), "unknown");
  Peer::send(*node, 1, 1);
  EXPECT_EQ(Peer::requested(*node), "reverse");
  Peer::send(*node, 0, 0);
  EXPECT_EQ(Peer::requested(*node), "off");
  Peer::send(*node, 1, 0);
  EXPECT_EQ(Peer::requested(*node), "forward");
  Peer::disconnect(*node);
  EXPECT_EQ(Peer::requested(*node), "unknown");
  Peer::send(*node, 1, 1);
  EXPECT_EQ(Peer::requested(*node), "unknown");
}

}  // namespace mowgli_hardware
