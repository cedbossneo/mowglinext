// SPDX-License-Identifier: GPL-3.0
/**
 * @file hardware_bridge_node.cpp
 * @brief ROS2 node: serial bridge between the STM32 firmware and the rest of
 *        the Mowgli ROS2 stack.
 *
 * The node communicates with the STM32 over USB-serial using the COBS-framed,
 * CRC-16-protected packet protocol defined in ll_datatypes.hpp.
 *
 * Published topics (relative to node namespace):
 *   ~/status       mowgli_interfaces/msg/Status
 *   ~/emergency    mowgli_interfaces/msg/Emergency
 *   ~/power        mowgli_interfaces/msg/Power
 *   ~/imu/data_raw sensor_msgs/msg/Imu
 *
 * Subscribed topics:
 *   ~/cmd_vel      geometry_msgs/msg/Twist  → LlCmdVel packet to STM32
 *
 * Services:
 *   ~/mower_control  mowgli_interfaces/srv/MowerControl
 *   ~/emergency_stop mowgli_interfaces/srv/EmergencyStop
 *
 * Parameters:
 *   serial_port      (string,  default "/dev/ttyUSB0")
 *   baud_rate        (int,     default 115200)
 *   heartbeat_rate   (double,  default 4.0 Hz  → 250 ms period)
 *   publish_rate     (double,  default 100.0 Hz → 10 ms period)
 *   high_level_rate  (double,  default 2.0 Hz   → 500 ms period)
 */

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/emergency_stop.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"

#include "mowgli_hardware/ll_datatypes.hpp"
#include "mowgli_hardware/packet_handler.hpp"
#include "mowgli_hardware/serial_port.hpp"

namespace mowgli_hardware
{

using namespace std::chrono_literals;

class HardwareBridgeNode : public rclcpp::Node
{
public:
  explicit HardwareBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("hardware_bridge", options)
  {
    declare_parameters();
    create_publishers();
    create_subscribers();
    create_services();
    open_serial_port();
    create_timers();
  }

  ~HardwareBridgeNode() override = default;

private:
  // ---------------------------------------------------------------------------
  // Initialisation helpers
  // ---------------------------------------------------------------------------

  void declare_parameters()
  {
    serial_port_path_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_rate_        = declare_parameter<int>("baud_rate", 115200);
    heartbeat_rate_   = declare_parameter<double>("heartbeat_rate", 4.0);
    publish_rate_     = declare_parameter<double>("publish_rate", 100.0);
    high_level_rate_  = declare_parameter<double>("high_level_rate", 2.0);

    RCLCPP_INFO(
      get_logger(),
      "Parameters: serial_port=%s baud_rate=%d heartbeat_rate=%.1f Hz "
      "publish_rate=%.1f Hz high_level_rate=%.1f Hz",
      serial_port_path_.c_str(), baud_rate_,
      heartbeat_rate_, publish_rate_, high_level_rate_);
  }

  void create_publishers()
  {
    pub_status_    = create_publisher<mowgli_interfaces::msg::Status>("~/status", 10);
    pub_emergency_ = create_publisher<mowgli_interfaces::msg::Emergency>("~/emergency", 10);
    pub_power_     = create_publisher<mowgli_interfaces::msg::Power>("~/power", 10);
    pub_imu_       = create_publisher<sensor_msgs::msg::Imu>("~/imu/data_raw", 10);
  }

  void create_subscribers()
  {
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        on_cmd_vel(msg);
      });
  }

  void create_services()
  {
    srv_mower_control_ = create_service<mowgli_interfaces::srv::MowerControl>(
      "~/mower_control",
      [this](
        const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
        std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res)
      {
        on_mower_control(req, res);
      });

    srv_emergency_stop_ = create_service<mowgli_interfaces::srv::EmergencyStop>(
      "~/emergency_stop",
      [this](
        const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> req,
        std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> res)
      {
        on_emergency_stop(req, res);
      });
  }

  void open_serial_port()
  {
    serial_ = std::make_unique<SerialPort>(serial_port_path_, baud_rate_);

    packet_handler_.set_callback(
      [this](const uint8_t * data, std::size_t len) {
        on_packet_received(data, len);
      });

    if (!serial_->open()) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to open serial port '%s' at %d baud. "
        "The node will retry on each read tick.",
        serial_port_path_.c_str(), baud_rate_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Opened serial port '%s' at %d baud.",
        serial_port_path_.c_str(), baud_rate_);
    }
  }

  void create_timers()
  {
    // Serial read / packet dispatch.
    const auto read_period_ms =
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
    timer_read_ = create_wall_timer(
      read_period_ms, [this]() { read_serial_tick(); });

    // Heartbeat.
    const auto hb_period_ms =
      std::chrono::milliseconds(static_cast<int>(1000.0 / heartbeat_rate_));
    timer_heartbeat_ = create_wall_timer(
      hb_period_ms, [this]() { send_heartbeat(); });

    // High-level state.
    const auto hl_period_ms =
      std::chrono::milliseconds(static_cast<int>(1000.0 / high_level_rate_));
    timer_high_level_ = create_wall_timer(
      hl_period_ms, [this]() { send_high_level_state(); });
  }

  // ---------------------------------------------------------------------------
  // Serial I/O
  // ---------------------------------------------------------------------------

  void read_serial_tick()
  {
    // If the port was never opened or was closed due to an error, attempt to
    // (re)open it.
    if (!serial_->is_open()) {
      if (!serial_->open()) {
        return;  // Still not open; will retry next tick.
      }
      RCLCPP_INFO(get_logger(), "Serial port re-opened successfully.");
    }

    constexpr std::size_t kReadBufSize = 512u;
    uint8_t buf[kReadBufSize];

    // Drain all available bytes in one tick.
    while (true) {
      const ssize_t n = serial_->read(buf, kReadBufSize);
      if (n <= 0) {
        break;
      }
      packet_handler_.feed(buf, static_cast<std::size_t>(n));
    }
  }

  bool send_raw_packet(const uint8_t * data, std::size_t len)
  {
    if (!serial_->is_open()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot send: serial port not open.");
      return false;
    }

    const std::vector<uint8_t> frame = packet_handler_.encode_packet(data, len);
    const ssize_t written =
      serial_->write(frame.data(), frame.size());

    if (written < 0 || static_cast<std::size_t>(written) != frame.size()) {
      RCLCPP_WARN(get_logger(), "Short write or error sending packet.");
      return false;
    }
    return true;
  }

  // ---------------------------------------------------------------------------
  // Packet dispatch (STM32 → ROS2)
  // ---------------------------------------------------------------------------

  void on_packet_received(const uint8_t * data, std::size_t len)
  {
    if (len == 0) {
      return;
    }

    const auto type = static_cast<PacketId>(data[0]);

    switch (type) {
      case PACKET_ID_LL_STATUS:
        handle_status(data, len);
        break;
      case PACKET_ID_LL_IMU:
        handle_imu(data, len);
        break;
      case PACKET_ID_LL_UI_EVENT:
        handle_ui_event(data, len);
        break;
      default:
        RCLCPP_DEBUG(
          get_logger(), "Unhandled packet type 0x%02X (len=%zu)", data[0], len);
        break;
    }
  }

  void handle_status(const uint8_t * data, std::size_t len)
  {
    if (len < sizeof(LlStatus)) {
      RCLCPP_WARN(get_logger(), "Status packet too short: %zu < %zu", len, sizeof(LlStatus));
      return;
    }

    LlStatus pkt{};
    std::memcpy(&pkt, data, sizeof(LlStatus));

    const auto stamp = now();

    // ---- Status message ----
    {
      auto msg = mowgli_interfaces::msg::Status{};
      msg.stamp = stamp;
      msg.mower_status = (pkt.status_bitmask & STATUS_BIT_INITIALIZED) != 0u
        ? mowgli_interfaces::msg::Status::MOWER_STATUS_OK
        : mowgli_interfaces::msg::Status::MOWER_STATUS_INITIALIZING;
      msg.raspberry_pi_power   = (pkt.status_bitmask & STATUS_BIT_RASPI_POWER)  != 0u;
      msg.is_charging          = (pkt.status_bitmask & STATUS_BIT_CHARGING)     != 0u;
      msg.rain_detected        = (pkt.status_bitmask & STATUS_BIT_RAIN)         != 0u;
      msg.sound_module_available = (pkt.status_bitmask & STATUS_BIT_SOUND_AVAIL) != 0u;
      msg.sound_module_busy    = (pkt.status_bitmask & STATUS_BIT_SOUND_BUSY)   != 0u;
      msg.ui_board_available   = (pkt.status_bitmask & STATUS_BIT_UI_AVAIL)     != 0u;
      // mow_enabled and ESC fields are set by mower_control service state.
      msg.mow_enabled          = mow_enabled_;
      msg.esc_power            = mow_enabled_;
      pub_status_->publish(msg);
    }

    // ---- Emergency message ----
    {
      auto msg = mowgli_interfaces::msg::Emergency{};
      msg.stamp = stamp;
      msg.latched_emergency = (pkt.emergency_bitmask & EMERGENCY_BIT_LATCH) != 0u;
      msg.active_emergency  = pkt.emergency_bitmask != 0u;
      if ((pkt.emergency_bitmask & EMERGENCY_BIT_STOP) != 0u) {
        msg.reason = "STOP button";
      } else if ((pkt.emergency_bitmask & EMERGENCY_BIT_LIFT) != 0u) {
        msg.reason = "Lift detected";
      } else if (msg.active_emergency) {
        msg.reason = "Unknown emergency";
      }
      pub_emergency_->publish(msg);
    }

    // ---- Power message ----
    {
      auto msg = mowgli_interfaces::msg::Power{};
      msg.stamp = stamp;
      msg.v_charge      = pkt.v_charge;
      msg.v_battery     = pkt.v_system;
      msg.charge_current = pkt.charging_current;
      msg.charger_enabled = (pkt.status_bitmask & STATUS_BIT_CHARGING) != 0u;
      msg.charger_status = msg.charger_enabled ? "charging" : "idle";
      pub_power_->publish(msg);
    }
  }

  void handle_imu(const uint8_t * data, std::size_t len)
  {
    if (len < sizeof(LlImu)) {
      RCLCPP_WARN(get_logger(), "IMU packet too short: %zu < %zu", len, sizeof(LlImu));
      return;
    }

    LlImu pkt{};
    std::memcpy(&pkt, data, sizeof(LlImu));

    auto msg = sensor_msgs::msg::Imu{};
    msg.header.stamp    = now();
    msg.header.frame_id = "imu_link";

    msg.linear_acceleration.x = static_cast<double>(pkt.acceleration_mss[0]);
    msg.linear_acceleration.y = static_cast<double>(pkt.acceleration_mss[1]);
    msg.linear_acceleration.z = static_cast<double>(pkt.acceleration_mss[2]);

    msg.angular_velocity.x = static_cast<double>(pkt.gyro_rads[0]);
    msg.angular_velocity.y = static_cast<double>(pkt.gyro_rads[1]);
    msg.angular_velocity.z = static_cast<double>(pkt.gyro_rads[2]);

    // Orientation not computed here; fill with identity and mark as unknown.
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;  // Signal: orientation unknown.

    pub_imu_->publish(msg);
  }

  void handle_ui_event(const uint8_t * data, std::size_t len)
  {
    if (len < sizeof(LlUiEvent)) {
      RCLCPP_WARN(
        get_logger(), "UI event packet too short: %zu < %zu", len, sizeof(LlUiEvent));
      return;
    }

    LlUiEvent pkt{};
    std::memcpy(&pkt, data, sizeof(LlUiEvent));

    RCLCPP_INFO(
      get_logger(),
      "UI button event: button_id=%u duration=%u",
      pkt.button_id, pkt.press_duration);
  }

  // ---------------------------------------------------------------------------
  // Periodic transmit (Pi → STM32)
  // ---------------------------------------------------------------------------

  void send_heartbeat()
  {
    LlHeartbeat pkt{};
    pkt.type = PACKET_ID_LL_HEARTBEAT;
    pkt.emergency_requested         = emergency_active_ ? 1u : 0u;
    pkt.emergency_release_requested = emergency_release_pending_ ? 1u : 0u;

    // Consume the one-shot release flag.
    emergency_release_pending_ = false;

    send_raw_packet(
      reinterpret_cast<const uint8_t *>(&pkt),
      sizeof(LlHeartbeat) - sizeof(uint16_t));  // CRC appended by encode_packet.
  }

  void send_high_level_state()
  {
    LlHighLevelState pkt{};
    pkt.type         = PACKET_ID_LL_HIGH_LEVEL_STATE;
    pkt.current_mode = current_mode_;
    pkt.gps_quality  = gps_quality_;

    send_raw_packet(
      reinterpret_cast<const uint8_t *>(&pkt),
      sizeof(LlHighLevelState) - sizeof(uint16_t));
  }

  // ---------------------------------------------------------------------------
  // cmd_vel subscriber
  // ---------------------------------------------------------------------------

  void on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    LlCmdVel pkt{};
    pkt.type      = PACKET_ID_LL_CMD_VEL;
    pkt.linear_x  = static_cast<float>(msg->linear.x);
    pkt.angular_z = static_cast<float>(msg->angular.z);

    send_raw_packet(
      reinterpret_cast<const uint8_t *>(&pkt),
      sizeof(LlCmdVel) - sizeof(uint16_t));
  }

  // ---------------------------------------------------------------------------
  // Service handlers
  // ---------------------------------------------------------------------------

  void on_mower_control(
    const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
    std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res)
  {
    mow_enabled_ = (req->mow_enabled != 0u);

    RCLCPP_INFO(
      get_logger(),
      "MowerControl: mow_enabled=%s mow_direction=%u",
      mow_enabled_ ? "true" : "false",
      req->mow_direction);

    // Send an immediate heartbeat with current emergency state so the STM32
    // picks up the mode change without waiting for the next scheduled tick.
    send_heartbeat();

    res->success = true;
  }

  void on_emergency_stop(
    const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> req,
    std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> res)
  {
    if (req->emergency != 0u) {
      RCLCPP_WARN(get_logger(), "Emergency stop requested via service.");
      emergency_active_ = true;
    } else {
      RCLCPP_INFO(get_logger(), "Emergency release requested via service.");
      emergency_active_          = false;
      emergency_release_pending_ = true;
    }

    send_heartbeat();

    res->success = true;
  }

  // ---------------------------------------------------------------------------
  // Members: ROS2 interfaces
  // ---------------------------------------------------------------------------

  rclcpp::Publisher<mowgli_interfaces::msg::Status>::SharedPtr    pub_status_;
  rclcpp::Publisher<mowgli_interfaces::msg::Emergency>::SharedPtr pub_emergency_;
  rclcpp::Publisher<mowgli_interfaces::msg::Power>::SharedPtr     pub_power_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             pub_imu_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  rclcpp::Service<mowgli_interfaces::srv::MowerControl>::SharedPtr  srv_mower_control_;
  rclcpp::Service<mowgli_interfaces::srv::EmergencyStop>::SharedPtr srv_emergency_stop_;

  rclcpp::TimerBase::SharedPtr timer_read_;
  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_high_level_;

  // ---------------------------------------------------------------------------
  // Members: serial and protocol
  // ---------------------------------------------------------------------------

  std::string serial_port_path_;
  int         baud_rate_{115200};
  double      heartbeat_rate_{4.0};
  double      publish_rate_{100.0};
  double      high_level_rate_{2.0};

  std::unique_ptr<SerialPort> serial_;
  PacketHandler               packet_handler_;

  // ---------------------------------------------------------------------------
  // Members: stateful state communicated to the STM32
  // ---------------------------------------------------------------------------

  bool    emergency_active_{false};
  bool    emergency_release_pending_{false};
  bool    mow_enabled_{false};
  uint8_t current_mode_{0};
  uint8_t gps_quality_{0};
};

}  // namespace mowgli_hardware

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_hardware::HardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
