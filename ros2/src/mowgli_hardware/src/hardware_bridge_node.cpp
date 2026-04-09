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
 *   ~/wheel_odom   nav_msgs/msg/Odometry
 *   /battery_state sensor_msgs/msg/BatteryState  (for opennav_docking)
 *
 * Subscribed topics:
 *   ~/cmd_vel      geometry_msgs/msg/Twist  → LlCmdVel packet to STM32
 *
 * Services:
 *   ~/mower_control  mowgli_interfaces/srv/MowerControl
 *   ~/emergency_stop mowgli_interfaces/srv/EmergencyStop
 *
 * Parameters:
 *   serial_port      (string,  default "/dev/mowgli")
 *   baud_rate        (int,     default 115200)
 *   heartbeat_rate   (double,  default 4.0 Hz  → 250 ms period)
 *   publish_rate     (double,  default 100.0 Hz → 10 ms period)
 *   high_level_rate  (double,  default 2.0 Hz   → 500 ms period)
 */

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mowgli_hardware/ll_datatypes.hpp"
#include "mowgli_hardware/packet_handler.hpp"
#include "mowgli_hardware/serial_port.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/emergency_stop.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

namespace mowgli_hardware
{

using namespace std::chrono_literals;

class HardwareBridgeNode : public rclcpp::Node
{
public:
  explicit HardwareBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
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
    serial_port_path_ = declare_parameter<std::string>("serial_port", "/dev/mowgli");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    heartbeat_rate_ = declare_parameter<double>("heartbeat_rate", 4.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 100.0);
    high_level_rate_ = declare_parameter<double>("high_level_rate", 2.0);
    dock_x_ = declare_parameter<double>("dock_pose_x", 0.0);
    dock_y_ = declare_parameter<double>("dock_pose_y", 0.0);
    dock_yaw_ = declare_parameter<double>("dock_pose_yaw", 0.0);
    // imu_yaw parameter is used by URDF for mounting rotation, not needed here

    RCLCPP_INFO(get_logger(),
                "Parameters: serial_port=%s baud_rate=%d heartbeat_rate=%.1f Hz "
                "publish_rate=%.1f Hz high_level_rate=%.1f Hz",
                serial_port_path_.c_str(),
                baud_rate_,
                heartbeat_rate_,
                publish_rate_,
                high_level_rate_);
  }

  void create_publishers()
  {
    pub_status_ = create_publisher<mowgli_interfaces::msg::Status>("~/status", 10);
    pub_emergency_ = create_publisher<mowgli_interfaces::msg::Emergency>("~/emergency", 10);
    pub_power_ = create_publisher<mowgli_interfaces::msg::Power>("~/power", 10);
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("~/imu/data_raw", 10);
    pub_wheel_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/wheel_odom", 10);
    pub_battery_state_ = create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
    // dock_pose_fix publisher removed — ekf_map doesn't publish TF and nothing
    // reads /odometry/filtered_map. Re-add if ekf_map becomes TF publisher.
  }

  void create_subscribers()
  {
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel",
        10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg)
        {
          on_cmd_vel(msg);
        });

    // Mirror the behavior tree's high-level state to the firmware so it
    // knows when to accept cmd_vel (mode != IDLE).
    sub_hl_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
        "/behavior_tree_node/high_level_status",
        10,
        [this](mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
        {
          current_mode_ = msg->state;
          RCLCPP_DEBUG(get_logger(),
                       "High-level mode updated to %u (%s)",
                       msg->state,
                       msg->state_name.c_str());
        });
  }

  void create_services()
  {
    srv_mower_control_ = create_service<mowgli_interfaces::srv::MowerControl>(
        "~/mower_control",
        [this](const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
               std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res)
        {
          on_mower_control(req, res);
        });

    srv_emergency_stop_ = create_service<mowgli_interfaces::srv::EmergencyStop>(
        "~/emergency_stop",
        [this](const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> req,
               std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> res)
        {
          on_emergency_stop(req, res);
        });
  }

  void open_serial_port()
  {
    serial_ = std::make_unique<SerialPort>(serial_port_path_, baud_rate_);

    packet_handler_.set_callback(
        [this](const uint8_t* data, std::size_t len)
        {
          on_packet_received(data, len);
        });

    if (!serial_->open())
    {
      RCLCPP_ERROR(get_logger(),
                   "Failed to open serial port '%s' at %d baud. "
                   "The node will retry on each read tick.",
                   serial_port_path_.c_str(),
                   baud_rate_);
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Opened serial port '%s' at %d baud.",
                  serial_port_path_.c_str(),
                  baud_rate_);
    }
  }

  void create_timers()
  {
    // Serial read / packet dispatch.
    const auto read_period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
    timer_read_ = create_wall_timer(read_period_ms,
                                    [this]()
                                    {
                                      read_serial_tick();
                                    });

    // Heartbeat.
    const auto hb_period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / heartbeat_rate_));
    timer_heartbeat_ = create_wall_timer(hb_period_ms,
                                         [this]()
                                         {
                                           // On startup, send emergency release for the first few
                                           // heartbeats to clear any watchdog-latched emergency
                                           // from the container restart gap.
                                           if (startup_release_count_ > 0)
                                           {
                                             emergency_release_pending_ = true;
                                             --startup_release_count_;
                                           }
                                           send_heartbeat();
                                         });

    // High-level state.
    const auto hl_period_ms =
        std::chrono::milliseconds(static_cast<int>(1000.0 / high_level_rate_));
    timer_high_level_ = create_wall_timer(hl_period_ms,
                                          [this]()
                                          {
                                            send_high_level_state();
                                          });
  }

  // ---------------------------------------------------------------------------
  // Serial I/O
  // ---------------------------------------------------------------------------

  void read_serial_tick()
  {
    // If the port was never opened or was closed due to an error, attempt to
    // (re)open it.
    if (!serial_->is_open())
    {
      if (!serial_->open())
      {
        return;  // Still not open; will retry next tick.
      }
      RCLCPP_INFO(get_logger(), "Serial port re-opened successfully.");
    }

    constexpr std::size_t kReadBufSize = 512u;
    uint8_t buf[kReadBufSize];

    // Drain all available bytes in one tick.
    while (true)
    {
      const ssize_t n = serial_->read(buf, kReadBufSize);
      if (n <= 0)
      {
        break;
      }
      packet_handler_.feed(buf, static_cast<std::size_t>(n));
    }
  }

  bool send_raw_packet(const uint8_t* data, std::size_t len)
  {
    if (!serial_->is_open())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot send: serial port not open.");
      return false;
    }

    const std::vector<uint8_t> frame = packet_handler_.encode_packet(data, len);
    const ssize_t written = serial_->write(frame.data(), frame.size());

    if (written < 0 || static_cast<std::size_t>(written) != frame.size())
    {
      RCLCPP_WARN(get_logger(), "Short write or error sending packet.");
      return false;
    }
    return true;
  }

  // ---------------------------------------------------------------------------
  // Packet dispatch (STM32 → ROS2)
  // ---------------------------------------------------------------------------

  void on_packet_received(const uint8_t* data, std::size_t len)
  {
    if (len == 0)
    {
      return;
    }

    const auto type = static_cast<PacketId>(data[0]);

    switch (type)
    {
      case PACKET_ID_LL_STATUS:
        handle_status(data, len);
        break;
      case PACKET_ID_LL_IMU:
        handle_imu(data, len);
        break;
      case PACKET_ID_LL_UI_EVENT:
        handle_ui_event(data, len);
        break;
      case PACKET_ID_LL_ODOMETRY:
        handle_odometry(data, len);
        break;
      case PACKET_ID_LL_BLADE_STATUS:
        handle_blade_status(data, len);
        break;
      default:
        RCLCPP_DEBUG(get_logger(), "Unhandled packet type 0x%02X (len=%zu)", data[0], len);
        break;
    }
  }

  void handle_status(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlStatus))
    {
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
      msg.raspberry_pi_power = (pkt.status_bitmask & STATUS_BIT_RASPI_POWER) != 0u;
      is_charging_ = (pkt.status_bitmask & STATUS_BIT_CHARGING) != 0u;
      msg.is_charging = is_charging_;
      msg.rain_detected = (pkt.status_bitmask & STATUS_BIT_RAIN) != 0u;
      msg.sound_module_available = (pkt.status_bitmask & STATUS_BIT_SOUND_AVAIL) != 0u;
      msg.sound_module_busy = (pkt.status_bitmask & STATUS_BIT_SOUND_BUSY) != 0u;
      msg.ui_board_available = (pkt.status_bitmask & STATUS_BIT_UI_AVAIL) != 0u;
      // Blade motor fields from live telemetry
      msg.mow_enabled = mow_enabled_;
      msg.esc_power = mow_enabled_ || blade_active_;
      msg.mower_esc_status = blade_active_ ? 1u : 0u;
      msg.mower_motor_rpm = blade_rpm_;
      msg.mower_motor_temperature = blade_temperature_;
      msg.mower_esc_current = blade_esc_current_;
      pub_status_->publish(msg);
    }

    // ---- Dock pose fix ----
    // Previously published dock position+heading to ekf_map while charging.
    // Removed: ekf_map has publish_tf=false and nothing reads its output.
    // SLAM is the sole TF authority. dock_pose_yaw is only used for SLAM
    // map_start_pose (on saved maps) and by the BT for heading reference.
    // If ekf_map is re-enabled as TF publisher in the future, re-add this.

    // ---- Emergency message ----
    {
      auto msg = mowgli_interfaces::msg::Emergency{};
      msg.stamp = stamp;
      msg.latched_emergency = (pkt.emergency_bitmask & EMERGENCY_BIT_LATCH) != 0u;
      // Active emergency = a trigger (STOP or LIFT) is currently asserted,
      // not just a latched state from a previous event.
      const bool stop_active = (pkt.emergency_bitmask & EMERGENCY_BIT_STOP) != 0u;
      const bool lift_active = (pkt.emergency_bitmask & EMERGENCY_BIT_LIFT) != 0u;
      msg.active_emergency = stop_active || lift_active;
      if (stop_active)
      {
        msg.reason = "STOP button";
      }
      else if (lift_active)
      {
        msg.reason = "Lift detected";
      }
      else if (msg.latched_emergency)
      {
        msg.reason = "Latched (press play button to release)";
      }
      pub_emergency_->publish(msg);
    }

    // ---- Power message ----
    {
      auto msg = mowgli_interfaces::msg::Power{};
      msg.stamp = stamp;
      msg.v_charge = pkt.v_charge;
      msg.v_battery = pkt.v_system;
      msg.charge_current = pkt.charging_current;
      msg.charger_enabled = (pkt.status_bitmask & STATUS_BIT_CHARGING) != 0u;
      msg.charger_status = msg.charger_enabled ? "charging" : "idle";
      pub_power_->publish(msg);
    }

    // ---- BatteryState message (for opennav_docking charge detection) ----
    {
      auto msg = sensor_msgs::msg::BatteryState{};
      msg.header.stamp = stamp;
      msg.header.frame_id = "base_link";
      msg.voltage = pkt.v_system;
      msg.current = pkt.charging_current;
      msg.percentage = static_cast<float>(pkt.batt_percentage) / 100.0f;
      msg.power_supply_status =
          is_charging_ ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                       : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      msg.present = true;
      pub_battery_state_->publish(msg);
    }
  }

  void handle_imu(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlImu))
    {
      RCLCPP_WARN(get_logger(), "IMU packet too short: %zu < %zu", len, sizeof(LlImu));
      return;
    }

    LlImu pkt{};
    std::memcpy(&pkt, data, sizeof(LlImu));

    auto msg = sensor_msgs::msg::Imu{};
    msg.header.stamp = now();
    msg.header.frame_id = "imu_link";

    msg.linear_acceleration.x = static_cast<double>(pkt.acceleration_mss[0]);
    msg.linear_acceleration.y = static_cast<double>(pkt.acceleration_mss[1]);
    msg.linear_acceleration.z = static_cast<double>(pkt.acceleration_mss[2]);

    // Publish raw gyro values without bias compensation.
    // The EKF handles gyro drift via differential mode (imu0_differential: true).
    // v2 (OpenMower/rosserial) published raw gyro and it worked correctly.
    msg.angular_velocity.x = static_cast<double>(pkt.gyro_rads[0]);
    msg.angular_velocity.y = static_cast<double>(pkt.gyro_rads[1]);
    msg.angular_velocity.z = static_cast<double>(pkt.gyro_rads[2]);

    // Magnetometer data is ignored — uncalibrated on metal robot chassis,
    // gives ~229° error vs real heading. dock_pose_yaw is set from config
    // (user measures with phone compass).

    // Write resolved dock pose to file for SLAM initialization.
    // On fresh map start, navigation.launch.py reads this file to set
    // SLAM's map_start_pose so the map frame aligns with GPS/datum.
    if (is_charging_ && !dock_pose_written_)
    {
      const double dx = dock_x_;
      const double dy = dock_y_;
      // dock_yaw_ is from user config only (magnetometer no longer used)
      std::ofstream f("/tmp/dock_start_pose.txt");
      if (f.is_open())
      {
        f << dx << " " << dy << " " << dock_yaw_ << std::endl;
        dock_pose_written_ = true;
        RCLCPP_INFO(get_logger(),
                    "Wrote dock start pose to /tmp/dock_start_pose.txt: [%.2f, %.2f, %.3f]",
                    dx,
                    dy,
                    dock_yaw_);
      }
    }

    // Orientation not computed here; fill with identity and mark as unknown.
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;  // Signal: orientation unknown.

    // Gyro covariance: WT901 gyro z-axis severely under-reports yaw rate
    // (~17% of actual). Set high covariance so the EKF trusts wheel odom
    // angular velocity over the gyro for yaw rate.
    msg.angular_velocity_covariance[0] = 0.1;  // roll rate
    msg.angular_velocity_covariance[4] = 0.1;  // pitch rate
    msg.angular_velocity_covariance[8] = 1.0;  // yaw rate — low confidence

    pub_imu_->publish(msg);
  }

  void handle_ui_event(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlUiEvent))
    {
      RCLCPP_WARN(get_logger(), "UI event packet too short: %zu < %zu", len, sizeof(LlUiEvent));
      return;
    }

    LlUiEvent pkt{};
    std::memcpy(&pkt, data, sizeof(LlUiEvent));

    RCLCPP_INFO(get_logger(),
                "UI button event: button_id=%u duration=%u",
                pkt.button_id,
                pkt.press_duration);
  }

  void handle_odometry(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlOdometry))
    {
      RCLCPP_WARN(get_logger(), "Odometry packet too short: %zu < %zu", len, sizeof(LlOdometry));
      return;
    }

    LlOdometry pkt{};
    std::memcpy(&pkt, data, sizeof(LlOdometry));

    const auto stamp = now();
    const double dt_sec = static_cast<double>(pkt.dt_millis) / 1000.0;

    // Debug: log raw tick values periodically
    static int odom_debug_count = 0;
    if (++odom_debug_count % 50 == 0)
    {
      RCLCPP_INFO(get_logger(),
                  "Odom raw: L=%d R=%d dt=%u spd_L=%d spd_R=%d dir_L=%u dir_R=%u",
                  pkt.left_ticks,
                  pkt.right_ticks,
                  pkt.dt_millis,
                  pkt.left_speed,
                  pkt.right_speed,
                  pkt.left_direction,
                  pkt.right_direction);
    }

    // Compute tick deltas since last packet.
    // Tick counters are cumulative absolute (always increasing).
    // The direction fields indicate forward (1) or reverse (2).
    // Apply sign based on direction so differential kinematics work.
    int32_t d_left = pkt.left_ticks - prev_left_ticks_;
    int32_t d_right = pkt.right_ticks - prev_right_ticks_;
    if (pkt.left_direction == 2)
      d_left = -d_left;  // reverse
    if (pkt.right_direction == 2)
      d_right = -d_right;  // reverse
    prev_left_ticks_ = pkt.left_ticks;
    prev_right_ticks_ = pkt.right_ticks;

    // Track whether wheels are stationary (for gyro bias compensation)
    wheels_stationary_ = (d_left == 0 && d_right == 0);

    // Skip the first packet (no valid delta yet)
    if (!odom_initialized_)
    {
      odom_initialized_ = true;
      return;
    }

    // Convert ticks to metres (TICKS_PER_M = 300.0)
    constexpr double kTicksPerMetre = 300.0;
    constexpr double kWheelBase = 0.325;

    const double d_left_m = static_cast<double>(d_left) / kTicksPerMetre;
    const double d_right_m = static_cast<double>(d_right) / kTicksPerMetre;

    // Differential drive kinematics
    const double d_centre = (d_left_m + d_right_m) / 2.0;
    const double d_theta = (d_right_m - d_left_m) / kWheelBase;

    // Velocity (m/s, rad/s)
    double vx = 0.0;
    double vyaw = 0.0;
    if (dt_sec > 0.001)
    {
      vx = d_centre / dt_sec;
      vyaw = d_theta / dt_sec;
    }

    auto msg = nav_msgs::msg::Odometry{};
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    // When charging AND idle, the robot is mechanically fixed to the dock.
    // Force zero velocity with very tight covariance so the EKF
    // trusts this "not moving" signal over process noise drift.
    // During undocking (current_mode_ != 0), the charger bit may still be
    // set while the robot is backing off the contacts — don't zero odom
    // or SLAM will see zero motion while LiDAR scans shift, corrupting
    // the map.
    const bool force_zero = is_charging_ && (current_mode_ == 0u);
    if (force_zero)
    {
      vx = 0.0;
      vyaw = 0.0;
    }

    msg.twist.twist.linear.x = vx;
    msg.twist.twist.angular.z = vyaw;

    // Covariance: when docked and idle, very tight (certain we're not moving).
    // When driving, loose (wheel slip on grass).
    const double vel_var = force_zero ? 1e-6 : 0.01;
    msg.twist.covariance[0] = vel_var;  // vx variance
    msg.twist.covariance[7] = 1e6;  // vy (no lateral) - very high = unknown
    msg.twist.covariance[14] = 1e6;  // vz - unknown
    msg.twist.covariance[21] = 1e6;  // wx - unknown
    msg.twist.covariance[28] = 1e6;  // wy - unknown
    msg.twist.covariance[35] = force_zero ? 1e-6 : 0.05;  // wz variance

    pub_wheel_odom_->publish(msg);
  }

  // ---------------------------------------------------------------------------
  // Periodic transmit (Pi → STM32)
  // ---------------------------------------------------------------------------

  void send_heartbeat()
  {
    LlHeartbeat pkt{};
    pkt.type = PACKET_ID_LL_HEARTBEAT;
    pkt.emergency_requested = emergency_active_ ? 1u : 0u;
    pkt.emergency_release_requested = emergency_release_pending_ ? 1u : 0u;

    // Consume the one-shot release flag.
    emergency_release_pending_ = false;

    send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(LlHeartbeat) - sizeof(uint16_t));  // CRC appended by encode_packet.
  }

  void send_high_level_state()
  {
    LlHighLevelState pkt{};
    pkt.type = PACKET_ID_LL_HIGH_LEVEL_STATE;
    pkt.current_mode = current_mode_;
    pkt.gps_quality = gps_quality_;

    send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(LlHighLevelState) - sizeof(uint16_t));
  }

  void send_blade_command(uint8_t on, uint8_t dir)
  {
    LlCmdBlade pkt{};
    pkt.type = PACKET_ID_LL_CMD_BLADE;
    pkt.blade_on = on;
    pkt.blade_dir = dir;

    send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt), sizeof(LlCmdBlade) - sizeof(uint16_t));
  }

  void handle_blade_status(const uint8_t* data, std::size_t len)
  {
    if (len < sizeof(LlBladeStatus))
    {
      return;
    }

    LlBladeStatus pkt{};
    std::memcpy(&pkt, data, sizeof(LlBladeStatus));

    // Update the Status message fields with live blade data
    blade_active_ = pkt.is_active != 0u;
    blade_rpm_ = static_cast<float>(pkt.rpm);
    blade_temperature_ = pkt.temperature;
    blade_esc_current_ = static_cast<float>(pkt.power_watts);
  }

  // ---------------------------------------------------------------------------
  // cmd_vel subscriber
  // ---------------------------------------------------------------------------

  void on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    // The firmware ignores cmd_vel when mode is IDLE.  When velocity commands
    // arrive (from Nav2 or teleop), ensure the firmware is in AUTONOMOUS mode.
    if (current_mode_ == 0u && (msg->linear.x != 0.0 || msg->angular.z != 0.0))
    {
      current_mode_ = 1u;  // AUTONOMOUS
      send_high_level_state();
    }

    LlCmdVel pkt{};
    pkt.type = PACKET_ID_LL_CMD_VEL;
    pkt.linear_x = static_cast<float>(msg->linear.x);
    pkt.angular_z = static_cast<float>(msg->angular.z);

    send_raw_packet(reinterpret_cast<const uint8_t*>(&pkt), sizeof(LlCmdVel) - sizeof(uint16_t));
  }

  // ---------------------------------------------------------------------------
  // Service handlers
  // ---------------------------------------------------------------------------

  void on_mower_control(const std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> req,
                        std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> res)
  {
    mow_enabled_ = (req->mow_enabled != 0u);

    RCLCPP_INFO(get_logger(),
                "MowerControl: mow_enabled=%s mow_direction=%u",
                mow_enabled_ ? "true" : "false",
                req->mow_direction);

    // Send blade command to STM32
    send_blade_command(mow_enabled_ ? 1u : 0u, req->mow_direction);

    res->success = true;
  }

  void on_emergency_stop(const std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Request> req,
                         std::shared_ptr<mowgli_interfaces::srv::EmergencyStop::Response> res)
  {
    if (req->emergency != 0u)
    {
      RCLCPP_WARN(get_logger(), "Emergency stop requested via service.");
      emergency_active_ = true;
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Emergency release requested via service.");
      emergency_active_ = false;
      emergency_release_pending_ = true;
    }

    send_heartbeat();

    res->success = true;
  }

  // ---------------------------------------------------------------------------
  // Members: ROS2 interfaces
  // ---------------------------------------------------------------------------

  rclcpp::Publisher<mowgli_interfaces::msg::Status>::SharedPtr pub_status_;
  rclcpp::Publisher<mowgli_interfaces::msg::Emergency>::SharedPtr pub_emergency_;
  rclcpp::Publisher<mowgli_interfaces::msg::Power>::SharedPtr pub_power_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_wheel_odom_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr sub_hl_status_;

  rclcpp::Service<mowgli_interfaces::srv::MowerControl>::SharedPtr srv_mower_control_;
  rclcpp::Service<mowgli_interfaces::srv::EmergencyStop>::SharedPtr srv_emergency_stop_;

  rclcpp::TimerBase::SharedPtr timer_read_;
  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_high_level_;

  // ---------------------------------------------------------------------------
  // Members: serial and protocol
  // ---------------------------------------------------------------------------

  std::string serial_port_path_;
  int baud_rate_{115200};
  double heartbeat_rate_{4.0};
  double publish_rate_{100.0};
  double high_level_rate_{2.0};

  std::unique_ptr<SerialPort> serial_;
  PacketHandler packet_handler_;

  // ---------------------------------------------------------------------------
  // Members: stateful state communicated to the STM32
  // ---------------------------------------------------------------------------

  bool emergency_active_{false};
  bool emergency_release_pending_{false};
  int startup_release_count_{5};  // Send release for first 5 heartbeats
  double dock_x_{0.0};
  double dock_y_{0.0};
  double dock_yaw_{0.0};
  bool mow_enabled_{false};
  bool is_charging_{false};
  uint8_t current_mode_{0};
  uint8_t gps_quality_{0};

  // Blade motor state (updated from LlBladeStatus packets)
  bool blade_active_{false};
  float blade_rpm_{0.0f};
  float blade_temperature_{0.0f};
  float blade_esc_current_{0.0f};

  // Odometry state
  int32_t prev_left_ticks_{0};
  int32_t prev_right_ticks_{0};
  bool odom_initialized_{false};
  bool wheels_stationary_{true};

  // Gyro bias estimate (updated when wheels are stationary)
  double gyro_bias_x_{0.0};
  double gyro_bias_y_{0.0};
  double gyro_bias_z_{0.0};

  bool dock_pose_written_{false};
};

}  // namespace mowgli_hardware

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_hardware::HardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
