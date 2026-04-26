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
 * @file calibrate_imu_yaw_node.hpp
 * @brief Autonomously estimates IMU mounting yaw / pitch / roll vs base_link.
 *
 * The WT901 chip is mounted at an unknown rotation around Z relative to
 * base_link. Pure rotation does not reveal that angle (gyro_z is invariant
 * to yaw mount rotation around the common Z axis). Pure linear acceleration
 * along base_link +X DOES reveal it: accel observed in the chip frame is
 *     (a * cos(imu_yaw), -a * sin(imu_yaw), g)
 * so for non-zero body acceleration a,
 *     imu_yaw = atan2(-ay_chip, ax_chip)
 *
 * Pitch / roll are independently observable from the gravity vector while
 * the robot is perfectly still. With URDF rpy applied before imu_yaw, the
 * chip-frame angles extracted here go directly into mowgli_robot.yaml:
 *     imu_pitch = atan2(-ax_chip, az_chip)
 *     imu_roll  = atan2( ay_chip, az_chip)
 *
 * This node was originally a Python (rclpy) script. It was ported to C++
 * (rclcpp) to dodge a foxglove_bridge ↔ rclpy ↔ cyclonedds cross-language
 * service typesupport dispatch bug that broke calls to this service from
 * the GUI (closes #19). Algorithm and numerical behaviour are preserved
 * 1:1.
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <thread>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_interfaces/msg/calibrate_imu_yaw_status.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/calibrate_imu_yaw.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mowgli_localization
{

class CalibrateImuYawNode : public rclcpp::Node
{
public:
  explicit CalibrateImuYawNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CalibrateImuYawNode() override = default;

private:
  // ---- Sample structs ----------------------------------------------------
  struct ImuSample
  {
    double t;
    double ax;
    double ay;
    double az;
  };
  struct OdomSample
  {
    double t;
    double vx;
    double wz;
  };

  // ---- Numerical core (pure function) ------------------------------------
  struct CalibrationResult
  {
    bool success{false};
    std::string message;
    double imu_yaw_rad{0.0};
    double imu_yaw_deg{0.0};
    int samples_used{0};
    double std_dev_deg{0.0};
    double imu_pitch_rad{0.0};
    double imu_pitch_deg{0.0};
    double imu_roll_rad{0.0};
    double imu_roll_deg{0.0};
    int stationary_samples_used{0};
    double gravity_mag_mps2{0.0};
  };

  CalibrationResult compute_imu_yaw(const std::vector<ImuSample>& imu_samples,
                                    const std::vector<OdomSample>& odom_samples,
                                    std::size_t baseline_count) const;

  // ---- Subscription callbacks --------------------------------------------
  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg);
  void on_emergency(mowgli_interfaces::msg::Emergency::ConstSharedPtr msg);
  void on_high_level_status(mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg);

  // ---- Drive helpers (publish on /cmd_vel_teleop at CMD_RATE_HZ) ---------
  void publish_vx(double vx);
  void drive_profile(double signed_cruise_speed);
  void pause_for(double seconds);

  // ---- BT state-machine helpers ------------------------------------------
  bool call_hlc(uint8_t command, const std::string& label);
  bool wait_for_bt_state(int target, double timeout_sec);

  // ---- Service handler ---------------------------------------------------
  // Returns immediately. The actual calibration runs in a worker thread and
  // publishes its result on the `~/calibrate_status` topic. Response.success
  // means "request accepted to start", not "calibration completed cleanly".
  void on_calibrate(
    const std::shared_ptr<mowgli_interfaces::srv::CalibrateImuYaw::Request> request,
    std::shared_ptr<mowgli_interfaces::srv::CalibrateImuYaw::Response> response);

  // ---- Worker thread + publisher helper ----------------------------------
  // run_calibration is invoked on a dedicated thread spawned by on_calibrate.
  // It performs the drive profile, computes the yaw/pitch/roll, and publishes
  // a final CalibrateImuYawStatus with done=true.
  void run_calibration(const std::string& job_id);

  // publish_status assembles a CalibrateImuYawStatus and publishes it on
  // status_pub_. Used both by run_calibration's terminal publish and by
  // on_calibrate's preflight-fail short-circuit.
  void publish_status(const std::string& job_id, bool done, bool success,
                      const std::string& message,
                      const CalibrationResult* result_or_null);

  // ---- Constants ---------------------------------------------------------
  // Sample filter thresholds (yaw solve)
  static constexpr double WZ_STRAIGHT_THRESHOLD = 0.05;     // rad/s
  static constexpr double ACCEL_BODY_THRESHOLD = 0.3;       // m/s^2
  static constexpr int MIN_SAMPLES = 50;

  // Stationary detection (pitch/roll solve)
  static constexpr double STATIONARY_VX_THRESHOLD = 0.01;   // m/s
  static constexpr double STATIONARY_WZ_THRESHOLD = 0.02;   // rad/s
  static constexpr int MIN_STATIONARY_SAMPLES = 150;

  // Motion profile
  static constexpr double CRUISE_SPEED = 0.5;               // m/s
  static constexpr double RAMP_SEC = 0.5;
  static constexpr double CRUISE_SEC = 1.0;
  static constexpr double PAUSE_SEC = 1.0;
  static constexpr double CMD_RATE_HZ = 20.0;
  static constexpr double SETTLE_SEC = 1.0;
  static constexpr double BASELINE_SEC = 1.5;
  static constexpr int N_CYCLES = 3;

  // HighLevelStatus state codes (mirror HighLevelStatus.msg)
  static constexpr int HL_STATE_NULL = 0;
  static constexpr int HL_STATE_IDLE = 1;
  static constexpr int HL_STATE_AUTONOMOUS = 2;
  static constexpr int HL_STATE_RECORDING = 3;

  // HighLevelControl command codes (mirror HighLevelControl.srv)
  static constexpr uint8_t HL_CMD_RECORD_AREA = 3;
  static constexpr uint8_t HL_CMD_RECORD_CANCEL = 6;

  // ---- ROS interfaces ----------------------------------------------------
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Emergency>::SharedPtr emergency_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr bt_status_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<mowgli_interfaces::msg::CalibrateImuYawStatus>::SharedPtr status_pub_;
  rclcpp::Service<mowgli_interfaces::srv::CalibrateImuYaw>::SharedPtr srv_;
  rclcpp::Client<mowgli_interfaces::srv::HighLevelControl>::SharedPtr hlc_client_;

  // ---- State -------------------------------------------------------------
  std::mutex samples_mu_;
  std::atomic<bool> collecting_{false};
  std::vector<ImuSample> imu_samples_;
  std::vector<OdomSample> odom_samples_;

  std::atomic<bool> is_charging_{false};
  std::atomic<bool> emergency_active_{false};
  std::atomic<int> bt_state_{HL_STATE_NULL};

  // True while a calibration run (preflight + drive + compute + publish) is
  // in flight. Guards against concurrent service calls. The worker thread is
  // detached and clears this flag itself on exit, so the main thread never
  // joins it.
  std::atomic<bool> running_{false};
};

}  // namespace mowgli_localization
