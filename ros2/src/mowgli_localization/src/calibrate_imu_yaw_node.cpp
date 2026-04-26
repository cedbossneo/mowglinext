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
 * @file calibrate_imu_yaw_node.cpp
 * @brief Implementation of CalibrateImuYawNode — see header for the rationale.
 */

#include "mowgli_localization/calibrate_imu_yaw_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <sstream>
#include <thread>

namespace mowgli_localization
{

namespace
{

constexpr double DEG_PER_RAD = 180.0 / M_PI;

double stamp_to_sec(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CalibrateImuYawNode::CalibrateImuYawNode(const rclcpp::NodeOptions& options)
    : Node("calibrate_imu_yaw_node", options)
{
  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  // Sensor data: best effort, depth 50.
  rclcpp::QoS imu_qos(rclcpp::KeepLast(50));
  imu_qos.best_effort();

  // /wheel_odom: reliable, depth 50.
  rclcpp::QoS odom_qos(rclcpp::KeepLast(50));
  odom_qos.reliable();

  // Status / emergency / BT: reliable, depth 10.
  rclcpp::QoS state_qos(rclcpp::KeepLast(10));
  state_qos.reliable();

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", imu_qos,
    [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(msg); }, sub_opts);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/wheel_odom", odom_qos,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(msg); }, sub_opts);

  status_sub_ = create_subscription<mowgli_interfaces::msg::Status>(
    "/hardware_bridge/status", state_qos,
    [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg) { on_status(msg); }, sub_opts);

  emergency_sub_ = create_subscription<mowgli_interfaces::msg::Emergency>(
    "/hardware_bridge/emergency", state_qos,
    [this](mowgli_interfaces::msg::Emergency::ConstSharedPtr msg) { on_emergency(msg); },
    sub_opts);

  bt_status_sub_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
    "/behavior_tree_node/high_level_status", state_qos,
    [this](mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
    { on_high_level_status(msg); },
    sub_opts);

  // Teleop-priority cmd_vel matches the GUI joystick path so twist_mux gives
  // it the right priority and downstream safety still applies.
  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_teleop", state_qos);

  // Status topic carries the calibration result (the reason this exists is
  // documented in CalibrateImuYaw.srv). transient_local + depth=1 lets a
  // subscriber that attaches AFTER the run still receive the final status —
  // job_id matching in the message handles the resulting stale-cache risk.
  rclcpp::QoS status_qos(rclcpp::KeepLast(1));
  status_qos.reliable();
  status_qos.transient_local();
  status_pub_ =
    create_publisher<mowgli_interfaces::msg::CalibrateImuYawStatus>("~/calibrate_status",
                                                                    status_qos);

  hlc_client_ = create_client<mowgli_interfaces::srv::HighLevelControl>(
    "/behavior_tree_node/high_level_control", rclcpp::ServicesQoS(), cb_group_);

  srv_ = create_service<mowgli_interfaces::srv::CalibrateImuYaw>(
    "~/calibrate",
    [this](const std::shared_ptr<mowgli_interfaces::srv::CalibrateImuYaw::Request> req,
           std::shared_ptr<mowgli_interfaces::srv::CalibrateImuYaw::Response> res)
    { on_calibrate(req, res); },
    rclcpp::ServicesQoS(), cb_group_);

  RCLCPP_INFO(get_logger(),
              "IMU yaw calibration node ready. Ensure robot is undocked with "
              "~1 m of clear space in front and behind, then call ~/calibrate.");
}

// ---------------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------------

void CalibrateImuYawNode::on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!collecting_.load()) {
    return;
  }
  ImuSample s{stamp_to_sec(msg->header.stamp),
              msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z};
  std::lock_guard<std::mutex> lock(samples_mu_);
  imu_samples_.push_back(s);
}

void CalibrateImuYawNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!collecting_.load()) {
    return;
  }
  OdomSample s{stamp_to_sec(msg->header.stamp), msg->twist.twist.linear.x,
               msg->twist.twist.angular.z};
  std::lock_guard<std::mutex> lock(samples_mu_);
  odom_samples_.push_back(s);
}

void CalibrateImuYawNode::on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg)
{
  is_charging_.store(msg->is_charging);
}

void CalibrateImuYawNode::on_emergency(mowgli_interfaces::msg::Emergency::ConstSharedPtr msg)
{
  emergency_active_.store(msg->active_emergency || msg->latched_emergency);
}

void CalibrateImuYawNode::on_high_level_status(
  mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
{
  bt_state_.store(static_cast<int>(msg->state));
}

// ---------------------------------------------------------------------------
// Drive helpers
// ---------------------------------------------------------------------------

void CalibrateImuYawNode::publish_vx(double vx)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "base_footprint";
  msg.twist.linear.x = vx;
  cmd_pub_->publish(msg);
}

void CalibrateImuYawNode::drive_profile(double signed_cruise_speed)
{
  const auto period = std::chrono::duration<double>(1.0 / CMD_RATE_HZ);

  // Acceleration ramp.
  int n = std::max(1, static_cast<int>(RAMP_SEC * CMD_RATE_HZ));
  for (int i = 0; i < n; ++i) {
    double v = signed_cruise_speed * static_cast<double>(i + 1) / static_cast<double>(n);
    publish_vx(v);
    std::this_thread::sleep_for(period);
  }
  // Cruise.
  n = std::max(1, static_cast<int>(CRUISE_SEC * CMD_RATE_HZ));
  for (int i = 0; i < n; ++i) {
    publish_vx(signed_cruise_speed);
    std::this_thread::sleep_for(period);
  }
  // Deceleration ramp.
  n = std::max(1, static_cast<int>(RAMP_SEC * CMD_RATE_HZ));
  for (int i = 0; i < n; ++i) {
    double v = signed_cruise_speed * static_cast<double>(n - i - 1) / static_cast<double>(n);
    publish_vx(v);
    std::this_thread::sleep_for(period);
  }
  publish_vx(0.0);
}

void CalibrateImuYawNode::pause_for(double seconds)
{
  const auto period = std::chrono::duration<double>(1.0 / CMD_RATE_HZ);
  int n = std::max(1, static_cast<int>(seconds * CMD_RATE_HZ));
  for (int i = 0; i < n; ++i) {
    publish_vx(0.0);
    std::this_thread::sleep_for(period);
  }
}

bool CalibrateImuYawNode::call_hlc(uint8_t command, const std::string& label)
{
  if (!hlc_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_WARN(get_logger(), "HighLevelControl service not available — cannot %s.",
                label.c_str());
    return false;
  }
  auto req = std::make_shared<mowgli_interfaces::srv::HighLevelControl::Request>();
  req->command = command;
  auto fut = hlc_client_->async_send_request(req).future;
  if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "HighLevelControl %s timed out.", label.c_str());
    return false;
  }
  auto resp = fut.get();
  if (!resp || !resp->success) {
    RCLCPP_WARN(get_logger(), "HighLevelControl %s rejected by BT.", label.c_str());
    return false;
  }
  return true;
}

bool CalibrateImuYawNode::wait_for_bt_state(int target, double timeout_sec)
{
  const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::duration<double>(timeout_sec));
  while (std::chrono::steady_clock::now() < deadline) {
    if (bt_state_.load() == target) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return bt_state_.load() == target;
}

// ---------------------------------------------------------------------------
// Service handler — thin gate that returns immediately
// ---------------------------------------------------------------------------
//
// The actual calibration runs in a worker thread; the result is delivered on
// the `~/calibrate_status` topic, not in this response. See header rationale.

void CalibrateImuYawNode::on_calibrate(
  const std::shared_ptr<mowgli_interfaces::srv::CalibrateImuYaw::Request> request,
  std::shared_ptr<mowgli_interfaces::srv::CalibrateImuYaw::Response> response)
{
  const std::string job_id = request->job_id;

  // Reject overlapping calls. compare_exchange would be cleaner but exchange
  // is fine here since we never re-enter the no-running branch.
  if (running_.exchange(true)) {
    publish_status(job_id, /*done=*/true, /*success=*/false,
                   "Another calibration run is already in progress.", nullptr);
    response->success = false;
    return;
  }

  // ---- Preflight checks ------------------------------------------------
  if (is_charging_.load()) {
    publish_status(job_id, true, false,
                   "Refusing to calibrate while charging — undock the robot first. "
                   "Calibration drives the robot ~0.6 m forward then back.",
                   nullptr);
    running_.store(false);
    response->success = false;
    return;
  }
  if (emergency_active_.load()) {
    publish_status(job_id, true, false,
                   "Refusing to calibrate while an emergency is active/latched. "
                   "Clear the emergency first.",
                   nullptr);
    running_.store(false);
    response->success = false;
    return;
  }
  if (bt_state_.load() == HL_STATE_AUTONOMOUS) {
    publish_status(job_id, true, false,
                   "Refusing to calibrate while BT is AUTONOMOUS (mowing in progress). "
                   "Stop mowing first (HOME command).",
                   nullptr);
    running_.store(false);
    response->success = false;
    return;
  }

  // Preflight passed — kick off the worker thread and return success.
  std::thread(&CalibrateImuYawNode::run_calibration, this, job_id).detach();
  response->success = true;
}

// ---------------------------------------------------------------------------
// Worker thread — full calibration drive + compute + final publish
// ---------------------------------------------------------------------------

void CalibrateImuYawNode::run_calibration(const std::string& job_id)
{
  // ---- Enter RECORDING so firmware accepts cmd_vel_teleop --------------
  // Firmware's on_cmd_vel() drops commands when BT reports IDLE. RECORDING
  // lets teleop through without spinning the blade; CANCEL discards the
  // in-progress trajectory so we don't pollute the area list.
  bool need_exit_recording = false;
  if (bt_state_.load() != HL_STATE_RECORDING) {
    RCLCPP_INFO(get_logger(), "Entering RECORDING mode for calibration drive.");
    if (!call_hlc(HL_CMD_RECORD_AREA, "enter recording")) {
      publish_status(job_id, true, false,
                     "Could not enter RECORDING mode via BT. Check that the "
                     "behavior_tree_node is alive.",
                     nullptr);
      running_.store(false);
      return;
    }
    if (!wait_for_bt_state(HL_STATE_RECORDING, 5.0)) {
      call_hlc(HL_CMD_RECORD_CANCEL, "cancel after failed entry");
      std::ostringstream m;
      m << "BT did not transition to RECORDING within 5s (stuck at state="
        << bt_state_.load() << ").";
      publish_status(job_id, true, false, m.str(), nullptr);
      running_.store(false);
      return;
    }
    need_exit_recording = true;
  }

  {
    std::lock_guard<std::mutex> lock(samples_mu_);
    imu_samples_.clear();
    odom_samples_.clear();
  }
  collecting_.store(true);

  // ---- Stationary baseline --------------------------------------------
  // Capture IMU while still to measure the DC offset on accel_x / accel_y
  // caused by chassis tilt and sensor bias. That offset is subtracted from
  // motion samples before computing per-sample imu_yaw, otherwise gravity
  // leaks dominate the result (raw ax of ±2 m/s² were observed during a
  // 0.2 m/s² drive — baseline was ~±0.8 m/s² from tilt alone).
  RCLCPP_INFO(get_logger(), "Capturing %.1fs stationary baseline before drive.",
              BASELINE_SEC);
  pause_for(BASELINE_SEC);

  // Snapshot baseline sample count so we can split it from the motion data.
  std::size_t baseline_start_count = 0;
  {
    std::lock_guard<std::mutex> lock(samples_mu_);
    baseline_start_count = imu_samples_.size();
  }

  const double total_motion_sec =
    2.0 * (RAMP_SEC + CRUISE_SEC + RAMP_SEC) + PAUSE_SEC;
  RCLCPP_INFO(get_logger(),
              "Autonomous calibration drive starting — forward %.2f m/s then back, "
              "~%.0fs total per cycle (%d cycles).",
              CRUISE_SPEED, total_motion_sec, N_CYCLES);

  bool drive_errored = false;
  std::string drive_error_msg;
  try {
    for (int cycle = 0; cycle < N_CYCLES; ++cycle) {
      RCLCPP_INFO(get_logger(), "Drive cycle %d/%d", cycle + 1, N_CYCLES);
      drive_profile(+CRUISE_SPEED);
      pause_for(PAUSE_SEC);
      drive_profile(-CRUISE_SPEED);
      pause_for(SETTLE_SEC);
    }
  } catch (const std::exception& e) {
    drive_errored = true;
    drive_error_msg = e.what();
  }

  // Belt-and-braces stop: a few zero commands after the profile in any case.
  for (int i = 0; i < 5; ++i) {
    publish_vx(0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (drive_errored) {
    collecting_.store(false);
    if (need_exit_recording) {
      call_hlc(HL_CMD_RECORD_CANCEL, "cancel after drive error");
    }
    publish_status(job_id, true, false, "Drive profile errored: " + drive_error_msg,
                   nullptr);
    running_.store(false);
    return;
  }

  // ---- Exit RECORDING cleanly — CANCEL discards the trajectory --------
  if (need_exit_recording) {
    call_hlc(HL_CMD_RECORD_CANCEL, "cancel recording");
    wait_for_bt_state(HL_STATE_IDLE, 3.0);
  }

  collecting_.store(false);

  std::vector<ImuSample> imu_snapshot;
  std::vector<OdomSample> odom_snapshot;
  {
    std::lock_guard<std::mutex> lock(samples_mu_);
    imu_snapshot = imu_samples_;
    odom_snapshot = odom_samples_;
  }

  RCLCPP_INFO(get_logger(), "Drive complete — %zu IMU / %zu odom samples collected.",
              imu_snapshot.size(), odom_snapshot.size());

  CalibrationResult result =
    compute_imu_yaw(imu_snapshot, odom_snapshot, baseline_start_count);

  if (result.success) {
    RCLCPP_INFO(get_logger(),
                "imu_yaw = %+.4f rad (%+.2f°) from %d samples, stddev %.2f°",
                result.imu_yaw_rad, result.imu_yaw_deg, result.samples_used,
                result.std_dev_deg);
    if (result.stationary_samples_used >= MIN_STATIONARY_SAMPLES) {
      RCLCPP_INFO(get_logger(),
                  "imu_pitch = %+.4f rad (%+.2f°), imu_roll = %+.4f rad (%+.2f°) "
                  "from %d stationary samples (|g|=%.3f m/s²). Promote to "
                  "mowgli_robot.yaml if |pitch| or |roll| > 1°.",
                  result.imu_pitch_rad, result.imu_pitch_deg, result.imu_roll_rad,
                  result.imu_roll_deg, result.stationary_samples_used,
                  result.gravity_mag_mps2);
    } else {
      RCLCPP_WARN(get_logger(),
                  "Pitch/roll not computed: only %d stationary IMU samples "
                  "(need ≥ %d).",
                  result.stationary_samples_used, MIN_STATIONARY_SAMPLES);
    }
  } else {
    RCLCPP_WARN(get_logger(), "Calibration failed: %s", result.message.c_str());
  }

  publish_status(job_id, /*done=*/true, result.success, result.message, &result);
  running_.store(false);
}

// ---------------------------------------------------------------------------
// Status topic publish helper
// ---------------------------------------------------------------------------

void CalibrateImuYawNode::publish_status(const std::string& job_id, bool done,
                                         bool success, const std::string& message,
                                         const CalibrationResult* r)
{
  mowgli_interfaces::msg::CalibrateImuYawStatus msg;
  msg.job_id = job_id;
  msg.done = done;
  msg.success = success;
  msg.message = message;
  if (r) {
    msg.imu_yaw_rad = r->imu_yaw_rad;
    msg.imu_yaw_deg = r->imu_yaw_deg;
    msg.samples_used = r->samples_used;
    msg.std_dev_deg = r->std_dev_deg;
    msg.imu_pitch_rad = r->imu_pitch_rad;
    msg.imu_pitch_deg = r->imu_pitch_deg;
    msg.imu_roll_rad = r->imu_roll_rad;
    msg.imu_roll_deg = r->imu_roll_deg;
    msg.stationary_samples_used = r->stationary_samples_used;
    msg.gravity_mag_mps2 = r->gravity_mag_mps2;
  }
  status_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// Numerical core — pure function over collected samples
// ---------------------------------------------------------------------------

CalibrateImuYawNode::CalibrationResult CalibrateImuYawNode::compute_imu_yaw(
  const std::vector<ImuSample>& imu_samples_in,
  const std::vector<OdomSample>& odom_samples_in,
  std::size_t baseline_count) const
{
  CalibrationResult r;

  if (odom_samples_in.size() < 3) {
    std::ostringstream m;
    m << "Not enough wheel_odom samples (" << odom_samples_in.size()
      << "). Is /wheel_odom being published?";
    r.message = m.str();
    return r;
  }

  // Split baseline (stationary, before motion) from motion samples. Compute
  // the DC offset on ax/ay from the baseline; subtract it from motion data
  // so we read motion-induced horizontal accel only.
  double baseline_ax = 0.0;
  double baseline_ay = 0.0;
  std::vector<ImuSample> baseline_samples;
  std::vector<ImuSample> motion_samples;
  if (baseline_count > 0 && imu_samples_in.size() > baseline_count) {
    baseline_samples.assign(imu_samples_in.begin(),
                            imu_samples_in.begin() + static_cast<std::ptrdiff_t>(baseline_count));
    motion_samples.assign(imu_samples_in.begin() + static_cast<std::ptrdiff_t>(baseline_count),
                          imu_samples_in.end());
    double sum_ax = 0.0, sum_ay = 0.0;
    for (const auto& s : baseline_samples) {
      sum_ax += s.ax;
      sum_ay += s.ay;
    }
    baseline_ax = sum_ax / static_cast<double>(baseline_samples.size());
    baseline_ay = sum_ay / static_cast<double>(baseline_samples.size());
  } else {
    motion_samples = imu_samples_in;
  }

  if (static_cast<int>(motion_samples.size()) < MIN_SAMPLES) {
    std::ostringstream m;
    m << "Not enough /imu/data samples (" << motion_samples.size()
      << "). Is the IMU running?";
    r.message = m.str();
    return r;
  }

  // Sort copies by timestamp so the bracketing search is monotone.
  std::vector<OdomSample> odom = odom_samples_in;
  std::sort(odom.begin(), odom.end(),
            [](const OdomSample& a, const OdomSample& b) { return a.t < b.t; });
  std::sort(motion_samples.begin(), motion_samples.end(),
            [](const ImuSample& a, const ImuSample& b) { return a.t < b.t; });

  if (odom.size() < 3) {
    r.message = "Not enough odom samples for central difference.";
    return r;
  }

  // Body acceleration via central difference on odom_vx.
  std::vector<double> a_body(odom.size(), 0.0);
  for (std::size_t i = 1; i + 1 < odom.size(); ++i) {
    double dt = std::max(odom[i + 1].t - odom[i - 1].t, 1e-6);
    a_body[i] = (odom[i + 1].vx - odom[i - 1].vx) / dt;
  }
  a_body.front() = a_body[1];
  a_body.back() = a_body[a_body.size() - 2];

  // Filter and accumulate.
  double sum_ax_signed = 0.0;
  double sum_ay_signed = 0.0;
  double sum_abs = 0.0;
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  int valid_count = 0;
  int per_sample_count = 0;

  // For pitch/roll we need samples where the robot was stationary. Track
  // them in parallel here so we do a single pass over motion_samples.
  std::vector<ImuSample> still_samples;

  for (const auto& s : motion_samples) {
    // Find bracketing odom samples for s.t.
    auto it_right = std::lower_bound(
      odom.begin(), odom.end(), s.t,
      [](const OdomSample& o, double t) { return o.t < t; });
    if (it_right == odom.begin()) {
      it_right = odom.begin() + 1;
    } else if (it_right == odom.end()) {
      it_right = odom.end() - 1;
    }
    auto it_left = it_right - 1;
    std::size_t li = static_cast<std::size_t>(it_left - odom.begin());
    std::size_t ri = static_cast<std::size_t>(it_right - odom.begin());

    double dt = std::max(odom[ri].t - odom[li].t, 1e-9);
    double frac = (s.t - odom[li].t) / dt;
    if (frac < 0.0) {
      frac = 0.0;
    }
    if (frac > 1.0) {
      frac = 1.0;
    }
    double wz_interp = odom[li].wz + (odom[ri].wz - odom[li].wz) * frac;
    double a_interp = a_body[li] + (a_body[ri] - a_body[li]) * frac;
    double vx_interp = odom[li].vx + (odom[ri].vx - odom[li].vx) * frac;

    bool straight = std::abs(wz_interp) < WZ_STRAIGHT_THRESHOLD;
    bool moving = std::abs(a_interp) > ACCEL_BODY_THRESHOLD;

    if (straight && moving) {
      double ax_v = s.ax - baseline_ax;
      double ay_v = s.ay - baseline_ay;
      double sign_a = (a_interp > 0.0) ? 1.0 : (a_interp < 0.0 ? -1.0 : 0.0);
      double ax_s = ax_v * sign_a;
      double ay_s = ay_v * sign_a;

      // Vector-sum yaw solve: chip-frame mean accel for body +X is
      // (a*cos(imu_yaw), -a*sin(imu_yaw)). Sum gives a vector along that
      // direction; zero-mean noise cancels as 1/sqrt(N). This is
      // magnitude-weighted (large-|a| samples dominate) — better than the
      // old per-sample atan2 average that exploded when WT901 noise (~0.5
      // m/s²) approached peak body accel (~1 m/s²).
      sum_ax_signed += ax_s;
      sum_ay_signed += ay_s;
      sum_abs += std::hypot(ax_s, ay_s);

      // Per-sample diagnostic: a noisy run shows up as a low resultant on
      // unit-circle averaging even when the vector sum resolves cleanly.
      double per_sample_yaw = std::atan2(-ay_s, ax_s);
      sum_cos += std::cos(per_sample_yaw);
      sum_sin += std::sin(per_sample_yaw);
      ++per_sample_count;

      ++valid_count;
    }

    // Stationary sample harvesting for pitch/roll.
    if (std::abs(vx_interp) < STATIONARY_VX_THRESHOLD &&
        std::abs(wz_interp) < STATIONARY_WZ_THRESHOLD) {
      still_samples.push_back(s);
    }
  }

  if (valid_count < MIN_SAMPLES) {
    r.samples_used = valid_count;
    std::ostringstream m;
    m << "Only " << valid_count << " valid samples (need ≥ " << MIN_SAMPLES
      << "). Is the robot stuck, colliding, or on uneven ground?";
    r.message = m.str();
    return r;
  }

  double imu_yaw_rad = std::atan2(-sum_ay_signed, sum_ax_signed);

  // Confidence: vector-sum magnitude / sum of per-sample magnitudes.
  // 1.0 = perfectly aligned, 0.0 = uniform random directions.
  double r_bar = (sum_abs > 1e-9)
                   ? std::hypot(sum_ax_signed, sum_ay_signed) / sum_abs
                   : 0.0;
  if (r_bar < 1e-9) {
    r_bar = 1e-9;
  }
  if (r_bar > 1.0) {
    r_bar = 1.0;
  }
  double std_rad = std::sqrt(-2.0 * std::log(r_bar));

  double per_sample_r = 0.0;
  if (per_sample_count > 0) {
    double mean_cos = sum_cos / static_cast<double>(per_sample_count);
    double mean_sin = sum_sin / static_cast<double>(per_sample_count);
    per_sample_r = std::hypot(mean_cos, mean_sin);
  }

  // ---- Pitch / roll from stationary samples ----------------------------
  // Combine baseline (pre-drive stillness) with samples where wheel vx and
  // wz were both below the stationary thresholds. At rest the chip-frame
  // accel reads pure gravity rotated by URDF (roll·pitch·yaw); pitch/roll
  // are independent of the yaw solve (they live in the chip frame, pre-yaw).
  std::vector<ImuSample> all_still;
  all_still.reserve(baseline_samples.size() + still_samples.size());
  all_still.insert(all_still.end(), baseline_samples.begin(), baseline_samples.end());
  all_still.insert(all_still.end(), still_samples.begin(), still_samples.end());

  int stationary_count = static_cast<int>(all_still.size());
  double pitch_rad = 0.0;
  double roll_rad = 0.0;
  double gravity_mag = 0.0;
  if (stationary_count >= MIN_STATIONARY_SAMPLES) {
    double sax = 0.0, say = 0.0, saz = 0.0;
    for (const auto& s : all_still) {
      sax += s.ax;
      say += s.ay;
      saz += s.az;
    }
    double n = static_cast<double>(stationary_count);
    double mean_ax = sax / n;
    double mean_ay = say / n;
    double mean_az = saz / n;
    pitch_rad = std::atan2(-mean_ax, mean_az);
    roll_rad = std::atan2(mean_ay, mean_az);
    gravity_mag = std::sqrt(mean_ax * mean_ax + mean_ay * mean_ay + mean_az * mean_az);
  }

  std::ostringstream msg;
  msg << "imu_yaw=" << std::fixed;
  msg.precision(2);
  msg << (imu_yaw_rad * DEG_PER_RAD) << "° from " << valid_count
      << " samples (vector R=" << r_bar << ", per-sample R=" << per_sample_r << ").";
  if (stationary_count >= MIN_STATIONARY_SAMPLES) {
    msg << " pitch=" << (pitch_rad * DEG_PER_RAD) << "°"
        << " roll=" << (roll_rad * DEG_PER_RAD) << "°"
        << " from " << stationary_count << " stationary samples"
        << " (|g|=" << gravity_mag << " m/s²).";
  }

  r.success = true;
  r.message = msg.str();
  r.imu_yaw_rad = imu_yaw_rad;
  r.imu_yaw_deg = imu_yaw_rad * DEG_PER_RAD;
  r.samples_used = valid_count;
  r.std_dev_deg = std_rad * DEG_PER_RAD;
  r.imu_pitch_rad = pitch_rad;
  r.imu_pitch_deg = pitch_rad * DEG_PER_RAD;
  r.imu_roll_rad = roll_rad;
  r.imu_roll_deg = roll_rad * DEG_PER_RAD;
  r.stationary_samples_used = stationary_count;
  r.gravity_mag_mps2 = gravity_mag;
  return r;
}

}  // namespace mowgli_localization

// ---------------------------------------------------------------------------
// main — multi-threaded executor so the service handler's blocking sleep
// loops don't starve subscription delivery (sample callbacks must keep
// running while drive_profile() iterates).
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mowgli_localization::CalibrateImuYawNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
