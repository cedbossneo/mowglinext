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
//
// ---------------------------------------------------------------------------
// GPS-SLAM Corrector Node — Umeyama-style alignment
//
// Replaces the previous low-pass corrector. Inspired by CIFASIS
// gnss-stereo-inertial-fusion: accumulate (slam_pose, gps_pose) pairs
// during an alignment phase, solve a weighted SE(2) Umeyama alignment
// for yaw + translation, then freeze map→slam_map. The frozen alignment
// doesn't drift with Float GPS jitter.
//
// State machine:
//   ACCUMULATING:  collect correspondences. Publish identity map→slam_map.
//   ALIGNED:       compute & publish the fitted TF. Stay fixed unless a
//                  re-align is triggered.
//
// Transition ACCUMULATING → ALIGNED requires:
//   (a) at least `min_samples` correspondences (default 20)
//   (b) SLAM pose spread >= `min_motion_m` in both X and Y (yaw observability)
//
// Lever arm: the SLAM pose is base_footprint; GPS reports antenna. We
// apply the forward model
//   p_gps_expected = R_align · (R(yaw_slam) · t_bg + p_slam) + t_align
// so the lever arm is baked into the correspondence before solving.
//
// Covariance weighting: each correspondence is weighted by
// 1 / NavSatFix.position_covariance[0] (horizontal σ²). Float fixes with
// large covariance contribute less than Fixed — this is exactly what
// CIFASIS does for their g2o edges.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace mowgli_localization
{

namespace
{

constexpr double METERS_PER_DEG = 111320.0;

// Per-sample weight from NavSatFix covariance. Defensive against zeros
// and garbage. Falls back to a status-based weight when covariance is
// unknown (position_covariance_type == 0).
double weight_from_fix(const sensor_msgs::msg::NavSatFix& msg)
{
  const double fallback_sigma2 = 1.0;  // 1 m² if we know nothing
  double sigma2 = fallback_sigma2;

  if (msg.position_covariance_type != 0)
  {
    const double cov_xx = msg.position_covariance[0];
    const double cov_yy = msg.position_covariance[4];
    const double mean = 0.5 * (std::max(cov_xx, 0.0) + std::max(cov_yy, 0.0));
    if (mean > 1e-6)
      sigma2 = mean;
  }
  else
  {
    // Without covariance, weight by fix status:
    //   2 = RTK/Fixed  → σ² ≈ 0.001 (2 cm)
    //   1 = DGPS/Float → σ² ≈ 0.05  (20 cm)
    //   0 = basic GPS  → σ² ≈ 4.0   (2 m)
    switch (msg.status.status)
    {
      case 2: sigma2 = 0.001; break;
      case 1: sigma2 = 0.05; break;
      default: sigma2 = 4.0; break;
    }
  }
  return 1.0 / sigma2;
}

}  // namespace

class GpsSlamCorrectorNode : public rclcpp::Node
{
public:
  GpsSlamCorrectorNode() : Node("gps_slam_corrector")
  {
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("slam_frame", "slam_map");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<double>("publish_rate", 10.0);
    declare_parameter<int>("min_samples", 20);
    declare_parameter<double>("min_motion_m", 1.5);
    declare_parameter<int>("min_fix_status", 1);  // accept DGPS+ (Float) during alignment
    declare_parameter<double>("datum_lat", 0.0);
    declare_parameter<double>("datum_lon", 0.0);
    declare_parameter<double>("gps_x", 0.0);
    declare_parameter<double>("gps_y", 0.0);
    declare_parameter<double>("sample_period_sec", 0.2);  // 5 Hz correspondence rate

    map_frame_ = get_parameter("map_frame").as_string();
    slam_frame_ = get_parameter("slam_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_rate_ = get_parameter("publish_rate").as_double();
    min_samples_ = get_parameter("min_samples").as_int();
    min_motion_m_ = get_parameter("min_motion_m").as_double();
    min_fix_status_ = get_parameter("min_fix_status").as_int();
    gps_lever_x_ = get_parameter("gps_x").as_double();
    gps_lever_y_ = get_parameter("gps_y").as_double();
    sample_period_ = get_parameter("sample_period_sec").as_double();

    // Datum: use configured if non-zero, else first-fix.
    const double cfg_lat = get_parameter("datum_lat").as_double();
    const double cfg_lon = get_parameter("datum_lon").as_double();
    if (cfg_lat != 0.0 && cfg_lon != 0.0)
    {
      set_datum(cfg_lat, cfg_lon);
      RCLCPP_INFO(get_logger(),
                  "GPS datum from config: lat=%.7f lon=%.7f",
                  datum_lat_,
                  datum_lon_);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) { on_gps_fix(msg); });

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_alignment",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
        {
          reset();
          resp->success = true;
          resp->message = "Alignment reset; corrector back to ACCUMULATING.";
        });

    publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_),
        [this]() { publish_correction(); });

    RCLCPP_INFO(
        get_logger(),
        "GPS-SLAM corrector (Umeyama): min_samples=%d, min_motion_m=%.2f, lever=(%.3f,%.3f)",
        min_samples_,
        min_motion_m_,
        gps_lever_x_,
        gps_lever_y_);
  }

private:
  struct Correspondence
  {
    // q_i = R(yaw_slam) · t_bg + p_slam  (antenna position in slam_map frame)
    double q_x;
    double q_y;
    // GPS antenna position in ENU (datum-relative)
    double g_x;
    double g_y;
    double weight;
  };

  enum class State
  {
    ACCUMULATING,
    ALIGNED,
  };

  // ─── GPS callback ─────────────────────────────────────────────────────────

  void on_gps_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    if (msg->status.status < min_fix_status_)
      return;

    if (!datum_set_)
    {
      set_datum(msg->latitude, msg->longitude);
      RCLCPP_INFO(
          get_logger(), "GPS datum from first fix: lat=%.7f lon=%.7f",
          datum_lat_,
          datum_lon_);
    }

    const rclcpp::Time stamp = msg->header.stamp;
    const rclcpp::Time now_t = now();
    // Sample rate gate — one correspondence per `sample_period_` seconds.
    if (last_sample_time_.nanoseconds() > 0 &&
        (now_t - last_sample_time_).seconds() < sample_period_)
      return;

    // SLAM pose: robot (base_footprint) in slam_map.
    geometry_msgs::msg::TransformStamped slam_to_base;
    try
    {
      slam_to_base = tf_buffer_->lookupTransform(slam_frame_, base_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException&)
    {
      return;
    }

    const double slam_x = slam_to_base.transform.translation.x;
    const double slam_y = slam_to_base.transform.translation.y;
    const double slam_yaw = tf2::getYaw(slam_to_base.transform.rotation);

    // GPS antenna position in ENU (datum-relative).
    const double g_x = (msg->longitude - datum_lon_) * cos_datum_lat_ * METERS_PER_DEG;
    const double g_y = (msg->latitude - datum_lat_) * METERS_PER_DEG;

    // Antenna position in slam_map frame = R(yaw_slam) · t_bg + p_slam.
    const double c = std::cos(slam_yaw);
    const double s = std::sin(slam_yaw);
    const double q_x = c * gps_lever_x_ - s * gps_lever_y_ + slam_x;
    const double q_y = s * gps_lever_x_ + c * gps_lever_y_ + slam_y;

    Correspondence corr{q_x, q_y, g_x, g_y, weight_from_fix(*msg)};

    if (state_ == State::ALIGNED)
    {
      // After alignment, keep collecting residuals for diagnostics and
      // optional re-alignment trigger, but don't change the published TF.
      // (Re-alignment via /reset_alignment service or restart.)
      record_post_alignment_residual(corr);
      last_sample_time_ = now_t;
      return;
    }

    correspondences_.push_back(corr);
    last_sample_time_ = now_t;

    // Throttled progress logging
    if (correspondences_.size() % 5 == 0)
    {
      RCLCPP_INFO(
          get_logger(),
          "ACCUMULATING: %zu samples (need %d), spread=(%.2f, %.2f) m",
          correspondences_.size(),
          min_samples_,
          slam_spread_x(),
          slam_spread_y());
    }

    // Alignment check
    if (static_cast<int>(correspondences_.size()) >= min_samples_ &&
        slam_spread_x() >= min_motion_m_ && slam_spread_y() >= min_motion_m_)
    {
      try_align();
    }
  }

  // ─── Spread metric for observability ──────────────────────────────────────

  double slam_spread_x() const
  {
    if (correspondences_.empty())
      return 0.0;
    double mn = correspondences_.front().q_x;
    double mx = mn;
    for (const auto& c : correspondences_)
    {
      mn = std::min(mn, c.q_x);
      mx = std::max(mx, c.q_x);
    }
    return mx - mn;
  }

  double slam_spread_y() const
  {
    if (correspondences_.empty())
      return 0.0;
    double mn = correspondences_.front().q_y;
    double mx = mn;
    for (const auto& c : correspondences_)
    {
      mn = std::min(mn, c.q_y);
      mx = std::max(mx, c.q_y);
    }
    return mx - mn;
  }

  // ─── Weighted SE(2) Umeyama ───────────────────────────────────────────────
  // Solve R, t such that g_i ≈ R · q_i + t with weights w_i (closed form).
  //   μ_q = Σ w_i q_i / Σ w_i
  //   μ_g = Σ w_i g_i / Σ w_i
  //   s_i = q_i - μ_q ;  d_i = g_i - μ_g
  //   H = Σ w_i s_i d_iᵀ     (2×2 cross-covariance)
  //   yaw = atan2(H[0,1] - H[1,0], H[0,0] + H[1,1])     (SO(2) case)
  //   t = μ_g - R(yaw) · μ_q

  void try_align()
  {
    double sum_w = 0.0;
    double mean_qx = 0.0, mean_qy = 0.0, mean_gx = 0.0, mean_gy = 0.0;
    for (const auto& c : correspondences_)
    {
      sum_w += c.weight;
      mean_qx += c.weight * c.q_x;
      mean_qy += c.weight * c.q_y;
      mean_gx += c.weight * c.g_x;
      mean_gy += c.weight * c.g_y;
    }
    if (sum_w < 1e-9)
    {
      RCLCPP_WARN(get_logger(), "Alignment skipped: zero total weight");
      return;
    }
    mean_qx /= sum_w;
    mean_qy /= sum_w;
    mean_gx /= sum_w;
    mean_gy /= sum_w;

    double H00 = 0.0, H01 = 0.0, H10 = 0.0, H11 = 0.0;
    for (const auto& c : correspondences_)
    {
      const double sx = c.q_x - mean_qx;
      const double sy = c.q_y - mean_qy;
      const double dx = c.g_x - mean_gx;
      const double dy = c.g_y - mean_gy;
      H00 += c.weight * sx * dx;
      H01 += c.weight * sx * dy;
      H10 += c.weight * sy * dx;
      H11 += c.weight * sy * dy;
    }

    const double yaw = std::atan2(H01 - H10, H00 + H11);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double tx = mean_gx - (cy * mean_qx - sy * mean_qy);
    const double ty = mean_gy - (sy * mean_qx + cy * mean_qy);

    // RMS residual for reporting
    double rms = 0.0;
    double total_w = 0.0;
    for (const auto& c : correspondences_)
    {
      const double px = cy * c.q_x - sy * c.q_y + tx;
      const double py = sy * c.q_x + cy * c.q_y + ty;
      const double ex = c.g_x - px;
      const double ey = c.g_y - py;
      rms += c.weight * (ex * ex + ey * ey);
      total_w += c.weight;
    }
    rms = std::sqrt(rms / std::max(total_w, 1e-9));

    std::lock_guard<std::mutex> lock(align_mutex_);
    align_tx_ = tx;
    align_ty_ = ty;
    align_yaw_ = yaw;
    state_ = State::ALIGNED;

    RCLCPP_INFO(
        get_logger(),
        "ALIGNED: n=%zu yaw=%.3f rad (%.1f deg) t=(%.3f, %.3f) rms=%.3f m",
        correspondences_.size(),
        yaw,
        yaw * 180.0 / M_PI,
        tx,
        ty,
        rms);
  }

  void record_post_alignment_residual(const Correspondence& c)
  {
    double cy, sy, tx, ty;
    {
      std::lock_guard<std::mutex> lock(align_mutex_);
      cy = std::cos(align_yaw_);
      sy = std::sin(align_yaw_);
      tx = align_tx_;
      ty = align_ty_;
    }
    const double px = cy * c.q_x - sy * c.q_y + tx;
    const double py = sy * c.q_x + cy * c.q_y + ty;
    const double ex = c.g_x - px;
    const double ey = c.g_y - py;
    const double err = std::sqrt(ex * ex + ey * ey);

    // EWMA of post-alignment residual error
    constexpr double alpha = 0.05;
    post_residual_rms_ = (1.0 - alpha) * post_residual_rms_ + alpha * err;

    RCLCPP_DEBUG(
        get_logger(), "Post-align residual: %.3f m (ewma %.3f m)", err, post_residual_rms_);
  }

  // ─── Publish map→slam_map TF ──────────────────────────────────────────────

  void publish_correction()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = map_frame_;
    t.child_frame_id = slam_frame_;

    double tx = 0.0, ty = 0.0, yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(align_mutex_);
      if (state_ == State::ALIGNED)
      {
        tx = align_tx_;
        ty = align_ty_;
        yaw = align_yaw_;
      }
    }

    t.transform.translation.x = tx;
    t.transform.translation.y = ty;
    t.transform.translation.z = 0.0;
    const double half = 0.5 * yaw;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = std::sin(half);
    t.transform.rotation.w = std::cos(half);
    tf_broadcaster_->sendTransform(t);
  }

  // ─── Reset ────────────────────────────────────────────────────────────────

  void reset()
  {
    std::lock_guard<std::mutex> lock(align_mutex_);
    state_ = State::ACCUMULATING;
    correspondences_.clear();
    align_tx_ = align_ty_ = align_yaw_ = 0.0;
    post_residual_rms_ = 0.0;
    RCLCPP_WARN(get_logger(), "Alignment reset. Re-accumulating correspondences.");
  }

  // ─── Datum ────────────────────────────────────────────────────────────────

  void set_datum(double lat, double lon)
  {
    datum_lat_ = lat;
    datum_lon_ = lon;
    cos_datum_lat_ = std::cos(datum_lat_ * M_PI / 180.0);
    datum_set_ = true;
  }

  // ─── State ────────────────────────────────────────────────────────────────

  std::string map_frame_, slam_frame_, base_frame_;
  double publish_rate_;
  int min_samples_;
  double min_motion_m_;
  int min_fix_status_;
  double gps_lever_x_, gps_lever_y_;
  double sample_period_;

  bool datum_set_ = false;
  double datum_lat_ = 0.0, datum_lon_ = 0.0, cos_datum_lat_ = 1.0;

  std::mutex align_mutex_;
  State state_ = State::ACCUMULATING;
  double align_tx_ = 0.0, align_ty_ = 0.0, align_yaw_ = 0.0;
  double post_residual_rms_ = 0.0;

  std::vector<Correspondence> correspondences_;
  rclcpp::Time last_sample_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::GpsSlamCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}
