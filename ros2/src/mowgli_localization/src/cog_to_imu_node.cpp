// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// cog_to_imu_node.cpp
//
// C++ port of scripts/cog_to_imu.py — derives a GPS course-over-ground
// heading from successive RTK-Fixed positions and publishes it as a
// sensor_msgs/Imu absolute-yaw observation on /imu/cog_heading. Also
// runs an online magnetometer calibration least-squares fit using
// COG yaw as ground-truth.
//
// Topics, parameters, formulas and gating thresholds match the Python
// implementation 1:1.

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace mowgli_localization
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kMetersPerDeg = 111319.49079327357;

rclcpp::QoS sensor_qos()
{
  // Match Python QoSProfile(BEST_EFFORT, VOLATILE, KEEP_LAST, depth=10).
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

std::string utc_iso8601_now()
{
  // Format identical enough to Python datetime.isoformat() with tz info
  // (used as a free-form string in the YAML — consumers don't parse it).
  const auto now = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                      now.time_since_epoch())
                      .count() %
                  1000000;
  std::tm tm_utc{};
  gmtime_r(&tt, &tm_utc);
  char buf[64];
  std::snprintf(buf, sizeof(buf),
                "%04d-%02d-%02dT%02d:%02d:%02d.%06ld+00:00",
                tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday,
                tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec,
                static_cast<long>(us));
  return std::string(buf);
}
}  // namespace

class CogToImuNode : public rclcpp::Node
{
public:
  CogToImuNode() : Node("cog_to_imu")
  {
    // ── Sample-pair gating thresholds ────────────────────────────────
    min_abs_wheel_       = declare_parameter<double>("min_abs_wheel_ms", 0.05);
    max_pos_accuracy_    = declare_parameter<double>("max_pos_accuracy_m", 0.05);
    min_dt_              = declare_parameter<double>("min_sample_dt_s", 0.05);
    max_dt_              = declare_parameter<double>("max_sample_dt_s", 0.50);
    max_yaw_var_         = declare_parameter<double>("max_yaw_variance", 3.0);
    min_yaw_var_         = declare_parameter<double>("min_yaw_variance", 7.6e-5);

    // ── Stationary yaw latch ────────────────────────────────────────
    stationary_seed_rate_hz_ =
        declare_parameter<double>("stationary_seed_rate_hz", 2.0);
    stationary_drift_rate_ =
        declare_parameter<double>("stationary_yaw_drift_rate", 0.005);
    stationary_max_age_s_ =
        declare_parameter<double>("stationary_max_age_s", 600.0);

    // Datum for flat-earth ENU projection.
    datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
    datum_lon_ = declare_parameter<double>("datum_lon", 0.0);
    datum_seeded_ = (datum_lat_ != 0.0 || datum_lon_ != 0.0);
    cos_datum_lat_ = std::cos(datum_lat_ * kDegToRad);

    // ── Online mag calibration parameters ───────────────────────────
    enable_mag_cal_ = declare_parameter<bool>("enable_mag_cal", true);
    mag_cal_path_ = declare_parameter<std::string>(
        "mag_calibration_path", "/ros2_ws/maps/mag_calibration.yaml");
    mag_min_samples_ = declare_parameter<int>("mag_min_samples", 30);
    mag_max_samples_ = declare_parameter<int>("mag_max_samples", 600);
    mag_refit_period_sec_ =
        declare_parameter<double>("mag_refit_period_sec", 60.0);
    mag_refit_throttle_sec_ =
        declare_parameter<double>("mag_refit_throttle_sec", 300.0);

    auto qos = sensor_qos();
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", qos,
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) { on_fix(*msg); });
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odom", qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_wheel(*msg); });
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/cog_heading", qos);

    if (enable_mag_cal_)
    {
      mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
          "/imu/mag_raw", qos,
          [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
          { on_mag_raw(*msg); });
      mag_refit_timer_ = create_wall_timer(
          std::chrono::duration<double>(mag_refit_period_sec_),
          [this]() { maybe_refit_mag(); });
    }

    stats_timer_ = create_wall_timer(std::chrono::seconds(30),
                                     [this]() { log_stats(); });
    if (stationary_seed_rate_hz_ > 0.0)
    {
      stationary_timer_ = create_wall_timer(
          std::chrono::duration<double>(1.0 / stationary_seed_rate_hz_),
          [this]() { republish_latched_when_stationary(); });
    }

    RCLCPP_INFO(get_logger(),
                "cog_to_imu started — publish /imu/cog_heading on every "
                "RTK-Fixed sample with |wheel_vx| > %.2f m/s (forward + "
                "reverse, adaptive covariance, no hard speed gate)",
                min_abs_wheel_);
  }

private:
  void on_wheel(const nav_msgs::msg::Odometry & msg)
  {
    wheel_vx_ = msg.twist.twist.linear.x;
  }

  void on_mag_raw(const sensor_msgs::msg::MagneticField & msg)
  {
    latest_mag_ = std::array<double, 3>{
        static_cast<double>(msg.magnetic_field.x) * 1e6,
        static_cast<double>(msg.magnetic_field.y) * 1e6,
        static_cast<double>(msg.magnetic_field.z) * 1e6};
    latest_mag_valid_ = true;
  }

  void on_fix(const sensor_msgs::msg::NavSatFix & msg)
  {
    using sensor_msgs::msg::NavSatStatus;
    if (msg.status.status < NavSatStatus::STATUS_GBAS_FIX)
    {
      ++rejected_fix_;
      return;
    }

    const double var_lat = msg.position_covariance[0];
    const double var_lon = msg.position_covariance[4];
    double pos_acc;
    if (var_lat <= 0.0 || var_lon <= 0.0)
    {
      pos_acc = 10.0;
    }
    else
    {
      pos_acc = std::sqrt((var_lat + var_lon) * 0.5);
    }
    if (pos_acc > max_pos_accuracy_)
    {
      ++rejected_accuracy_;
      return;
    }

    if (!datum_seeded_)
    {
      datum_lat_ = msg.latitude;
      datum_lon_ = msg.longitude;
      cos_datum_lat_ = std::cos(datum_lat_ * kDegToRad);
      datum_seeded_ = true;
      RCLCPP_INFO(get_logger(),
                  "datum self-seeded from first RTK fix: lat=%.8f lon=%.8f",
                  datum_lat_, datum_lon_);
    }

    const double x =
        (msg.longitude - datum_lon_) * cos_datum_lat_ * kMetersPerDeg;
    const double y = (msg.latitude - datum_lat_) * kMetersPerDeg;
    const double t = static_cast<double>(msg.header.stamp.sec) +
                     static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    pos_acc = std::max(pos_acc, 0.002);

    if (!have_prev_)
    {
      prev_t_ = t;
      prev_x_ = x;
      prev_y_ = y;
      prev_pa_ = pos_acc;
      have_prev_ = true;
      return;
    }

    const double dt = t - prev_t_;
    if (dt < min_dt_ || dt > max_dt_)
    {
      if (dt > max_dt_)
      {
        prev_t_ = t;
        prev_x_ = x;
        prev_y_ = y;
        prev_pa_ = pos_acc;
      }
      return;
    }

    const double dx = x - prev_x_;
    const double dy = y - prev_y_;
    const double displacement = std::hypot(dx, dy);
    const double pa0 = prev_pa_;
    prev_t_ = t;
    prev_x_ = x;
    prev_y_ = y;
    prev_pa_ = pos_acc;

    if (displacement < 0.005)
    {
      ++rejected_displacement_;
      return;
    }

    if (std::abs(wheel_vx_) < min_abs_wheel_)
    {
      ++rejected_stationary_;
      return;
    }

    double yaw;
    if (wheel_vx_ >= 0.0)
    {
      yaw = std::atan2(dy, dx);
      ++published_fwd_;
    }
    else
    {
      yaw = std::atan2(-dy, -dx);
      ++published_rev_;
    }

    const double sigma_pos = std::hypot(pos_acc, pa0);
    const double sigma_yaw =
        std::atan2(2.0 * sigma_pos, std::max(displacement, 1e-3));
    const double yaw_var = std::max(
        min_yaw_var_, std::min(sigma_yaw * sigma_yaw, max_yaw_var_));

    publish_imu(now(), yaw, yaw_var);

    const double mono_now =
        static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
    latched_yaw_ = LatchedYaw{mono_now, yaw, yaw_var};

    // Online mag-cal sample collection.
    if (enable_mag_cal_ && latest_mag_valid_ &&
        sigma_yaw < (15.0 * kDegToRad))
    {
      mag_samples_.emplace_back(latest_mag_[0], latest_mag_[1],
                                latest_mag_[2], yaw);
      while (static_cast<int>(mag_samples_.size()) > mag_max_samples_)
      {
        mag_samples_.pop_front();
      }
    }
  }

  void republish_latched_when_stationary()
  {
    if (!latched_yaw_)
    {
      return;
    }
    if (std::abs(wheel_vx_) >= min_abs_wheel_)
    {
      return;
    }
    const double mono_now =
        static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
    const double age = std::max(0.0, mono_now - latched_yaw_->stamp);
    if (age > stationary_max_age_s_)
    {
      latched_yaw_.reset();
      return;
    }
    const double drift_var =
        (stationary_drift_rate_ * age) * (stationary_drift_rate_ * age);
    const double yaw_var =
        std::max(min_yaw_var_,
                 std::min(latched_yaw_->base_var + drift_var, max_yaw_var_));
    publish_imu(now(), latched_yaw_->yaw, yaw_var);
    ++stationary_seeds_published_;
  }

  void publish_imu(const rclcpp::Time & stamp, double yaw, double yaw_var)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "base_footprint";
    imu.orientation.w = std::cos(yaw / 2.0);
    imu.orientation.x = 0.0;
    imu.orientation.y = 0.0;
    imu.orientation.z = std::sin(yaw / 2.0);
    for (auto & v : imu.orientation_covariance) v = 0.0;
    imu.orientation_covariance[8] = yaw_var;
    for (auto & v : imu.angular_velocity_covariance) v = 0.0;
    imu.angular_velocity_covariance[0] = -1.0;
    for (auto & v : imu.linear_acceleration_covariance) v = 0.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    pub_->publish(imu);
  }

  void log_stats()
  {
    RCLCPP_INFO(get_logger(),
                "cog_to_imu stats: published fwd=%d rev=%d seed=%d, "
                "rejected fix=%d accuracy=%d stationary=%d displacement=%d, "
                "mag_cal samples=%zu (writes=%d)",
                published_fwd_, published_rev_, stationary_seeds_published_,
                rejected_fix_, rejected_accuracy_, rejected_stationary_,
                rejected_displacement_, mag_samples_.size(), mag_fit_count_);
    published_fwd_ = 0;
    published_rev_ = 0;
    stationary_seeds_published_ = 0;
    rejected_fix_ = 0;
    rejected_accuracy_ = 0;
    rejected_stationary_ = 0;
    rejected_displacement_ = 0;
  }

  // ── Online magnetometer calibration ────────────────────────────────
  static bool headings_well_distributed(
      const std::deque<std::tuple<double, double, double, double>> & samples)
  {
    int bins[8] = {0};
    for (const auto & s : samples)
    {
      const double psi = std::get<3>(s);
      int idx = static_cast<int>(((psi + M_PI) / (2.0 * M_PI)) * 8.0);
      idx = ((idx % 8) + 8) % 8;
      bins[idx]++;
    }
    int populated = 0;
    for (int b : bins)
      if (b > 0) ++populated;
    return populated >= 3;
  }

  void maybe_refit_mag()
  {
    if (static_cast<int>(mag_samples_.size()) < mag_min_samples_)
    {
      return;
    }
    const double now_s =
        static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
    if (mag_fit_count_ > 0 &&
        (now_s - mag_last_write_t_) < mag_refit_throttle_sec_)
    {
      return;
    }
    if (!headings_well_distributed(mag_samples_))
    {
      RCLCPP_INFO(get_logger(),
                  "online mag fit: %zu samples but heading distribution too "
                  "narrow — waiting for more diverse motion.",
                  mag_samples_.size());
      return;
    }

    // Solve LS with normal equations: M^T M sol = M^T rhs.
    // Unknowns: [ox, oy, A, B] (4×4 symmetric).
    // Top equations:    bx = ox + A·sin ψ + B·cos ψ
    // Bottom equations: by = oy + A·cos ψ - B·sin ψ
    double mtm[4][4] = {{0}};
    double mtr[4] = {0};
    std::vector<double> bzs;
    bzs.reserve(mag_samples_.size());
    const std::size_t n = mag_samples_.size();

    for (const auto & s : mag_samples_)
    {
      const double bx = std::get<0>(s);
      const double by = std::get<1>(s);
      const double bz = std::get<2>(s);
      const double psi = std::get<3>(s);
      bzs.push_back(bz);
      const double sp = std::sin(psi);
      const double cp = std::cos(psi);
      // Top row: rt = [1, 0, sp, cp], target bx
      // Bot row: rb = [0, 1, cp, -sp], target by
      const double rt[4] = {1.0, 0.0, sp, cp};
      const double rb[4] = {0.0, 1.0, cp, -sp};
      for (int i = 0; i < 4; ++i)
      {
        mtr[i] += rt[i] * bx + rb[i] * by;
        for (int j = 0; j < 4; ++j)
        {
          mtm[i][j] += rt[i] * rt[j] + rb[i] * rb[j];
        }
      }
    }

    // Solve 4×4 via Gaussian elimination.
    double a[4][5];
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j) a[i][j] = mtm[i][j];
      a[i][4] = mtr[i];
    }
    for (int i = 0; i < 4; ++i)
    {
      // pivot
      int p = i;
      double pv = std::abs(a[p][i]);
      for (int k = i + 1; k < 4; ++k)
      {
        if (std::abs(a[k][i]) > pv)
        {
          pv = std::abs(a[k][i]);
          p = k;
        }
      }
      if (pv < 1e-12)
      {
        RCLCPP_WARN(get_logger(),
                    "online mag fit: singular normal matrix, skipping.");
        return;
      }
      if (p != i)
      {
        for (int j = 0; j < 5; ++j) std::swap(a[i][j], a[p][j]);
      }
      const double inv = 1.0 / a[i][i];
      for (int j = i; j < 5; ++j) a[i][j] *= inv;
      for (int k = 0; k < 4; ++k)
      {
        if (k == i) continue;
        const double f = a[k][i];
        if (f == 0.0) continue;
        for (int j = i; j < 5; ++j) a[k][j] -= f * a[i][j];
      }
    }
    const double ox = a[0][4];
    const double oy = a[1][4];
    const double A_ = a[2][4];
    const double B_ = a[3][4];

    // Compute residual rms over both halves.
    double sse = 0.0;
    for (const auto & s : mag_samples_)
    {
      const double bx = std::get<0>(s);
      const double by = std::get<1>(s);
      const double psi = std::get<3>(s);
      const double sp = std::sin(psi);
      const double cp = std::cos(psi);
      const double pred_x = ox + A_ * sp + B_ * cp;
      const double pred_y = oy + A_ * cp - B_ * sp;
      sse += (bx - pred_x) * (bx - pred_x);
      sse += (by - pred_y) * (by - pred_y);
    }
    const double rms = std::sqrt(sse / static_cast<double>(2 * n));
    const double bh = std::sqrt(A_ * A_ + B_ * B_);

    if (!(bh >= 10.0 && bh <= 100.0))
    {
      RCLCPP_WARN(get_logger(),
                  "online mag fit rejected: |B_h|=%.1f µT out of range "
                  "(expected 25–60). N=%zu.",
                  bh, n);
      return;
    }
    if (rms > 0.4 * bh)
    {
      RCLCPP_WARN(get_logger(),
                  "online mag fit rejected: rms=%.1f µT > 40%% of |B_h|=%.1f "
                  "µT. N=%zu.",
                  rms, bh, n);
      return;
    }

    // Median of bz.
    std::sort(bzs.begin(), bzs.end());
    const double oz = (bzs.size() % 2 == 1)
                          ? bzs[bzs.size() / 2]
                          : 0.5 * (bzs[bzs.size() / 2 - 1] +
                                   bzs[bzs.size() / 2]);

    try
    {
      write_mag_cal(ox, oy, oz, bh, rms, static_cast<int>(n));
    }
    catch (const std::exception & exc)
    {
      RCLCPP_ERROR(get_logger(), "online mag cal write failed: %s", exc.what());
      return;
    }
    mag_last_write_t_ = now_s;
    ++mag_fit_count_;
    RCLCPP_INFO(get_logger(),
                "online mag cal #%d: offset=(%+7.2f, %+7.2f, %+7.2f) µT  "
                "|B_h|=%.2f µT  rms=%.2f µT  N=%zu  → %s",
                mag_fit_count_, ox, oy, oz, bh, rms, n, mag_cal_path_.c_str());
  }

  void write_mag_cal(double ox, double oy, double oz, double bh, double rms,
                     int n)
  {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "mag_calibration" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "offset_x_uT" << YAML::Value << ox;
    out << YAML::Key << "offset_y_uT" << YAML::Value << oy;
    out << YAML::Key << "offset_z_uT" << YAML::Value << oz;
    out << YAML::Key << "scale_x" << YAML::Value << 1.0;
    out << YAML::Key << "scale_y" << YAML::Value << 1.0;
    out << YAML::Key << "scale_z" << YAML::Value << 1.0;
    out << YAML::Key << "sample_count" << YAML::Value << n;
    out << YAML::Key << "magnitude_mean_uT" << YAML::Value << bh;
    out << YAML::Key << "magnitude_std_uT" << YAML::Value << rms;
    out << YAML::Key << "calibrated_at" << YAML::Value << utc_iso8601_now();
    out << YAML::Key << "source" << YAML::Value << "online_cog_fit";
    out << YAML::Key << "fit_count" << YAML::Value << (mag_fit_count_ + 1);
    out << YAML::EndMap;
    out << YAML::EndMap;

    namespace fs = std::filesystem;
    fs::path target(mag_cal_path_);
    if (target.has_parent_path())
    {
      std::error_code ec;
      fs::create_directories(target.parent_path(), ec);
    }
    const std::string tmp = mag_cal_path_ + ".tmp";
    {
      std::ofstream fh(tmp);
      if (!fh)
      {
        throw std::runtime_error("cannot open " + tmp + " for writing");
      }
      fh << out.c_str();
    }
    std::error_code ec;
    fs::rename(tmp, target, ec);
    if (ec)
    {
      throw std::runtime_error("rename failed: " + ec.message());
    }
  }

  // ── State ─────────────────────────────────────────────────────────
  double min_abs_wheel_{};
  double max_pos_accuracy_{};
  double min_dt_{}, max_dt_{};
  double max_yaw_var_{}, min_yaw_var_{};
  double stationary_seed_rate_hz_{};
  double stationary_drift_rate_{};
  double stationary_max_age_s_{};

  double datum_lat_{}, datum_lon_{};
  bool datum_seeded_{false};
  double cos_datum_lat_{1.0};

  bool enable_mag_cal_{true};
  std::string mag_cal_path_;
  int mag_min_samples_{};
  int mag_max_samples_{};
  double mag_refit_period_sec_{};
  double mag_refit_throttle_sec_{};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_, stationary_timer_, mag_refit_timer_;

  bool have_prev_{false};
  double prev_t_{}, prev_x_{}, prev_y_{}, prev_pa_{};
  double wheel_vx_{0.0};

  struct LatchedYaw
  {
    double stamp;
    double yaw;
    double base_var;
  };
  std::optional<LatchedYaw> latched_yaw_;

  std::array<double, 3> latest_mag_{};
  bool latest_mag_valid_{false};
  std::deque<std::tuple<double, double, double, double>> mag_samples_;
  double mag_last_write_t_{0.0};
  int mag_fit_count_{0};

  int published_fwd_{0}, published_rev_{0};
  int rejected_stationary_{0}, rejected_accuracy_{0}, rejected_fix_{0};
  int rejected_displacement_{0};
  int stationary_seeds_published_{0};
};

}  // namespace mowgli_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::CogToImuNode>());
  rclcpp::shutdown();
  return 0;
}
