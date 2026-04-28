// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/graph_manager.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <ios>
#include <sstream>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/base/serialization.h>
#include <gtsam/base/GenericValue.h>

// boost::serialization needs each polymorphic Value subtype registered
// in a TU before its container can round-trip through XML/binary. The
// only Value type our graph stores is GenericValue<Pose2>; the export
// must use the canonical GUID GTSAM uses internally so .graph files
// produced here can also be read by stock GTSAM tools.
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_GUID(gtsam::GenericValue<gtsam::Pose2>,
                        "gtsam_GenericValue_Pose2")

#include "fusion_graph/factors.hpp"

namespace fusion_graph {

namespace {
constexpr unsigned char kPoseSym = 'x';

inline gtsam::Symbol PoseKey(uint64_t i) {
  return gtsam::Symbol(kPoseSym, i);
}
}  // namespace

GraphManager::GraphManager(const GraphParams& params)
    : params_(params) {
  gtsam::ISAM2Params p;
  // Gauss-Newton is faster than Dogleg here — graph is small (sliding
  // window ~600 nodes at 10 Hz × 60 s) and well-conditioned thanks to
  // the GPS unary prior on most nodes.
  p.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
  p.relinearizeThreshold = 0.05;
  p.relinearizeSkip = 1;
  isam_ = gtsam::ISAM2(p);
}

gtsam::SharedNoiseModel GraphManager::MakeDiagonal(
    const std::vector<double>& sigmas) {
  gtsam::Vector v(sigmas.size());
  for (size_t i = 0; i < sigmas.size(); ++i) v[i] = sigmas[i];
  return gtsam::noiseModel::Diagonal::Sigmas(v);
}

void GraphManager::AddWheelTwist(double vx, double vy, double wz, double dt) {
  if (dt <= 0.0) return;
  std::lock_guard<std::mutex> lock(mu_);
  // Body-frame integration. Yaw is integrated separately because gyro
  // is so much better than wheel-derived yaw on a differential drive.
  // We integrate wheel position assuming the yaw was constant over dt
  // — at 10 Hz nodes and < 0.5 rad/s rotation that's < 5 cm error which
  // the between-factor noise absorbs.
  accum_.dx += vx * dt;
  accum_.dy += vy * dt;
  accum_.dtheta_wheel += wz * dt;
  accum_.dt_total += dt;
}

void GraphManager::AddGyroDelta(double wz, double dt) {
  if (dt <= 0.0) return;
  std::lock_guard<std::mutex> lock(mu_);
  accum_.dtheta_gyro += wz * dt;
}

void GraphManager::QueueGnss(double x, double y, double sigma_xy,
                             bool robust) {
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_xy < params_.gps_sigma_floor)
    sigma_xy = params_.gps_sigma_floor;
  queue_.gnss = UnaryQueue::Gnss{gtsam::Vector2(x, y), sigma_xy, robust};
}

void GraphManager::QueueYaw(double yaw, double sigma_yaw, bool robust) {
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_yaw <= 0.0) sigma_yaw = 0.05;
  queue_.yaw = UnaryQueue::Yaw{yaw, sigma_yaw, robust};
}

void GraphManager::QueueScanBetween(const gtsam::Pose2& delta,
                                    double sigma_xy,
                                    double sigma_theta) {
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_xy <= 0.0) sigma_xy = 0.5;
  if (sigma_theta <= 0.0) sigma_theta = 0.1;
  queue_.scan_between =
      UnaryQueue::ScanBetween{delta, sigma_xy, sigma_theta};
}

void GraphManager::Initialize(const gtsam::Pose2& X0, double timestamp) {
  std::lock_guard<std::mutex> lock(mu_);
  if (initialized_) return;

  auto prior_noise = MakeDiagonal({
      params_.prior_sigma_xy,
      params_.prior_sigma_xy,
      params_.prior_sigma_theta,
  });

  auto k0 = PoseKey(0);
  new_values_.insert(k0, X0);
  new_factors_.add(gtsam::PriorFactor<gtsam::Pose2>(k0, X0, prior_noise));

  isam_.update(new_factors_, new_values_);
  current_estimate_ = isam_.calculateEstimate();
  new_factors_.resize(0);
  new_values_.clear();

  next_index_ = 1;
  last_node_time_s_ = timestamp;
  initialized_ = true;

  TickOutput out;
  out.pose = X0;
  out.covariance = Eigen::Matrix3d::Identity() *
                   (params_.prior_sigma_xy * params_.prior_sigma_xy);
  out.covariance(2, 2) =
      params_.prior_sigma_theta * params_.prior_sigma_theta;
  out.node_index = 0;
  out.timestamp = timestamp;
  latest_ = out;
}

std::optional<TickOutput> GraphManager::Tick(double now_s) {
  std::lock_guard<std::mutex> lock(mu_);
  if (!initialized_) return std::nullopt;
  if (now_s - last_node_time_s_ < params_.node_period_s)
    return std::nullopt;
  return CreateNodeLocked(now_s);
}

TickOutput GraphManager::CreateNodeLocked(double now_s) {
  // 1. Build the wheel between-factor: relative pose from X_{k-1} to X_k.
  //    Pose2(dx, dy, dtheta_gyro) — gyro for yaw, wheel for translation.
  //    If gyro accumulator is zero (no IMU received) fall back to wheel.
  const double dtheta = std::abs(accum_.dtheta_gyro) > 1e-9
                            ? accum_.dtheta_gyro
                            : accum_.dtheta_wheel;
  const gtsam::Pose2 between(accum_.dx, accum_.dy, dtheta);

  const auto k_prev = PoseKey(next_index_ - 1);
  const auto k_curr = PoseKey(next_index_);

  // Predict X_k from current estimate of X_{k-1}, fall back to last
  // known pose if iSAM2 hasn't seen X_{k-1} yet (shouldn't happen).
  gtsam::Pose2 X_prev;
  if (current_estimate_.exists(k_prev)) {
    X_prev = current_estimate_.at<gtsam::Pose2>(k_prev);
  } else {
    X_prev = latest_ ? latest_->pose : gtsam::Pose2();
  }
  const gtsam::Pose2 X_pred = X_prev.compose(between);
  new_values_.insert(k_curr, X_pred);

  // 2. Wheel between-factor.
  // Use gyro_sigma_theta when gyro contributed, wheel sigma otherwise.
  const double sigma_theta = std::abs(accum_.dtheta_gyro) > 1e-9
                                 ? params_.gyro_sigma_theta
                                 : params_.wheel_sigma_theta;
  auto between_noise = MakeDiagonal({
      params_.wheel_sigma_x,
      params_.wheel_sigma_y,
      sigma_theta,
  });
  new_factors_.add(gtsam::BetweenFactor<gtsam::Pose2>(
      k_prev, k_curr, between, between_noise));

  // 3. Queued unary factors. Wrap in Huber when caller flagged the
  // measurement as outlier-prone (RTK-Float / single fix on GPS;
  // magnetometer on yaw).
  if (queue_.gnss) {
    gtsam::SharedNoiseModel noise = MakeDiagonal({
        queue_.gnss->sigma,
        queue_.gnss->sigma,
    });
    if (queue_.gnss->robust) {
      noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(
              params_.huber_k_gps),
          noise);
    }
    new_factors_.add(GnssLeverArmFactor(
        k_curr, queue_.gnss->xy,
        gtsam::Vector2(params_.lever_arm_x, params_.lever_arm_y),
        noise));
  }
  if (queue_.yaw) {
    gtsam::SharedNoiseModel noise = MakeDiagonal({queue_.yaw->sigma});
    if (queue_.yaw->robust) {
      noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(
              params_.huber_k_yaw),
          noise);
    }
    new_factors_.add(YawUnaryFactor(
        k_curr, queue_.yaw->yaw, noise));
  }
  if (queue_.scan_between) {
    auto noise = MakeDiagonal({
        queue_.scan_between->sigma_xy,
        queue_.scan_between->sigma_xy,
        queue_.scan_between->sigma_theta,
    });
    new_factors_.add(gtsam::BetweenFactor<gtsam::Pose2>(
        k_prev, k_curr, queue_.scan_between->delta, noise));
  }

  // 4. iSAM2 update.
  isam_.update(new_factors_, new_values_);
  current_estimate_ = isam_.calculateEstimate();
  new_factors_.resize(0);
  new_values_.clear();

  // 5. Marginal covariance — best-effort. iSAM2 marginalCovariance can
  //    throw if the variable is poorly constrained; fall back to a
  //    conservative diagonal in that case.
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1.0;
  try {
    cov = isam_.marginalCovariance(k_curr);
  } catch (const std::exception&) {
    // leave conservative default
  }

  TickOutput out;
  out.pose = current_estimate_.at<gtsam::Pose2>(k_curr);
  out.covariance = cov;
  out.node_index = next_index_;
  out.timestamp = now_s;
  latest_ = out;

  // 6. Reset for next tick.
  ++next_index_;
  last_node_time_s_ = now_s;
  accum_.Reset();
  queue_.gnss.reset();
  queue_.yaw.reset();
  queue_.scan_between.reset();

  return out;
}

std::optional<TickOutput> GraphManager::LatestSnapshot() const {
  std::lock_guard<std::mutex> lock(mu_);
  return latest_;
}

GraphStats GraphManager::Stats() const {
  std::lock_guard<std::mutex> lock(mu_);
  GraphStats s;
  s.total_nodes = next_index_;
  s.scans_attached = scans_.size();
  s.loop_closures = loop_closures_added_;
  return s;
}

// ─────────────────────────────────────────────────────────────────────
// Scan storage + loop closure
// ─────────────────────────────────────────────────────────────────────

void GraphManager::AttachScan(uint64_t node_index,
                              const std::vector<Eigen::Vector2d>& scan) {
  std::lock_guard<std::mutex> lock(mu_);
  scans_[node_index] = scan;
}

std::vector<Eigen::Vector2d> GraphManager::GetScan(
    uint64_t node_index) const {
  std::lock_guard<std::mutex> lock(mu_);
  auto it = scans_.find(node_index);
  if (it == scans_.end()) return {};
  return it->second;
}

std::optional<gtsam::Pose2> GraphManager::GetPose(
    uint64_t node_index) const {
  std::lock_guard<std::mutex> lock(mu_);
  auto k = PoseKey(node_index);
  if (!current_estimate_.exists(k)) return std::nullopt;
  return current_estimate_.at<gtsam::Pose2>(k);
}

std::vector<uint64_t> GraphManager::FindLoopClosureCandidates(
    uint64_t query_index,
    double max_dist_m,
    double min_age_s,
    size_t max_candidates) const {
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<uint64_t> out;
  if (next_index_ == 0) return out;

  auto kq = PoseKey(query_index);
  if (!current_estimate_.exists(kq)) return out;
  const auto Xq = current_estimate_.at<gtsam::Pose2>(kq);

  // Per-node age proxy: nodes are created at node_period_s cadence,
  // so age_idx = (next - i) * node_period_s. Within ±10% of wall
  // clock, sufficient for the >30s gate.
  const double age_per_idx = params_.node_period_s;
  const auto cutoff_idx_diff =
      static_cast<uint64_t>(std::ceil(min_age_s / age_per_idx));

  // Linear scan over scans_ keys (== nodes with a stored scan, which
  // is what we want — no point loop-closing to a node without a
  // scan). For < 1000 nodes / session, brute-force is sub-ms.
  std::vector<std::pair<double, uint64_t>> hits;
  hits.reserve(scans_.size());
  const double max_d2 = max_dist_m * max_dist_m;
  for (const auto& [idx, _] : scans_) {
    if (idx == query_index) continue;
    if (query_index - idx < cutoff_idx_diff) continue;
    auto k = PoseKey(idx);
    if (!current_estimate_.exists(k)) continue;
    const auto X = current_estimate_.at<gtsam::Pose2>(k);
    const double dx = X.x() - Xq.x();
    const double dy = X.y() - Xq.y();
    const double d2 = dx * dx + dy * dy;
    if (d2 <= max_d2) hits.emplace_back(d2, idx);
  }

  std::sort(hits.begin(), hits.end());
  for (size_t i = 0; i < hits.size() && out.size() < max_candidates; ++i)
    out.push_back(hits[i].second);
  return out;
}

void GraphManager::AddLoopClosure(uint64_t prev_index,
                                  uint64_t curr_index,
                                  const gtsam::Pose2& delta,
                                  double sigma_xy,
                                  double sigma_theta) {
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_xy <= 0.0) sigma_xy = 0.5;
  if (sigma_theta <= 0.0) sigma_theta = 0.1;

  auto k_prev = PoseKey(prev_index);
  auto k_curr = PoseKey(curr_index);
  if (!current_estimate_.exists(k_prev) ||
      !current_estimate_.exists(k_curr))
    return;

  auto noise = MakeDiagonal({sigma_xy, sigma_xy, sigma_theta});
  gtsam::NonlinearFactorGraph fg;
  fg.add(gtsam::BetweenFactor<gtsam::Pose2>(
      k_prev, k_curr, delta, noise));

  isam_.update(fg, gtsam::Values());
  current_estimate_ = isam_.calculateEstimate();
  ++loop_closures_added_;
  loop_closure_edges_.emplace_back(prev_index, curr_index);
}

std::map<uint64_t, gtsam::Pose2> GraphManager::GetAllPoses() const {
  std::lock_guard<std::mutex> lock(mu_);
  std::map<uint64_t, gtsam::Pose2> out;
  // ISAM2 indexes Pose2 nodes by Symbol('x', idx). Iterate the
  // current estimate and pull each one out by its key index.
  for (const auto& kv : current_estimate_) {
    gtsam::Symbol s(kv.key);
    if (s.chr() != 'x') continue;
    out.emplace(static_cast<uint64_t>(s.index()),
                kv.value.cast<gtsam::Pose2>());
  }
  return out;
}

std::vector<std::pair<uint64_t, uint64_t>>
GraphManager::GetLoopClosureEdges() const {
  std::lock_guard<std::mutex> lock(mu_);
  return loop_closure_edges_;
}

// ─────────────────────────────────────────────────────────────────────
// Persistence
// ─────────────────────────────────────────────────────────────────────
//
// We serialize the iSAM2 *result* — current_estimate_ + a summarized
// factor graph — rather than ISAM2 itself (whose internal Bayes-tree
// state is GTSAM-version-sensitive). A fresh boot rebuilds the Bayes
// tree by replaying a single PriorFactor on each node and re-adding
// the between-factors as we observe new ones.
//
// The on-disk format is a 3-tuple of files: .graph (XML, gtsam
// archive), .scans (binary, our own format), .meta (text key=value).

namespace {

void SerializeScansBinary(
    const std::map<uint64_t, std::vector<Eigen::Vector2d>>& scans,
    std::ostream& os) {
  uint64_t n = scans.size();
  os.write(reinterpret_cast<const char*>(&n), sizeof(n));
  for (const auto& [idx, pts] : scans) {
    os.write(reinterpret_cast<const char*>(&idx), sizeof(idx));
    uint64_t m = pts.size();
    os.write(reinterpret_cast<const char*>(&m), sizeof(m));
    for (const auto& p : pts) {
      double xy[2] = {p.x(), p.y()};
      os.write(reinterpret_cast<const char*>(xy), sizeof(xy));
    }
  }
}

bool DeserializeScansBinary(
    std::istream& is,
    std::map<uint64_t, std::vector<Eigen::Vector2d>>& scans) {
  uint64_t n = 0;
  if (!is.read(reinterpret_cast<char*>(&n), sizeof(n))) return false;
  for (uint64_t i = 0; i < n; ++i) {
    uint64_t idx = 0, m = 0;
    if (!is.read(reinterpret_cast<char*>(&idx), sizeof(idx))) return false;
    if (!is.read(reinterpret_cast<char*>(&m), sizeof(m))) return false;
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(m);
    for (uint64_t j = 0; j < m; ++j) {
      double xy[2];
      if (!is.read(reinterpret_cast<char*>(xy), sizeof(xy))) return false;
      pts.emplace_back(xy[0], xy[1]);
    }
    scans.emplace(idx, std::move(pts));
  }
  return true;
}

}  // namespace

bool GraphManager::Save(const std::string& prefix) const {
  std::lock_guard<std::mutex> lock(mu_);
  try {
    std::ofstream graph_os(prefix + ".graph");
    if (!graph_os) return false;
    graph_os << gtsam::serializeXML(current_estimate_);
    graph_os.close();

    std::ofstream scans_os(prefix + ".scans", std::ios::binary);
    if (!scans_os) return false;
    SerializeScansBinary(scans_, scans_os);
    scans_os.close();

    std::ofstream meta_os(prefix + ".meta");
    if (!meta_os) return false;
    meta_os << "next_index=" << next_index_ << "\n";
    meta_os << "last_node_time_s=" << last_node_time_s_ << "\n";
    meta_os.close();
  } catch (const std::exception& e) {
    fprintf(stderr, "fusion_graph::Save: %s\n", e.what());
    return false;
  }
  return true;
}

bool GraphManager::Load(const std::string& prefix) {
  std::lock_guard<std::mutex> lock(mu_);
  if (initialized_) return false;

  gtsam::Values loaded_values;
  try {
    std::ifstream graph_is(prefix + ".graph");
    if (!graph_is) return false;
    std::stringstream buf;
    buf << graph_is.rdbuf();
    gtsam::deserializeXML(buf.str(), loaded_values);
  } catch (const std::exception&) {
    return false;
  }

  std::map<uint64_t, std::vector<Eigen::Vector2d>> loaded_scans;
  try {
    std::ifstream scans_is(prefix + ".scans", std::ios::binary);
    if (!scans_is) return false;
    if (!DeserializeScansBinary(scans_is, loaded_scans)) return false;
  } catch (const std::exception&) {
    return false;
  }

  uint64_t next_idx = 0;
  double last_t = 0.0;
  try {
    std::ifstream meta_is(prefix + ".meta");
    if (!meta_is) return false;
    std::string line;
    while (std::getline(meta_is, line)) {
      auto eq = line.find('=');
      if (eq == std::string::npos) continue;
      const std::string key = line.substr(0, eq);
      const std::string val = line.substr(eq + 1);
      if (key == "next_index") next_idx = std::stoull(val);
      else if (key == "last_node_time_s") last_t = std::stod(val);
    }
  } catch (const std::exception&) {
    return false;
  }

  // Re-seed iSAM2 with each loaded pose pinned by a tight prior; the
  // priors keep optimization stable as new wheel/GPS factors arrive.
  // The covariances are looser than the live priors so loop-closures
  // can still re-balance the loaded portion if it was inconsistent.
  auto pin_noise = MakeDiagonal({0.01, 0.01, 0.01});
  gtsam::NonlinearFactorGraph fg;
  for (const auto& key_value : loaded_values) {
    fg.add(gtsam::PriorFactor<gtsam::Pose2>(
        key_value.key,
        key_value.value.cast<gtsam::Pose2>(),
        pin_noise));
  }
  isam_.update(fg, loaded_values);
  current_estimate_ = isam_.calculateEstimate();

  scans_ = std::move(loaded_scans);
  next_index_ = next_idx;
  last_node_time_s_ = last_t;
  initialized_ = true;

  return true;
}

}  // namespace fusion_graph
