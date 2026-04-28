// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/fusion_graph_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fusion_graph {

namespace {

constexpr double kEarthRadius = 6378137.0;  // WGS84 equatorial, metres.

// Extract yaw from a geometry_msgs Quaternion.
double YawFromQuat(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3(tq).getRPY(r, p, y);
  return y;
}

geometry_msgs::msg::Quaternion QuatFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion m;
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  m.w = q.w();
  return m;
}

}  // namespace

FusionGraphNode::FusionGraphNode(const rclcpp::NodeOptions& opts)
    : rclcpp::Node("fusion_graph_node", opts) {
  // ── Parameters ────────────────────────────────────────────────────
  GraphParams gp;
  gp.node_period_s = declare_parameter<double>("node_period_s", 0.1);
  gp.wheel_sigma_x = declare_parameter<double>("wheel_sigma_x", 0.05);
  gp.wheel_sigma_y = declare_parameter<double>("wheel_sigma_y", 0.005);
  gp.wheel_sigma_theta =
      declare_parameter<double>("wheel_sigma_theta", 0.01);
  gp.gyro_sigma_theta = declare_parameter<double>("gyro_sigma_theta", 0.005);
  gp.gps_sigma_floor = declare_parameter<double>("gps_sigma_floor", 0.003);
  gp.prior_sigma_xy = declare_parameter<double>("prior_sigma_xy", 0.05);
  gp.prior_sigma_theta =
      declare_parameter<double>("prior_sigma_theta", 0.05);
  gp.lever_arm_x = declare_parameter<double>("lever_arm_x", 0.0);
  gp.lever_arm_y = declare_parameter<double>("lever_arm_y", 0.0);

  datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
  datum_lon_ = declare_parameter<double>("datum_lon", 0.0);
  datum_cos_lat_ = std::cos(datum_lat_ * M_PI / 180.0);

  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ =
      declare_parameter<std::string>("base_frame", "base_footprint");

  graph_ = std::make_unique<GraphManager>(gp);

  // ── Scan matching (optional) ─────────────────────────────────────
  use_scan_matching_ =
      declare_parameter<bool>("use_scan_matching", false);
  if (use_scan_matching_) {
    ScanMatcherParams sp;
    sp.max_iterations = declare_parameter<int>("icp_max_iter", 15);
    sp.max_correspondence_dist =
        declare_parameter<double>("icp_max_corresp_dist", 0.5);
    sp.source_subsample = static_cast<size_t>(
        declare_parameter<int>("icp_source_subsample", 60));
    sp.sigma_xy_base =
        declare_parameter<double>("icp_sigma_xy_base", 0.02);
    sp.sigma_theta_base =
        declare_parameter<double>("icp_sigma_theta_base", 0.005);
    scan_matcher_ = std::make_unique<ScanMatcher>(sp);
  }

  // ── Magnetometer (off by default) ───────────────────────────────
  // Motors near the chassis induce a heading-dependent bias on the
  // magnetometer that no static cal can remove (see CLAUDE.md
  // history). Default off so the graph never sees mag samples;
  // operators with a motor-isolated mag hardware setup can flip the
  // flag on at launch.
  use_magnetometer_ =
      declare_parameter<bool>("use_magnetometer", false);

  // ── Loop closure + persistence ───────────────────────────────────
  loop_closure_enabled_ =
      declare_parameter<bool>("use_loop_closure", false);
  lc_max_dist_m_ =
      declare_parameter<double>("lc_max_dist_m", 5.0);
  lc_min_age_s_ =
      declare_parameter<double>("lc_min_age_s", 30.0);
  lc_max_candidates_ = static_cast<size_t>(
      declare_parameter<int>("lc_max_candidates", 3));
  lc_max_rmse_ =
      declare_parameter<double>("lc_max_rmse", 0.10);
  lc_sigma_xy_ =
      declare_parameter<double>("lc_sigma_xy", 0.05);
  lc_sigma_theta_ =
      declare_parameter<double>("lc_sigma_theta", 0.02);

  graph_save_prefix_ = declare_parameter<std::string>(
      "graph_save_prefix", "/ros2_ws/maps/fusion_graph");
  const bool autoload =
      declare_parameter<bool>("autoload_graph", true);

  // Dock yaw read from mowgli_robot.yaml via the launch wrapper. Used
  // as a yaw seed at cold boot when GPS+COG aren't available yet but
  // a persisted graph exists — the robot is on the dock so this is
  // a tight prior.
  dock_pose_yaw_ = declare_parameter<double>("dock_pose_yaw", 0.0);

  if (autoload) {
    if (graph_->Load(graph_save_prefix_)) {
      autoload_succeeded_ = true;
      RCLCPP_INFO(get_logger(),
                  "fusion_graph: loaded persisted graph from '%s.*'",
                  graph_save_prefix_.c_str());
    }
  }

  // ── TF ────────────────────────────────────────────────────────────
  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Pubs/subs ─────────────────────────────────────────────────────
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odometry/filtered_map", 10);
  pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/fusion_graph/diagnostics", 10);
  pub_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
          "/fusion_graph/markers", rclcpp::QoS(1).transient_local());

  auto sensor_qos = rclcpp::SensorDataQoS();

  sub_wheel_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom", 50,
      std::bind(&FusionGraphNode::OnWheelOdom, this,
                std::placeholders::_1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos,
      std::bind(&FusionGraphNode::OnImu, this, std::placeholders::_1));

  sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", sensor_qos,
      std::bind(&FusionGraphNode::OnGnss, this, std::placeholders::_1));

  // /imu/cog_heading and /imu/mag_yaw are published BEST_EFFORT by
  // cog_to_imu.py and mag_yaw_publisher.py — use SensorDataQoS or
  // the subscription is silently dropped at the QoS handshake.
  sub_cog_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/cog_heading", sensor_qos,
      std::bind(&FusionGraphNode::OnCogHeading, this,
                std::placeholders::_1));

  if (use_magnetometer_) {
    sub_mag_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/mag_yaw", sensor_qos,
        std::bind(&FusionGraphNode::OnMagYaw, this,
                  std::placeholders::_1));
  }

  if (use_scan_matching_ || loop_closure_enabled_) {
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", sensor_qos,
        std::bind(&FusionGraphNode::OnScan, this,
                  std::placeholders::_1));
  }

  // ── Save-graph service ──────────────────────────────────────────
  // Trigger from the GUI / a BT node when transitioning out of
  // RECORDING, or manually via:
  //   ros2 service call /fusion_graph_node/save_graph std_srvs/Trigger
  srv_save_ = create_service<std_srvs::srv::Trigger>(
      "~/save_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        const bool ok = graph_->Save(graph_save_prefix_);
        resp->success = ok;
        resp->message = ok ? "saved to " + graph_save_prefix_ + ".*"
                           : "save failed";
        if (ok)
          RCLCPP_INFO(get_logger(), "fusion_graph: %s",
                      resp->message.c_str());
        else
          RCLCPP_WARN(get_logger(), "fusion_graph: %s",
                      resp->message.c_str());
      });

  // ── Tick timer ────────────────────────────────────────────────────
  // Run at 2× node rate so we never miss a node window.
  const double timer_period_s = gp.node_period_s * 0.5;
  tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_s),
      std::bind(&FusionGraphNode::OnTimer, this));

  // Diagnostics timer at 1 Hz — coarse, just for the session monitor.
  diag_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {
        auto stats = graph_->Stats();
        auto snap = graph_->LatestSnapshot();

        diagnostic_msgs::msg::DiagnosticArray msg;
        msg.header.stamp = this->now();
        diagnostic_msgs::msg::DiagnosticStatus s;
        s.name = "fusion_graph";
        s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        s.message = graph_->IsInitialized() ? "running" : "waiting init";

        auto add = [&s](const std::string& k, const std::string& v) {
          diagnostic_msgs::msg::KeyValue kv;
          kv.key = k;
          kv.value = v;
          s.values.push_back(kv);
        };
        add("total_nodes", std::to_string(stats.total_nodes));
        add("scans_attached", std::to_string(stats.scans_attached));
        add("loop_closures", std::to_string(stats.loop_closures));
        add("scans_received", std::to_string(scans_received_));
        add("scan_matches_ok", std::to_string(scan_matches_ok_));
        add("scan_matches_fail", std::to_string(scan_matches_fail_));
        if (snap) {
          char buf[64];
          std::snprintf(buf, sizeof(buf), "%.4f", snap->covariance(0, 0));
          add("cov_xx", buf);
          std::snprintf(buf, sizeof(buf), "%.4f", snap->covariance(1, 1));
          add("cov_yy", buf);
          std::snprintf(buf, sizeof(buf), "%.4f", snap->covariance(2, 2));
          add("cov_yawyaw", buf);
        }
        msg.status.push_back(s);
        pub_diag_->publish(msg);

        // ── Pose-graph viz ────────────────────────────────────────
        // Emits a single MarkerArray with three markers, each owning
        // its own id so subsequent publishes overwrite cleanly:
        //   id=0  SPHERE_LIST  — every node's optimized xy
        //   id=1  LINE_STRIP   — trajectory through nodes by index
        //   id=2  LINE_LIST    — accepted loop-closure edges
        // All in map_frame_; transient-local QoS so a Foxglove client
        // joining mid-session sees the whole graph immediately.
        const auto poses = graph_->GetAllPoses();
        const auto loops = graph_->GetLoopClosureEdges();
        const rclcpp::Time stamp = this->now();

        visualization_msgs::msg::MarkerArray ma;

        visualization_msgs::msg::Marker nodes;
        nodes.header.stamp = stamp;
        nodes.header.frame_id = map_frame_;
        nodes.ns = "fusion_graph";
        nodes.id = 0;
        nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes.action = visualization_msgs::msg::Marker::ADD;
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.10;
        nodes.color.r = 0.1f; nodes.color.g = 0.7f;
        nodes.color.b = 1.0f; nodes.color.a = 1.0f;
        nodes.pose.orientation.w = 1.0;

        visualization_msgs::msg::Marker traj;
        traj.header = nodes.header;
        traj.ns = "fusion_graph";
        traj.id = 1;
        traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
        traj.action = visualization_msgs::msg::Marker::ADD;
        traj.scale.x = 0.03;
        traj.color.r = 0.5f; traj.color.g = 0.5f;
        traj.color.b = 0.5f; traj.color.a = 0.8f;
        traj.pose.orientation.w = 1.0;

        for (const auto& [idx, p] : poses) {
          geometry_msgs::msg::Point pt;
          pt.x = p.x(); pt.y = p.y(); pt.z = 0.0;
          nodes.points.push_back(pt);
          traj.points.push_back(pt);
        }
        ma.markers.push_back(nodes);
        ma.markers.push_back(traj);

        visualization_msgs::msg::Marker lc;
        lc.header = nodes.header;
        lc.ns = "fusion_graph";
        lc.id = 2;
        lc.type = visualization_msgs::msg::Marker::LINE_LIST;
        lc.action = visualization_msgs::msg::Marker::ADD;
        lc.scale.x = 0.04;
        lc.color.r = 1.0f; lc.color.g = 0.2f;
        lc.color.b = 0.2f; lc.color.a = 0.9f;
        lc.pose.orientation.w = 1.0;
        for (const auto& [a, b] : loops) {
          auto ia = poses.find(a);
          auto ib = poses.find(b);
          if (ia == poses.end() || ib == poses.end()) continue;
          geometry_msgs::msg::Point pa, pb;
          pa.x = ia->second.x(); pa.y = ia->second.y();
          pb.x = ib->second.x(); pb.y = ib->second.y();
          lc.points.push_back(pa);
          lc.points.push_back(pb);
        }
        ma.markers.push_back(lc);

        pub_markers_->publish(ma);
      });

  RCLCPP_INFO(get_logger(),
              "fusion_graph_node up: datum=(%.6f, %.6f), node_period=%.3fs",
              datum_lat_, datum_lon_, gp.node_period_s);
}

// ── Callbacks ─────────────────────────────────────────────────────────

void FusionGraphNode::OnWheelOdom(
    nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  const rclcpp::Time stamp(msg->header.stamp);
  if (last_wheel_stamp_) {
    double dt = (stamp - *last_wheel_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0) {
      graph_->AddWheelTwist(msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.angular.z, dt);
    }
  }
  last_wheel_stamp_ = stamp;
}

void FusionGraphNode::OnImu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  const rclcpp::Time stamp(msg->header.stamp);
  if (last_imu_stamp_) {
    double dt = (stamp - *last_imu_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0) {
      graph_->AddGyroDelta(msg->angular_velocity.z, dt);
    }
  }
  last_imu_stamp_ = stamp;
}

void FusionGraphNode::OnGnss(
    sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
    return;
  if (datum_lat_ == 0.0 && datum_lon_ == 0.0) {
    // Self-seed datum from first valid fix. Not ideal — operator should
    // set datum in mowgli_robot.yaml — but keeps the node from refusing
    // to start during sim/dev.
    datum_lat_ = msg->latitude;
    datum_lon_ = msg->longitude;
    datum_cos_lat_ = std::cos(datum_lat_ * M_PI / 180.0);
    RCLCPP_WARN(get_logger(),
                "fusion_graph: datum self-seeded from first fix "
                "(%.6f, %.6f) — set datum_lat/lon explicitly",
                datum_lat_, datum_lon_);
  }

  double mx, my;
  LatLonToMap(msg->latitude, msg->longitude, mx, my);

  // covariance[0] is variance of east; take sqrt for sigma. Use the
  // diagonal mean for a single sigma_xy (factor model is isotropic).
  const double var_x = msg->position_covariance[0];
  const double var_y = msg->position_covariance[4];
  double sigma = std::sqrt(0.5 * (var_x + var_y));
  if (!std::isfinite(sigma) || sigma <= 0.0) sigma = -1.0;  // floor

  // RTK-Fixed (GBAS_FIX = 2) is sub-cm and statistically Gaussian; any
  // lower fix quality (Float / single / DGPS) routinely carries
  // multi-decimetre multipath outliers that the reported covariance
  // doesn't capture. Robustify with Huber in that case so the optimizer
  // downweights aberrant samples instead of pulling the trajectory.
  const bool robust =
      msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  graph_->QueueGnss(mx, my, sigma, robust);
  seed_xy_ = gtsam::Vector2(mx, my);

  TrySeedInitialPose();
}

void FusionGraphNode::OnCogHeading(
    sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  const double yaw = YawFromQuat(msg->orientation);
  // covariance[8] is yaw variance.
  double var = msg->orientation_covariance[8];
  if (!std::isfinite(var) || var <= 0.0) var = 0.05 * 0.05;
  const double sigma = std::sqrt(var);
  graph_->QueueYaw(yaw, sigma);
  seed_yaw_ = yaw;
  TrySeedInitialPose();
}

void FusionGraphNode::OnMagYaw(
    sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  const double yaw = YawFromQuat(msg->orientation);
  double var = msg->orientation_covariance[8];
  if (!std::isfinite(var) || var <= 0.0) var = 0.1 * 0.1;
  // Mag yaw carries heading-dependent calibration bias (~5-15° peaks)
  // even after tilt compensation. Always robustify so when COG is also
  // active the optimizer pulls toward COG and treats mag as a soft
  // anchor that prevents free drift, not as a precise observation.
  graph_->QueueYaw(yaw, std::sqrt(var), /*robust=*/true);
  if (!seed_yaw_) seed_yaw_ = yaw;
  TrySeedInitialPose();
}

void FusionGraphNode::OnScan(
    sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  // Resolve scan_frame -> base_footprint at the scan timestamp; if TF
  // isn't ready yet, drop this scan rather than warp it with stale
  // extrinsics.
  geometry_msgs::msg::TransformStamped t_base_scan;
  try {
    t_base_scan = tf_buffer_->lookupTransform(
        base_frame_, msg->header.frame_id,
        msg->header.stamp, tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException&) {
    return;
  }

  tf2::Transform T_base_scan;
  tf2::fromMsg(t_base_scan.transform, T_base_scan);

  std::vector<Eigen::Vector2d> pts;
  pts.reserve(msg->ranges.size());
  const double a0 = msg->angle_min;
  const double da = msg->angle_increment;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
      continue;
    const double a = a0 + da * static_cast<double>(i);
    tf2::Vector3 p_scan(r * std::cos(a), r * std::sin(a), 0.0);
    tf2::Vector3 p_base = T_base_scan * p_scan;
    pts.emplace_back(p_base.x(), p_base.y());
  }

  std::lock_guard<std::mutex> lock(scan_mu_);
  latest_scan_ = std::move(pts);
  latest_scan_valid_ = !latest_scan_.empty();
  ++scans_received_;

  // Cold-boot relocalization: if we autoloaded a graph but never had
  // a fresh GPS+COG to validate the live pose, ICP-match this first
  // scan against scans of nodes near the dock and force-anchor the
  // last loaded node at the matched pose. This unsticks the case
  // "GPS dead since boot, robot was placed back on dock manually
  // between sessions".
  if (autoload_succeeded_ && !relocalize_done_ && scan_matcher_ &&
      latest_scan_valid_ && graph_->IsInitialized()) {
    const auto candidates = graph_->FindNodesNearXY(
        0.0, 0.0, 5.0, 5);  // dock is map origin (datum)
    double best_rmse = std::numeric_limits<double>::infinity();
    gtsam::Pose2 best_pose;
    uint64_t best_idx = 0;
    for (uint64_t idx : candidates) {
      auto cand_scan = graph_->GetScan(idx);
      auto cand_pose = graph_->GetPose(idx);
      if (cand_scan.empty() || !cand_pose) continue;
      // Use the candidate's pose as init (we expect to be close).
      auto res = scan_matcher_->Match(
          cand_scan, latest_scan_, *cand_pose);
      if (res.ok && res.rmse < best_rmse) {
        best_rmse = res.rmse;
        best_pose = res.delta;
        best_idx = idx;
      }
    }
    if (std::isfinite(best_rmse) && best_rmse < 0.10) {
      // Anchor the latest loaded node at the matched pose so future
      // wheel/scan factors compose from a consistent reference.
      auto snap = graph_->LatestSnapshot();
      if (snap) {
        graph_->ForceAnchor(snap->node_index, best_pose, 0.05, 0.05);
        relocalize_done_ = true;
        RCLCPP_INFO(get_logger(),
                    "fusion_graph: relocalized via scan match "
                    "node=%lu rmse=%.3f → (%.2f, %.2f, %.2f rad)",
                    static_cast<unsigned long>(best_idx), best_rmse,
                    best_pose.x(), best_pose.y(), best_pose.theta());
      }
    }
  }
}

void FusionGraphNode::OnTimer() {
  const double now_s = this->now().seconds();

  // Run scan-matching against the previous-node scan and queue the
  // resulting between-factor before Tick — Tick consumes the queue
  // when it creates a new node.
  std::vector<Eigen::Vector2d> curr_scan;
  bool curr_valid = false;
  {
    std::lock_guard<std::mutex> lock(scan_mu_);
    if (latest_scan_valid_) {
      curr_scan = latest_scan_;
      curr_valid = true;
    }
  }

  if (use_scan_matching_ && scan_matcher_ && curr_valid &&
      prev_node_scan_valid_) {
    auto res = scan_matcher_->Match(prev_node_scan_, curr_scan,
                                    gtsam::Pose2());
    if (res.ok) {
      graph_->QueueScanBetween(res.delta, res.sigma_xy, res.sigma_theta);
      ++scan_matches_ok_;
    } else {
      ++scan_matches_fail_;
    }
  }

  auto out = graph_->Tick(now_s);
  if (out) {
    PublishOutputs(*out);

    // Attach the current scan to the new node (used for loop closures
    // + persistence). Use the still-valid current_scan we captured
    // above; reusing it as prev_node_scan is OK since std::move only
    // happens after this block.
    if (curr_valid) {
      graph_->AttachScan(out->node_index, curr_scan);
    }

    // Loop closure search — gated on loop_closure_enabled_ and on
    // having a scan matcher (we reuse it). Find candidates within
    // lc_max_dist_m_ that are at least lc_min_age_s_ old, run ICP for
    // each, accept those with rmse < lc_max_rmse_.
    if (loop_closure_enabled_ && scan_matcher_ && curr_valid) {
      auto candidates = graph_->FindLoopClosureCandidates(
          out->node_index, lc_max_dist_m_, lc_min_age_s_,
          lc_max_candidates_);
      for (uint64_t cand_idx : candidates) {
        auto cand_scan = graph_->GetScan(cand_idx);
        auto cand_pose = graph_->GetPose(cand_idx);
        if (cand_scan.empty() || !cand_pose) continue;

        // Init guess: transform from cand to current in map frame,
        // i.e. cand.between(curr).
        const gtsam::Pose2 init = cand_pose->between(out->pose);
        auto res = scan_matcher_->Match(cand_scan, curr_scan, init);
        if (!res.ok || res.rmse > lc_max_rmse_) continue;

        graph_->AddLoopClosure(cand_idx, out->node_index, res.delta,
                               lc_sigma_xy_, lc_sigma_theta_);
        RCLCPP_INFO(get_logger(),
                    "fusion_graph: loop closure %lu -> %lu accepted "
                    "(rmse=%.3f, dist=%.2f m)",
                    static_cast<unsigned long>(cand_idx),
                    static_cast<unsigned long>(out->node_index),
                    res.rmse,
                    std::hypot(out->pose.x() - cand_pose->x(),
                               out->pose.y() - cand_pose->y()));
      }
    }

    if (curr_valid) {
      prev_node_scan_ = std::move(curr_scan);
      prev_node_scan_valid_ = true;
    }
  }
}

// ── Helpers ───────────────────────────────────────────────────────────

void FusionGraphNode::LatLonToMap(double lat, double lon, double& x,
                                  double& y) const {
  const double dlat = (lat - datum_lat_) * M_PI / 180.0;
  const double dlon = (lon - datum_lon_) * M_PI / 180.0;
  x = kEarthRadius * datum_cos_lat_ * dlon;  // east
  y = kEarthRadius * dlat;                   // north
}

bool FusionGraphNode::TrySeedInitialPose() {
  if (graph_->IsInitialized()) return true;
  if (!seed_xy_ || !seed_yaw_) return false;
  graph_->Initialize(
      gtsam::Pose2(seed_xy_->x(), seed_xy_->y(), *seed_yaw_),
      this->now().seconds());
  RCLCPP_INFO(get_logger(),
              "fusion_graph: initialized at (%.3f, %.3f, %.3f rad)",
              seed_xy_->x(), seed_xy_->y(), *seed_yaw_);
  return true;
}

void FusionGraphNode::PublishOutputs(const TickOutput& out) {
  // 1. nav_msgs/Odometry on /odometry/filtered_map.
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->now();
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = out.pose.x();
  odom.pose.pose.position.y = out.pose.y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = QuatFromYaw(out.pose.theta());

  // Pose covariance is 6x6 row-major: x, y, z, roll, pitch, yaw.
  for (auto& v : odom.pose.covariance) v = 0.0;
  odom.pose.covariance[0]  = out.covariance(0, 0);
  odom.pose.covariance[1]  = out.covariance(0, 1);
  odom.pose.covariance[5]  = out.covariance(0, 2);
  odom.pose.covariance[6]  = out.covariance(1, 0);
  odom.pose.covariance[7]  = out.covariance(1, 1);
  odom.pose.covariance[11] = out.covariance(1, 2);
  odom.pose.covariance[30] = out.covariance(2, 0);
  odom.pose.covariance[31] = out.covariance(2, 1);
  odom.pose.covariance[35] = out.covariance(2, 2);
  // Z, roll, pitch — clamped, give them tiny variance so consumers
  // don't choke on zero.
  odom.pose.covariance[14] = 1e-9;
  odom.pose.covariance[21] = 1e-9;
  odom.pose.covariance[28] = 1e-9;
  pub_odom_->publish(odom);

  // 2. TF map -> odom. We need: T_map_base · (T_odom_base)^-1.
  //    T_odom_base is the local EKF's odom->base_footprint TF.
  geometry_msgs::msg::TransformStamped t_odom_base;
  try {
    t_odom_base = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "fusion_graph: TF %s -> %s not available: %s",
                         odom_frame_.c_str(), base_frame_.c_str(),
                         ex.what());
    return;
  }

  tf2::Transform T_odom_base;
  tf2::fromMsg(t_odom_base.transform, T_odom_base);

  tf2::Transform T_map_base;
  T_map_base.setOrigin(tf2::Vector3(out.pose.x(), out.pose.y(), 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, out.pose.theta());
  T_map_base.setRotation(q);

  const tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

  geometry_msgs::msg::TransformStamped t_map_odom;
  t_map_odom.header.stamp = this->now();
  t_map_odom.header.frame_id = map_frame_;
  t_map_odom.child_frame_id = odom_frame_;
  t_map_odom.transform = tf2::toMsg(T_map_odom);
  tf_broadcaster_->sendTransform(t_map_odom);
}

}  // namespace fusion_graph

// ── Entry point ──────────────────────────────────────────────────────

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fusion_graph::FusionGraphNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
