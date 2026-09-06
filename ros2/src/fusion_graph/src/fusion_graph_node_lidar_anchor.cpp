// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — LiDAR map anchor (Beluga). See lidar_map_anchor_gate.hpp
// for the two regimes and lidar_occupancy_mapper.hpp for the grid.

#include <algorithm>
#include <chrono>
#include <execution>
#include <utility>
#include <vector>

#include "fusion_graph/fusion_graph_node.hpp"
#include <beluga/beluga.hpp>

namespace fusion_graph
{

namespace
{
double MonotonicSeconds()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}
}  // namespace

void FusionGraphNode::RebuildLidarAnchorMap()
{
  const auto exported = lidar_mapper_->Export();
  lidar_map_occupied_cells_ = exported.occupied;
  auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  msg->header.frame_id = map_frame_;
  msg->header.stamp = this->now();
  msg->info.resolution = static_cast<float>(exported.resolution_m);
  msg->info.width = static_cast<uint32_t>(exported.width);
  msg->info.height = static_cast<uint32_t>(exported.height);
  msg->info.origin.position.x = exported.origin_x;
  msg->info.origin.position.y = exported.origin_y;
  msg->info.origin.orientation.w = 1.0;
  msg->data = exported.data;
  if (lidar_map_pub_)
    lidar_map_pub_->publish(*msg);
  if (exported.occupied == 0)
    return;  // published for inspection, but nothing to localise against yet
  beluga_ros::OccupancyGrid grid(msg);
  if (!lidar_anchor_filter_)
  {
    beluga::DifferentialDriveModelParam motion;
    motion.rotation_noise_from_rotation = lidar_anchor_odom_alpha_rot_;
    motion.rotation_noise_from_translation = lidar_anchor_odom_alpha_rot_;
    motion.translation_noise_from_translation = lidar_anchor_odom_alpha_trans_;
    motion.translation_noise_from_rotation = lidar_anchor_odom_alpha_trans_;
    beluga::LikelihoodFieldModelParam sensor;
    sensor.max_obstacle_distance = 2.0;
    sensor.max_laser_distance = lidar_anchor_max_laser_distance_m_;
    sensor.z_hit = lidar_anchor_z_hit_;
    sensor.z_random = lidar_anchor_z_rand_;
    sensor.sigma_hit = lidar_anchor_sigma_hit_m_;
    beluga_ros::AmclParams amcl;
    amcl.update_min_d = lidar_anchor_update_min_d_;
    amcl.update_min_a = lidar_anchor_update_min_a_;
    amcl.min_particles = static_cast<std::size_t>(std::max(1, lidar_anchor_min_particles_));
    amcl.max_particles = static_cast<std::size_t>(
        std::max(lidar_anchor_min_particles_, lidar_anchor_max_particles_));
    lidar_anchor_filter_ = std::make_unique<beluga_ros::Amcl>(
        grid,
        beluga::DifferentialDriveModel2d{motion},
        beluga::LikelihoodFieldModel<beluga_ros::OccupancyGrid>{sensor, grid},
        amcl,
        std::execution::seq);
  }
  else
  {
    lidar_anchor_filter_->update_map(grid);
  }
}

void FusionGraphNode::LidarMapAnchorStep(const std::vector<Eigen::Vector2d>& curr_scan,
                                         bool curr_valid)
{
  if (!lidar_mapper_ || !lidar_anchor_gate_ || !curr_valid)
    return;
  const double now_s = MonotonicSeconds();
  // Age of the last accepted RTK-Fixed receipt, in seconds; huge when none.
  double rtk_age_s = 1.0e9;
  if (last_rtk_fixed_stamp_)
  {
    rtk_age_s = std::max(0.0, (this->now() - *last_rtk_fixed_stamp_).seconds());
  }
  const bool map_has_structure = lidar_map_occupied_cells_ > 0;
  const auto d = lidar_anchor_gate_->Step(rtk_age_s, map_has_structure, now_s);

  auto snapshot = graph_->LatestSnapshot();

  if (d.insert_scan && snapshot)
  {
    std::vector<std::pair<double, double>> pts;
    pts.reserve(curr_scan.size());
    for (const auto& p : curr_scan)
      pts.emplace_back(p.x(), p.y());
    lidar_mapper_->Insert(snapshot->pose.x(), snapshot->pose.y(), snapshot->pose.theta(), pts);
    const bool due = (now_s - lidar_map_last_rebuild_s_) >= lidar_map_rebuild_period_s_;
    if (due && lidar_mapper_->inserted_scans() > lidar_map_scans_at_rebuild_)
    {
      RebuildLidarAnchorMap();
      lidar_map_last_rebuild_s_ = now_s;
      lidar_map_scans_at_rebuild_ = lidar_mapper_->inserted_scans();
    }
    return;
  }

  if (!d.run_filter || !lidar_anchor_filter_)
    return;

  if (d.seed_filter && snapshot)
  {
    // Start converged at the last trusted fused pose: the graph was
    // GPS-anchored until moments ago, and a global relocalisation here would
    // throw that away.
    const Sophus::SE2d seed(snapshot->pose.theta(),
                            Eigen::Vector2d(snapshot->pose.x(), snapshot->pose.y()));
    Sophus::Matrix3d cov = Sophus::Matrix3d::Zero();
    const double sxy = std::max(lidar_anchor_seed_sigma_xy_m_,
                                std::sqrt(std::max(snapshot->covariance(0, 0), 0.0)));
    const double syy = std::max(lidar_anchor_seed_sigma_xy_m_,
                                std::sqrt(std::max(snapshot->covariance(1, 1), 0.0)));
    cov(0, 0) = sxy * sxy;
    cov(1, 1) = syy * syy;
    cov(2, 2) = lidar_anchor_seed_sigma_theta_rad_ * lidar_anchor_seed_sigma_theta_rad_;
    lidar_anchor_filter_->initialize(seed, cov);
    ++lidar_anchor_seeds_;
  }

  // Motion input is the local dead-reckoning pose (odom frame) this node
  // already integrates; the filter only uses successive differences.
  const Sophus::SE2d base_in_odom(dr_yaw_, Eigen::Vector2d(dr_x_, dr_y_));
  std::vector<std::pair<double, double>> pts;
  const std::size_t stride = std::max<std::size_t>(
      1, curr_scan.size() / static_cast<std::size_t>(std::max(1, lidar_anchor_max_beams_)));
  pts.reserve(curr_scan.size() / stride + 1);
  for (std::size_t i = 0; i < curr_scan.size(); i += stride)
    pts.emplace_back(curr_scan[i].x(), curr_scan[i].y());

  const auto est = lidar_anchor_filter_->update(base_in_odom, std::move(pts));
  if (!est)
  {
    ++lidar_anchor_skipped_;  // below update_min_d / update_min_a: nothing new to say
    return;
  }
  const auto& [pose, cov3] = *est;
  Eigen::Matrix2d cov2 = cov3.topLeftCorner<2, 2>();
  graph_->QueueLidarMapXy(gtsam::Vector2(pose.translation().x(), pose.translation().y()),
                          cov2,
                          /*robust=*/true);
  ++lidar_anchor_updates_;
}

}  // namespace fusion_graph
