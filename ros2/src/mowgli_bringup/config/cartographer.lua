-- Cartographer configuration for MowgliNext robot mower.
--
-- Architecture (simplified vs slam_toolbox):
--   map → odom → base_footprint
--   Cartographer publishes map→odom directly.
--   FusionCore publishes odom→base_footprint (with GPS enabled).
--   No GPS-SLAM corrector needed — GPS is fused in FusionCore.
--
-- Tuned for outdoor garden with:
--   - LD19 LiDAR (8 Hz, ~500 rays, 25m range)
--   - FusionCore odometry (GPS+IMU+wheels fused)
--   - Low feature density (hedges, fences, trees)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,        -- FusionCore owns odom→base_footprint
  publish_frame_projected_to_2d = true,
  use_odometry = true,               -- FusionCore's /fusion/odom (GPS+IMU+wheels)
  use_nav_sat = false,               -- GPS already in FusionCore odometry
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.05,    -- 20 Hz TF publish (match slam_toolbox)
  trajectory_publish_period_sec = 0.1,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 2D trajectory builder for outdoor mowing
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2  -- ARM has limited cores

-- Disable IMU input to Cartographer: it asserts the IMU frame must be
-- colocated with tracking_frame (base_footprint), which our imu_link
-- is not. FusionCore's /fusion/odom already has IMU fused in — Cartographer
-- only needs LiDAR + odometry for 2D scan matching on flat ground.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.20            -- filter grass near robot
TRAJECTORY_BUILDER_2D.max_range = 8.0             -- match slam_toolbox range
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Submap parameters
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60  -- ~7.5s at 8Hz LiDAR
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Real-time correlative scan matcher (coarse search before Ceres)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres scan matcher (fine tuning)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0

-- Motion filter — don't process scans unless robot moved enough
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(15.0)

-- Pose graph optimization
POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.optimization_problem.huber_scale = 1e1

-- Constraint builder (loop closure)
POSE_GRAPH.constraint_builder.min_score = 0.55       -- strict for noisy outdoor
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3   -- save CPU on ARM
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.log_matches = true

-- Fast correlative scan matcher for loop closure
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.0)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres scan matcher for loop closure refinement
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.0

-- Global optimization
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5

return options
