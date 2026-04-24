// Copyright (C) 2024 Cedric <cedric@mowgli.dev>
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

#include "mowgli_map/map_server_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace mowgli_map
{

// Simple parser for /ros2_ws/maps/dock_calibration.yaml — see the twin
// helper in hardware_bridge_node.cpp. Duplicated locally to avoid
// introducing a shared header for a few dozen lines.
struct DockCalibrationFile
{
  double x{0.0};
  double y{0.0};
  double yaw_rad{0.0};
};

inline std::optional<double> parse_yaml_double(const std::string& content,
                                               const std::string& key)
{
  const std::string needle = key + ":";
  auto pos = content.find(needle);
  if (pos == std::string::npos) return std::nullopt;
  pos += needle.size();
  while (pos < content.size() &&
         (content[pos] == ' ' || content[pos] == '\t')) ++pos;
  auto end = pos;
  while (end < content.size() && content[end] != '\n' && content[end] != '\r') ++end;
  try
  {
    return std::stod(content.substr(pos, end - pos));
  }
  catch (...)
  {
    return std::nullopt;
  }
}

inline std::optional<DockCalibrationFile> load_dock_calibration_file(
    const std::string& path)
{
  std::ifstream f(path);
  if (!f.good()) return std::nullopt;
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string content = ss.str();
  auto x = parse_yaml_double(content, "dock_pose_x");
  auto y = parse_yaml_double(content, "dock_pose_y");
  auto yaw = parse_yaml_double(content, "dock_pose_yaw_rad");
  if (!x || !y || !yaw) return std::nullopt;
  return DockCalibrationFile{*x, *y, *yaw};
}

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

MapServerNode::MapServerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("map_server_node", options)
{
  // ── Declare and read parameters ──────────────────────────────────────────
  resolution_ = declare_parameter<double>("resolution", 0.05);
  map_size_x_ = declare_parameter<double>("map_size_x", 20.0);
  map_size_y_ = declare_parameter<double>("map_size_y", 20.0);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  decay_rate_per_hour_ = declare_parameter<double>("decay_rate_per_hour", 0.1);
  mower_width_ = declare_parameter<double>("mower_width", 0.18);
  map_file_path_ = declare_parameter<std::string>("map_file_path", "");
  areas_file_path_ = declare_parameter<std::string>("areas_file_path", "");
  publish_rate_ = declare_parameter<double>("publish_rate", 1.0);
  keepout_nav_margin_ = declare_parameter<double>("keepout_nav_margin", 1.5);
  // Two-tier boundary: if the robot is outside every defined area, we
  // publish /boundary_violation (BT attempts a recovery back inside). If
  // the robot is further than lethal_boundary_margin beyond any area
  // edge, we also publish /lethal_boundary_violation — BT must
  // emergency-stop because blade/motors outside the authorised zone
  // can do real damage.
  lethal_boundary_margin_m_ = declare_parameter<double>("lethal_boundary_margin_m", 0.5);
  boundary_recovery_offset_m_ =
      declare_parameter<double>("boundary_recovery_offset_m", 0.8);
  boundary_inner_margin_m_ =
      declare_parameter<double>("boundary_inner_margin_m", 0.3);
  strip_boundary_margin_m_ =
      declare_parameter<double>("strip_boundary_margin_m", 0.5);

  // Dock approach corridor — extends the no-mow zone in front of the dock
  // so coverage strips stop before the 1.5 m straight-line alignment that
  // opennav_docking needs for the final approach. Length is measured from
  // dock_pose in the -X direction (dock local frame, same direction as
  // staging_x_offset). Width is symmetric around the approach axis.
  dock_approach_corridor_length_m_ =
      declare_parameter<double>("dock_approach_corridor_length_m", 1.5);
  dock_approach_corridor_half_width_m_ =
      declare_parameter<double>("dock_approach_corridor_half_width_m", 0.40);

  RCLCPP_INFO(get_logger(),
              "MapServerNode: resolution=%.3f m, size=%.1f×%.1f m, frame='%s'",
              resolution_,
              map_size_x_,
              map_size_y_,
              map_frame_.c_str());

  // ── TF buffer for map-frame robot position lookup ────────────────────────
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Initialise map ───────────────────────────────────────────────────────
  init_map();
  last_decay_time_ = now();

  // ── Publishers ───────────────────────────────────────────────────────────
  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("~/grid_map", rclcpp::QoS(1));

  mow_progress_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("~/mow_progress", rclcpp::QoS(1));

  coverage_cells_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("~/coverage_cells", rclcpp::QoS(1));

  // Costmap filter publishers: transient_local durability so that Nav2 costmap
  // filter nodes that start after this node still receive the latched message.
  auto transient_qos = rclcpp::QoS(1).transient_local();

  keepout_filter_info_pub_ =
      create_publisher<nav2_msgs::msg::CostmapFilterInfo>("/costmap_filter_info", transient_qos);

  keepout_mask_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("/keepout_mask", transient_qos);

  speed_filter_info_pub_ =
      create_publisher<nav2_msgs::msg::CostmapFilterInfo>("/speed_filter_info", transient_qos);

  speed_mask_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/speed_mask", transient_qos);

  // ── Subscribers ──────────────────────────────────────────────────────────
  occupancy_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(1),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
      {
        on_occupancy_grid(std::move(msg));
      });

  status_sub_ = create_subscription<mowgli_interfaces::msg::Status>(
      "/hardware_bridge/status",
      rclcpp::QoS(1),
      [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg)
      {
        on_mower_status(std::move(msg));
      });

  auto odom_topic = declare_parameter<std::string>("odom_topic", "/fusion/odom");
  odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(odom_topic,
                                                   rclcpp::QoS(1),
                                                   [this](
                                                       nav_msgs::msg::Odometry::ConstSharedPtr msg)
                                                   {
                                                     on_odom(std::move(msg));
                                                   });

  // ── Services ─────────────────────────────────────────────────────────────
  save_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/save_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_save_map(req, res);
      });

  load_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/load_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_load_map(req, res);
      });

  clear_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_clear_map(req, res);
      });

  add_area_srv_ = create_service<mowgli_interfaces::srv::AddMowingArea>(
      "~/add_area",
      [this](const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
             mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res)
      {
        on_add_area(req, res);
      });

  get_mowing_area_srv_ = create_service<mowgli_interfaces::srv::GetMowingArea>(
      "~/get_mowing_area",
      [this](const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr req,
             mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res)
      {
        on_get_mowing_area(req, res);
      });

  set_docking_point_srv_ = create_service<mowgli_interfaces::srv::SetDockingPoint>(
      "~/set_docking_point",
      [this](const mowgli_interfaces::srv::SetDockingPoint::Request::SharedPtr req,
             mowgli_interfaces::srv::SetDockingPoint::Response::SharedPtr res)
      {
        on_set_docking_point(req, res);
      });

  save_areas_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/save_areas",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_save_areas(req, res);
      });

  load_areas_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/load_areas",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_load_areas(req, res);
      });

  // ── Strip planner services ──────────────────────────────────────────────
  get_next_strip_srv_ = create_service<mowgli_interfaces::srv::GetNextStrip>(
      "~/get_next_strip",
      [this](const mowgli_interfaces::srv::GetNextStrip::Request::SharedPtr req,
             mowgli_interfaces::srv::GetNextStrip::Response::SharedPtr res)
      {
        on_get_next_strip(req, res);
      });

  get_coverage_status_srv_ = create_service<mowgli_interfaces::srv::GetCoverageStatus>(
      "~/get_coverage_status",
      [this](const mowgli_interfaces::srv::GetCoverageStatus::Request::SharedPtr req,
             mowgli_interfaces::srv::GetCoverageStatus::Response::SharedPtr res)
      {
        on_get_coverage_status(req, res);
      });

  get_recovery_point_srv_ = create_service<mowgli_interfaces::srv::GetRecoveryPoint>(
      "~/get_recovery_point",
      [this](const mowgli_interfaces::srv::GetRecoveryPoint::Request::SharedPtr req,
             mowgli_interfaces::srv::GetRecoveryPoint::Response::SharedPtr res)
      {
        on_get_recovery_point(req, res);
      });

  // ── Replanning parameters ────────────────────────────────────────────────
  replan_cooldown_sec_ = declare_parameter<double>("replan_cooldown_sec", 30.0);
  last_replan_time_ = now();

  // ── Replan / boundary publishers ────────────────────────────────────────
  replan_needed_pub_ = create_publisher<std_msgs::msg::Bool>("~/replan_needed", rclcpp::QoS(1));
  boundary_violation_pub_ =
      create_publisher<std_msgs::msg::Bool>("~/boundary_violation", rclcpp::QoS(1));
  lethal_boundary_violation_pub_ =
      create_publisher<std_msgs::msg::Bool>("~/lethal_boundary_violation",
                                            rclcpp::QoS(1));

  docking_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/docking_pose",
                                                        rclcpp::QoS(1).transient_local());

  // ── Obstacle subscription ─────────────────────────────────────────────
  obstacle_sub_ = create_subscription<mowgli_interfaces::msg::ObstacleArray>(
      "/obstacle_tracker/obstacles",
      rclcpp::QoS(1),
      [this](mowgli_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
      {
        on_obstacles(std::move(msg));
      });

  // ── Load pre-defined areas from parameters ────────────────────────────
  load_areas_from_params();

  // ── Auto-load persisted areas from file (overrides parameter areas) ───
  if (!areas_file_path_.empty())
  {
    try
    {
      load_areas_from_file(areas_file_path_);
      RCLCPP_INFO(get_logger(), "Loaded persisted areas from %s", areas_file_path_.c_str());
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(get_logger(), "No persisted areas to load: %s", ex.what());
    }
  }

  // Resize map to fit loaded areas (if any).
  resize_map_to_areas();

  // If no docking pose was loaded from the persisted file, initialise from
  // the dock_pose_x/y/yaw parameters in mowgli_robot.yaml. This ensures the
  // GUI and BT always have a dock pose on first boot.
  if (!docking_pose_set_)
  {
    double dock_x = declare_parameter<double>("dock_pose_x", 0.0);
    double dock_y = declare_parameter<double>("dock_pose_y", 0.0);
    double dock_yaw = declare_parameter<double>("dock_pose_yaw", 0.0);
    const char* dock_source = "parameters";

    // Override the config-file dock pose with the calibrated value when
    // available. The file is written by /calibrate_imu_yaw_node/calibrate
    // and carries a GPS-derived yaw with ~1° σ — considerably better than
    // the phone-compass value a user enters once at install time.
    if (auto file_cal = load_dock_calibration_file(
            "/ros2_ws/maps/dock_calibration.yaml"))
    {
      dock_x = file_cal->x;
      dock_y = file_cal->y;
      dock_yaw = file_cal->yaw_rad;
      dock_source = "dock_calibration.yaml";
    }

    if (dock_x != 0.0 || dock_y != 0.0 || dock_yaw != 0.0)
    {
      docking_pose_.position.x = dock_x;
      docking_pose_.position.y = dock_y;
      docking_pose_.position.z = 0.0;
      docking_pose_.orientation.w = std::cos(dock_yaw / 2.0);
      docking_pose_.orientation.z = std::sin(dock_yaw / 2.0);
      docking_pose_.orientation.x = 0.0;
      docking_pose_.orientation.y = 0.0;
      docking_pose_set_ = true;
      RCLCPP_INFO(get_logger(),
                  "Dock pose from %s: (%.3f, %.3f) yaw=%.3f",
                  dock_source,
                  dock_x,
                  dock_y,
                  dock_yaw);
    }
  }

  // Publish docking pose if available (transient_local ensures late subscribers get it).
  if (docking_pose_set_)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = docking_pose_;
    docking_pose_pub_->publish(pose_msg);

    // Store dock exclusion polygon — used to mark dock cells as NO_GO_ZONE
    // in the classification layer so strips are not planned through the dock
    // NOR through the straight-line approach corridor that opennav_docking
    // needs for the final 1.5 m alignment. Rectangle in dock local frame:
    //   +X (into dock structure): dock_forward (covers robot when docked)
    //   -X (approach corridor)  : dock_approach_corridor_length_m_
    //   ±Y                      : dock_approach_corridor_half_width_m_
    // This is asymmetric — the robot must stay out of the approach lane so
    // it always reaches staging pose with the correct heading, but we still
    // cover the dock structure itself.
    const double dock_forward = 0.45;  // +X extent into dock (was symmetric)
    const double approach_back = dock_approach_corridor_length_m_;
    const double half_width = dock_approach_corridor_half_width_m_;
    const double d_x = docking_pose_.position.x;
    const double d_y = docking_pose_.position.y;
    const double d_yaw = 2.0 * std::atan2(docking_pose_.orientation.z,
                                           docking_pose_.orientation.w);
    const double cy = std::cos(d_yaw);
    const double sy = std::sin(d_yaw);
    const double corners[][2] = {
        { dock_forward,  half_width},
        { dock_forward, -half_width},
        {-approach_back, -half_width},
        {-approach_back,  half_width},
    };
    for (const auto& c : corners)
    {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(d_x + cy * c[0] - sy * c[1]);
      pt.y = static_cast<float>(d_y + sy * c[0] + cy * c[1]);
      pt.z = 0.0f;
      dock_exclusion_polygon_.points.push_back(pt);
    }
    dock_exclusion_polygon_.points.push_back(dock_exclusion_polygon_.points.front());
    has_dock_exclusion_ = true;
    RCLCPP_INFO(get_logger(),
                "Dock exclusion zone (with approach corridor): pose=(%.2f, %.2f) "
                "yaw=%.2f, forward=%.2fm, approach=%.2fm, half_width=%.2fm",
                d_x, d_y, d_yaw,
                dock_forward, approach_back, half_width);
  }

  // ── Publish timer ────────────────────────────────────────────────────────
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));

  publish_timer_ = create_wall_timer(period_ns,
                                     [this]()
                                     {
                                       on_publish_timer();
                                     });

  RCLCPP_INFO(get_logger(), "MapServerNode ready (%zu areas loaded).", areas_.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// Area loading from parameters
// ─────────────────────────────────────────────────────────────────────────────

geometry_msgs::msg::Polygon MapServerNode::parse_polygon_string(const std::string& s)
{
  geometry_msgs::msg::Polygon poly;
  if (s.empty())
  {
    return poly;
  }

  std::istringstream pts_stream(s);
  std::string point_str;
  while (std::getline(pts_stream, point_str, ';'))
  {
    std::istringstream coord_stream(point_str);
    std::string x_str, y_str;
    if (std::getline(coord_stream, x_str, ',') && std::getline(coord_stream, y_str, ','))
    {
      geometry_msgs::msg::Point32 p;
      p.x = std::stof(x_str);
      p.y = std::stof(y_str);
      p.z = 0.0f;
      poly.points.push_back(p);
    }
  }
  return poly;
}

void MapServerNode::load_areas_from_params()
{
  // Declare area parameter arrays with empty defaults.
  const auto area_names =
      declare_parameter<std::vector<std::string>>("area_names", std::vector<std::string>{});
  const auto area_polygons =
      declare_parameter<std::vector<std::string>>("area_polygons", std::vector<std::string>{});
  const auto area_is_navigation =
      declare_parameter<std::vector<bool>>("area_is_navigation", std::vector<bool>{});
  const auto area_obstacles =
      declare_parameter<std::vector<std::string>>("area_obstacles", std::vector<std::string>{});

  if (area_names.empty())
  {
    RCLCPP_WARN(get_logger(),
                "No areas configured (area_names is empty). "
                "Keepout mask will not be published until areas are added via service.");
    return;
  }

  if (area_names.size() != area_polygons.size())
  {
    RCLCPP_ERROR(get_logger(),
                 "area_names (%zu) and area_polygons (%zu) must have the same length!",
                 area_names.size(),
                 area_polygons.size());
    return;
  }

  for (std::size_t i = 0; i < area_names.size(); ++i)
  {
    AreaEntry entry;
    entry.name = area_names[i];
    entry.polygon = parse_polygon_string(area_polygons[i]);
    entry.is_navigation_area = (i < area_is_navigation.size()) && area_is_navigation[i];

    if (entry.polygon.points.size() < 3)
    {
      RCLCPP_WARN(get_logger(),
                  "Skipping area '%s': polygon has %zu vertices (need >= 3)",
                  entry.name.c_str(),
                  entry.polygon.points.size());
      continue;
    }

    // Parse obstacle polygons (semicolon-separated polygon strings, pipe-separated).
    // Format: "x1,y1;x2,y2;x3,y3|x4,y4;x5,y5;x6,y6" for multiple obstacles.
    if (i < area_obstacles.size() && !area_obstacles[i].empty())
    {
      std::istringstream obs_stream(area_obstacles[i]);
      std::string obs_str;
      while (std::getline(obs_stream, obs_str, '|'))
      {
        auto obs_poly = parse_polygon_string(obs_str);
        if (obs_poly.points.size() >= 3)
        {
          entry.obstacles.push_back(obs_poly);
          obstacle_polygons_.push_back(obs_poly);
        }
      }
    }

    RCLCPP_INFO(get_logger(),
                "Loaded area '%s': %zu vertices, %s, %zu obstacles",
                entry.name.c_str(),
                entry.polygon.points.size(),
                entry.is_navigation_area ? "navigation" : "mowing",
                entry.obstacles.size());

    areas_.push_back(std::move(entry));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Map initialisation
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::init_map()
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  map_ = grid_map::GridMap({std::string(layers::OCCUPANCY),
                            std::string(layers::CLASSIFICATION),
                            std::string(layers::MOW_PROGRESS),
                            std::string(layers::CONFIDENCE)});

  map_.setFrameId(map_frame_);
  map_.setGeometry(grid_map::Length(map_size_x_, map_size_y_),
                   resolution_,
                   grid_map::Position(0.0, 0.0));

  map_[std::string(layers::OCCUPANCY)].setConstant(defaults::OCCUPANCY);
  map_[std::string(layers::CLASSIFICATION)].setConstant(defaults::CLASSIFICATION);
  map_[std::string(layers::MOW_PROGRESS)].setConstant(defaults::MOW_PROGRESS);
  map_[std::string(layers::CONFIDENCE)].setConstant(defaults::CONFIDENCE);

  RCLCPP_DEBUG(get_logger(),
               "Grid map created: %zu×%zu cells",
               static_cast<std::size_t>(map_.getSize()(0)),
               static_cast<std::size_t>(map_.getSize()(1)));
}

void MapServerNode::resize_map_to_areas()
{
  if (areas_.empty())
  {
    return;
  }

  // Compute bounding box of all area polygons.
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& area : areas_)
  {
    for (const auto& pt : area.polygon.points)
    {
      min_x = std::min(min_x, static_cast<double>(pt.x));
      max_x = std::max(max_x, static_cast<double>(pt.x));
      min_y = std::min(min_y, static_cast<double>(pt.y));
      max_y = std::max(max_y, static_cast<double>(pt.y));
    }
  }

  // Add 5m margin on each side for navigation around the areas.
  constexpr double margin = 5.0;
  const double new_size_x = (max_x - min_x) + 2.0 * margin;
  const double new_size_y = (max_y - min_y) + 2.0 * margin;
  const double center_x = (min_x + max_x) * 0.5;
  const double center_y = (min_y + max_y) * 0.5;

  // Only resize if the new size differs meaningfully from the current one.
  if (std::abs(new_size_x - map_size_x_) < resolution_ &&
      std::abs(new_size_y - map_size_y_) < resolution_)
  {
    return;
  }

  map_size_x_ = new_size_x;
  map_size_y_ = new_size_y;

  std::lock_guard<std::mutex> lock(map_mutex_);
  map_.setGeometry(grid_map::Length(map_size_x_, map_size_y_),
                   resolution_,
                   grid_map::Position(center_x, center_y));

  map_[std::string(layers::OCCUPANCY)].setConstant(defaults::OCCUPANCY);
  map_[std::string(layers::CLASSIFICATION)].setConstant(defaults::CLASSIFICATION);
  map_[std::string(layers::MOW_PROGRESS)].setConstant(defaults::MOW_PROGRESS);
  map_[std::string(layers::CONFIDENCE)].setConstant(defaults::CONFIDENCE);

  masks_dirty_ = true;

  RCLCPP_INFO(get_logger(),
              "Map resized to %.1f×%.1f m (center: %.1f, %.1f) to fit %zu areas",
              map_size_x_,
              map_size_y_,
              center_x,
              center_y,
              areas_.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// Subscription callbacks
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_occupancy_grid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  grid_map::GridMap incoming;
  if (!grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "occupancy_in", incoming))
  {
    RCLCPP_WARN(get_logger(), "on_occupancy_grid: failed to convert OccupancyGrid");
    return;
  }

  const auto& info = msg->info;
  const float res = static_cast<float>(info.resolution);
  const float ox = static_cast<float>(info.origin.position.x) + res * 0.5F;
  const float oy = static_cast<float>(info.origin.position.y) + res * 0.5F;

  for (uint32_t row = 0; row < info.height; ++row)
  {
    for (uint32_t col = 0; col < info.width; ++col)
    {
      const int8_t cell_val = msg->data[static_cast<std::size_t>(row * info.width + col)];
      if (cell_val < 0)
      {
        continue;
      }

      const grid_map::Position pos(static_cast<double>(ox + static_cast<float>(col) * res),
                                   static_cast<double>(oy + static_cast<float>(row) * res));

      grid_map::Index idx;
      if (!map_.getIndex(pos, idx))
      {
        continue;
      }

      map_.at(std::string(layers::OCCUPANCY), idx) = (cell_val > 50) ? 1.0F : 0.0F;
    }
  }
}

void MapServerNode::on_mower_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg)
{
  mow_blade_enabled_ = msg->mow_enabled;
}

void MapServerNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr /*msg*/)
{
  // Use TF for the definitive map-frame robot position.
  // The odom message position may be in odom frame, not map frame.
  double x = 0.0, y = 0.0;
  if (tf_buffer_)
  {
    try
    {
      auto tf = tf_buffer_->lookupTransform(map_frame_, "base_link", tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
    }
    catch (const tf2::TransformException&)
    {
      return;  // No TF yet, skip
    }
  }
  else
  {
    return;
  }

  check_boundary_violation(x, y);

  if (!mow_blade_enabled_)
  {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    mark_cells_mowed(x, y);
  }
}

void MapServerNode::on_obstacles(mowgli_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  diff_and_update_obstacles(msg->obstacles);
}

// ─────────────────────────────────────────────────────────────────────────────
// Timer callback
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_publish_timer()
{
  const rclcpp::Time now_time = now();
  const double elapsed = (now_time - last_decay_time_).seconds();
  last_decay_time_ = now_time;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    apply_decay(elapsed);

    auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(map_);
    grid_map_pub_->publish(std::move(grid_map_msg));

    mow_progress_pub_->publish(mow_progress_to_occupancy_grid());
    coverage_cells_pub_->publish(coverage_cells_to_occupancy_grid());

    // Only recompute masks when areas or obstacles changed.
    if (masks_dirty_)
    {
      publish_keepout_mask();
      publish_speed_mask();
      masks_dirty_ = false;
    }
    else
    {
      // Republish cached masks (cheap — just a copy + publish).
      cached_keepout_mask_.header.stamp = now_time;
      keepout_mask_pub_->publish(cached_keepout_mask_);
      cached_speed_mask_.header.stamp = now_time;
      speed_mask_pub_->publish(cached_speed_mask_);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Service callbacks
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_save_map(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (map_file_path_.empty())
  {
    res->success = false;
    res->message = "map_file_path parameter is empty; cannot save.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try
  {
    std::lock_guard<std::mutex> lock(map_mutex_);

    const std::string yaml_path = map_file_path_ + ".yaml";
    const std::string data_path = map_file_path_ + ".dat";

    std::ofstream yaml(yaml_path);
    if (!yaml.is_open())
    {
      throw std::runtime_error("Cannot open " + yaml_path + " for writing");
    }
    yaml << "resolution: " << resolution_ << "\n"
         << "map_size_x: " << map_size_x_ << "\n"
         << "map_size_y: " << map_size_y_ << "\n"
         << "map_frame: " << map_frame_ << "\n"
         << "rows: " << map_.getSize()(0) << "\n"
         << "cols: " << map_.getSize()(1) << "\n"
         << "pos_x: " << map_.getPosition().x() << "\n"
         << "pos_y: " << map_.getPosition().y() << "\n";
    yaml.close();

    std::ofstream dat(data_path, std::ios::binary);
    if (!dat.is_open())
    {
      throw std::runtime_error("Cannot open " + data_path + " for writing");
    }

    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);

    const auto& occ = map_[std::string(layers::OCCUPANCY)];
    const auto& cls = map_[std::string(layers::CLASSIFICATION)];
    const auto& prog = map_[std::string(layers::MOW_PROGRESS)];
    const auto& conf = map_[std::string(layers::CONFIDENCE)];

    for (int r = 0; r < rows; ++r)
    {
      for (int c = 0; c < cols; ++c)
      {
        float vals[4] = {occ(r, c), cls(r, c), prog(r, c), conf(r, c)};
        dat.write(reinterpret_cast<const char*>(vals), sizeof(vals));
      }
    }
    dat.close();

    res->success = true;
    res->message = "Map saved to " + map_file_path_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("Save failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void MapServerNode::on_load_map(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (map_file_path_.empty())
  {
    res->success = false;
    res->message = "map_file_path parameter is empty; cannot load.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try
  {
    const std::string yaml_path = map_file_path_ + ".yaml";
    const std::string data_path = map_file_path_ + ".dat";

    std::ifstream yaml(yaml_path);
    if (!yaml.is_open())
    {
      throw std::runtime_error("Cannot open " + yaml_path);
    }

    double res_loaded{}, sx{}, sy{};
    std::string frame_loaded{};
    int rows_loaded{}, cols_loaded{};
    double pos_x{}, pos_y{};

    std::string line;
    while (std::getline(yaml, line))
    {
      std::istringstream ss(line);
      std::string key;
      if (!(ss >> key))
        continue;
      if (key == "resolution:")
        ss >> res_loaded;
      else if (key == "map_size_x:")
        ss >> sx;
      else if (key == "map_size_y:")
        ss >> sy;
      else if (key == "map_frame:")
        ss >> frame_loaded;
      else if (key == "rows:")
        ss >> rows_loaded;
      else if (key == "cols:")
        ss >> cols_loaded;
      else if (key == "pos_x:")
        ss >> pos_x;
      else if (key == "pos_y:")
        ss >> pos_y;
    }
    yaml.close();

    if (rows_loaded <= 0 || cols_loaded <= 0)
    {
      throw std::runtime_error("Invalid map dimensions in " + yaml_path);
    }

    std::lock_guard<std::mutex> lock(map_mutex_);

    resolution_ = res_loaded;
    map_size_x_ = sx;
    map_size_y_ = sy;
    map_frame_ = frame_loaded;

    map_ = grid_map::GridMap({std::string(layers::OCCUPANCY),
                              std::string(layers::CLASSIFICATION),
                              std::string(layers::MOW_PROGRESS),
                              std::string(layers::CONFIDENCE)});

    map_.setFrameId(map_frame_);
    map_.setGeometry(grid_map::Length(map_size_x_, map_size_y_),
                     resolution_,
                     grid_map::Position(pos_x, pos_y));

    std::ifstream dat(data_path, std::ios::binary);
    if (!dat.is_open())
    {
      throw std::runtime_error("Cannot open " + data_path);
    }

    auto& occ = map_[std::string(layers::OCCUPANCY)];
    auto& cls = map_[std::string(layers::CLASSIFICATION)];
    auto& prog = map_[std::string(layers::MOW_PROGRESS)];
    auto& conf = map_[std::string(layers::CONFIDENCE)];

    const int actual_rows = map_.getSize()(0);
    const int actual_cols = map_.getSize()(1);

    for (int r = 0; r < actual_rows && r < rows_loaded; ++r)
    {
      for (int c = 0; c < actual_cols && c < cols_loaded; ++c)
      {
        float vals[4] = {};
        dat.read(reinterpret_cast<char*>(vals), sizeof(vals));
        occ(r, c) = vals[0];
        cls(r, c) = vals[1];
        prog(r, c) = vals[2];
        conf(r, c) = vals[3];
      }
    }
    dat.close();

    last_decay_time_ = now();

    res->success = true;
    res->message = "Map loaded from " + map_file_path_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("Load failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void MapServerNode::on_clear_map(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                 std_srvs::srv::Trigger::Response::SharedPtr res)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    clear_map_layers();
  }
  areas_.clear();
  obstacle_polygons_.clear();
  docking_pose_set_ = false;
  keepout_filter_info_sent_ = false;
  speed_filter_info_sent_ = false;
  masks_dirty_ = true;

  res->success = true;
  res->message = "All map layers and areas cleared.";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void MapServerNode::on_add_area(const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
                                mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res)
{
  const auto& polygon_msg = req->area.area;

  if (polygon_msg.points.size() < 3)
  {
    res->success = false;
    RCLCPP_WARN(get_logger(), "add_area: polygon must have at least 3 points.");
    return;
  }

  // Build grid_map polygon from geometry_msgs polygon
  grid_map::Polygon gm_polygon;
  for (const auto& pt : polygon_msg.points)
  {
    gm_polygon.addVertex(grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
  }

  // Classify cells inside the area as LAWN (mowable), not NO_GO_ZONE.
  // Only exclusion zones and obstacles should be NO_GO_ZONE.
  const float lawn_val = static_cast<float>(CellType::LAWN);
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    for (grid_map::PolygonIterator it(map_, gm_polygon); !it.isPastEnd(); ++it)
    {
      map_.at(std::string(layers::CLASSIFICATION), *it) = lawn_val;
    }
  }

  // Store as an area entry.
  AreaEntry entry;
  entry.name = req->area.name;
  entry.polygon = polygon_msg;
  entry.is_navigation_area = req->is_navigation_area;

  // Store obstacle polygons from the MapArea message.
  // Only store in the area entry (static), NOT in obstacle_polygons_
  // (which is for dynamic LiDAR-detected obstacles).
  const float no_go_val = static_cast<float>(CellType::NO_GO_ZONE);
  for (const auto& obstacle : req->area.obstacles)
  {
    if (obstacle.points.size() >= 3)
    {
      entry.obstacles.push_back(obstacle);

      grid_map::Polygon obs_gm;
      for (const auto& pt : obstacle.points)
      {
        obs_gm.addVertex(grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
      }
      std::lock_guard<std::mutex> lock(map_mutex_);
      for (grid_map::PolygonIterator it(map_, obs_gm); !it.isPastEnd(); ++it)
      {
        map_.at(std::string(layers::CLASSIFICATION), *it) = no_go_val;
      }
    }
  }

  areas_.push_back(std::move(entry));
  resize_map_to_areas();
  masks_dirty_ = true;

  RCLCPP_INFO(get_logger(),
              "Added area '%s' (%s) with %zu vertices and %zu obstacles.",
              req->area.name.c_str(),
              req->is_navigation_area ? "navigation" : "mowing",
              polygon_msg.points.size(),
              req->area.obstacles.size());

  // Auto-save if persistence path is set.
  if (!areas_file_path_.empty())
  {
    try
    {
      save_areas_to_file(areas_file_path_);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(get_logger(), "Auto-save after area add failed: %s", ex.what());
    }
  }

  res->success = true;
}

void MapServerNode::on_get_mowing_area(
    const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr req,
    mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  const auto idx = static_cast<std::size_t>(req->index);
  if (idx < areas_.size())
  {
    const auto& entry = areas_[idx];
    res->area.name = entry.name;
    res->area.area = entry.polygon;
    // Start with user-defined (static) obstacles from config.
    res->area.obstacles = entry.obstacles;
    res->area.is_navigation_area = entry.is_navigation_area;

    // Also include persistent tracked obstacles from the obstacle tracker
    // so the coverage planner can avoid them in the initial plan.
    const auto n_static = res->area.obstacles.size();
    for (const auto& obs_poly : obstacle_polygons_)
    {
      if (obs_poly.points.size() >= 3)
      {
        res->area.obstacles.push_back(obs_poly);
      }
    }

    res->success = true;
    RCLCPP_INFO(get_logger(),
                "GetMowingArea[%u]: area='%s', %zu obstacles (%zu static + %zu tracked)",
                req->index,
                entry.name.c_str(),
                res->area.obstacles.size(),
                n_static,
                res->area.obstacles.size() - n_static);
  }
  else
  {
    res->success = false;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::clear_map_layers()
{
  map_[std::string(layers::OCCUPANCY)].setConstant(defaults::OCCUPANCY);
  map_[std::string(layers::CLASSIFICATION)].setConstant(defaults::CLASSIFICATION);
  map_[std::string(layers::MOW_PROGRESS)].setConstant(defaults::MOW_PROGRESS);
  map_[std::string(layers::CONFIDENCE)].setConstant(defaults::CONFIDENCE);
}

nav_msgs::msg::OccupancyGrid MapServerNode::mow_progress_to_occupancy_grid() const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = now();
  grid.header.frame_id = map_frame_;
  grid.info.resolution = static_cast<float>(resolution_);
  // grid_map: size(0) iterates along X (length_x), size(1) along Y (length_y).
  // OccupancyGrid: width = X cells, height = Y cells.
  // grid_map r=0 → X_max (decreasing), c=0 → Y_max (decreasing).
  // OccupancyGrid col=0 → X_min, row=0 → Y_min.
  const int nx = map_.getSize()(0);  // cells along X
  const int ny = map_.getSize()(1);  // cells along Y
  grid.info.width = static_cast<uint32_t>(nx);
  grid.info.height = static_cast<uint32_t>(ny);

  grid.info.origin.position.x = map_.getPosition().x() - map_.getLength().x() * 0.5;
  grid.info.origin.position.y = map_.getPosition().y() - map_.getLength().y() * 0.5;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  const auto& prog = map_[std::string(layers::MOW_PROGRESS)];

  grid.data.resize(static_cast<std::size_t>(nx * ny), 0);

  for (int r = 0; r < nx; ++r)
  {
    for (int c = 0; c < ny; ++c)
    {
      const float val = prog(r, c);
      const int og_col = nx - 1 - r;  // r=0 (X_max) → last col
      const int og_row = ny - 1 - c;  // c=0 (Y_max) → last row
      const auto flat_idx = static_cast<std::size_t>(og_row * nx + og_col);
      const float clamped = std::clamp(val, 0.0F, 1.0F);
      grid.data[flat_idx] = static_cast<int8_t>(std::lround(clamped * 100.0F));
    }
  }

  return grid;
}

nav_msgs::msg::OccupancyGrid MapServerNode::coverage_cells_to_occupancy_grid() const
{
  // grid_map: size(0) = cells along X, size(1) = cells along Y.
  // grid_map r=0 → X_max, c=0 → Y_max (both decrease with index).
  // OccupancyGrid: width = X, height = Y, col=0 → X_min, row=0 → Y_min.

  const auto& prog = map_[std::string(layers::MOW_PROGRESS)];
  const auto& cls = map_[std::string(layers::CLASSIFICATION)];
  const int nx = map_.getSize()(0);
  const int ny = map_.getSize()(1);

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = now();
  grid.header.frame_id = map_frame_;
  grid.info.resolution = static_cast<float>(resolution_);
  grid.info.width = static_cast<uint32_t>(nx);
  grid.info.height = static_cast<uint32_t>(ny);
  grid.info.origin.position.x = map_.getPosition().x() - map_.getLength().x() * 0.5;
  grid.info.origin.position.y = map_.getPosition().y() - map_.getLength().y() * 0.5;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(static_cast<std::size_t>(nx * ny), -1);

  for (int r = 0; r < nx; ++r)
  {
    for (int c = 0; c < ny; ++c)
    {
      const int og_col = nx - 1 - r;
      const int og_row = ny - 1 - c;
      const auto flat_idx = static_cast<std::size_t>(og_row * nx + og_col);

      grid_map::Position pos;
      const grid_map::Index idx(r, c);
      if (!map_.getPosition(idx, pos))
        continue;

      bool in_area = false;
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(pos.x());
      pt.y = static_cast<float>(pos.y());
      for (const auto& area : areas_)
      {
        if (area.is_navigation_area)
          continue;
        if (point_in_polygon(pt, area.polygon))
        {
          in_area = true;
          break;
        }
      }

      if (!in_area)
        continue;

      auto cell_type = static_cast<CellType>(static_cast<int>(cls(r, c)));
      if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY ||
          cell_type == CellType::NO_GO_ZONE)
      {
        grid.data[flat_idx] = 100;
      }
      else if (prog(r, c) >= 0.3f)
      {
        grid.data[flat_idx] = 0;
      }
      else
      {
        grid.data[flat_idx] = 60;
      }
    }
  }

  return grid;
}

void MapServerNode::apply_decay(double elapsed_seconds)
{
  if (elapsed_seconds <= 0.0 || decay_rate_per_hour_ <= 0.0)
  {
    return;
  }

  const double decay_per_second = decay_rate_per_hour_ / 3600.0;
  const float decay = static_cast<float>(decay_per_second * elapsed_seconds);

  auto& prog = map_[std::string(layers::MOW_PROGRESS)];
  prog = (prog.array() - decay).max(0.0F).matrix();
}

void MapServerNode::mark_cells_mowed(double x, double y)
{
  const grid_map::Position center(x, y);
  const double radius = mower_width_ * 0.5;

  for (grid_map::CircleIterator it(map_, center, radius); !it.isPastEnd(); ++it)
  {
    map_.at(std::string(layers::MOW_PROGRESS), *it) = 1.0F;
    map_.at(std::string(layers::CONFIDENCE), *it) += 1.0F;
  }
}

bool MapServerNode::point_in_polygon(const geometry_msgs::msg::Point32& pt,
                                     const geometry_msgs::msg::Polygon& polygon) noexcept
{
  const auto& pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 3)
  {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const float xi = pts[i].x, yi = pts[i].y;
    const float xj = pts[j].x, yj = pts[j].y;

    const bool intersect =
        ((yi > pt.y) != (yj > pt.y)) && (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi);

    if (intersect)
    {
      inside = !inside;
    }
  }
  return inside;
}

// ─────────────────────────────────────────────────────────────────────────────
// Costmap filter mask helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Closest point on the polygon perimeter to (px, py), plus its distance.
struct ClosestEdge
{
  double x{0.0};
  double y{0.0};
  double distance{std::numeric_limits<double>::max()};
};

static ClosestEdge closest_edge_point(double px, double py,
                                      const geometry_msgs::msg::Polygon& polygon)
{
  ClosestEdge best;
  const auto& pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 2)
  {
    return best;
  }

  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double ax = static_cast<double>(pts[j].x);
    const double ay = static_cast<double>(pts[j].y);
    const double bx = static_cast<double>(pts[i].x);
    const double by = static_cast<double>(pts[i].y);

    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;

    double t = 0.0;
    if (len2 > 1e-12)
    {
      t = std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0, 1.0);
    }

    const double cx = ax + t * dx;
    const double cy = ay + t * dy;
    const double dist = std::hypot(px - cx, py - cy);
    if (dist < best.distance)
    {
      best = {cx, cy, dist};
    }
  }
  return best;
}

/// Minimum distance from point (px, py) to the edges of a polygon.
static double point_to_polygon_distance(double px,
                                        double py,
                                        const geometry_msgs::msg::Polygon& polygon)
{
  return closest_edge_point(px, py, polygon).distance;
}

void MapServerNode::publish_keepout_mask()
{
  if (areas_.empty())
  {
    return;
  }

  // grid_map: size(0) = cells along X, size(1) = cells along Y.
  //   r=0 → X_max (decreasing), c=0 → Y_max (decreasing).
  // OccupancyGrid: width = X cells, height = Y cells.
  //   col=0 → X_min (at origin.x), row=0 → Y_min (at origin.y).
  // Both flip + swap roles: the OccupancyGrid's (row, col) is the grid_map's
  //   (cols - 1 - c, nx - 1 - r) mapping [mow_progress_to_occupancy_grid
  //   pattern]. Previously this publisher had the dimensions swapped —
  //   width/height set from the wrong grid_map axis — so every cell's
  //   value landed at a 90°-rotated position, marking interior polygon
  //   cells as lethal and breaking Smac planning with "Start occupied".
  const int nx = map_.getSize()(0);  // cells along X
  const int ny = map_.getSize()(1);  // cells along Y
  const float res = static_cast<float>(resolution_);

  nav_msgs::msg::OccupancyGrid mask;
  mask.header.stamp = now();
  mask.header.frame_id = map_frame_;
  mask.info.resolution = res;
  mask.info.width = static_cast<uint32_t>(nx);
  mask.info.height = static_cast<uint32_t>(ny);
  mask.info.origin.position.x = map_.getPosition().x() - map_.getLength().x() * 0.5;
  mask.info.origin.position.y = map_.getPosition().y() - map_.getLength().y() * 0.5;
  mask.info.origin.position.z = 0.0;
  mask.info.origin.orientation.w = 1.0;
  mask.data.resize(static_cast<std::size_t>(nx * ny), 100);  // default: keepout

  // A cell inside ANY area (mowing or navigation) is free (0).
  // A cell outside all areas but within keepout_nav_margin_ of any area
  // polygon edge is also free (0) — this prevents "Start occupied" when
  // the robot is near the boundary.
  // Cells beyond the margin stay 100 (keepout/lethal).
  for (int r = 0; r < nx; ++r)
  {
    for (int c = 0; c < ny; ++c)
    {
      grid_map::Position pos;
      const grid_map::Index idx(r, c);
      if (!map_.getPosition(idx, pos))
      {
        continue;
      }

      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(pos.x());
      pt.y = static_cast<float>(pos.y());
      pt.z = 0.0F;

      const int og_col = nx - 1 - r;  // grid_map r=0 (X_max) → OG col nx-1
      const int og_row = ny - 1 - c;  // grid_map c=0 (Y_max) → OG row ny-1
      const auto flat_idx = static_cast<std::size_t>(og_row * nx + og_col);

      bool inside_any = false;
      double inside_min_edge_dist = std::numeric_limits<double>::max();
      bool within_outside_margin = false;
      for (const auto& area : areas_)
      {
        if (point_in_polygon(pt, area.polygon))
        {
          inside_any = true;
          if (boundary_inner_margin_m_ > 0.0)
          {
            double d = point_to_polygon_distance(static_cast<double>(pt.x),
                                                 static_cast<double>(pt.y),
                                                 area.polygon);
            if (d < inside_min_edge_dist)
            {
              inside_min_edge_dist = d;
            }
          }
          // Keep scanning other polygons — a cell can be inside A but near the
          // edge of B. We want the nearest edge distance overall.
          continue;
        }
        if (!within_outside_margin && keepout_nav_margin_ > 0.0)
        {
          double dist = point_to_polygon_distance(static_cast<double>(pt.x),
                                                  static_cast<double>(pt.y),
                                                  area.polygon);
          if (dist <= keepout_nav_margin_)
          {
            within_outside_margin = true;
          }
        }
      }

      // Shrunk-polygon rule: cells inside a mowing area but within
      // boundary_inner_margin_m_ of the nearest edge become LETHAL in the
      // keepout mask. Effect: the Smac planner never drafts a path that
      // comes within that margin of the polygon edge, giving the FTC
      // controller room to track without spilling over. Combined with
      // inflation_layer, the total soft-wall is ~ margin + inflation_radius.
      bool inner_buffer = inside_any && boundary_inner_margin_m_ > 0.0 &&
                          inside_min_edge_dist < boundary_inner_margin_m_;

      if ((inside_any || within_outside_margin) && !inner_buffer)
      {
        mask.data[flat_idx] = 0;
      }
    }
  }

  // Overlay obstacle polygons: cells inside any obstacle -> 100 (lethal).
  for (int r = 0; r < nx; ++r)
  {
    for (int c = 0; c < ny; ++c)
    {
      grid_map::Position pos;
      const grid_map::Index idx(r, c);
      if (!map_.getPosition(idx, pos))
      {
        continue;
      }

      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(pos.x());
      pt.y = static_cast<float>(pos.y());
      pt.z = 0.0F;

      for (const auto& obs : obstacle_polygons_)
      {
        if (point_in_polygon(pt, obs))
        {
          const int og_col = nx - 1 - r;
          const int og_row = ny - 1 - c;
          mask.data[static_cast<std::size_t>(og_row * nx + og_col)] = 100;
          break;
        }
      }
    }
  }

  // Overlay no-go zones from classification layer.
  const auto& cls = map_[std::string(layers::CLASSIFICATION)];
  const float no_go_val = static_cast<float>(CellType::NO_GO_ZONE);
  for (int r = 0; r < nx; ++r)
  {
    for (int c = 0; c < ny; ++c)
    {
      if (cls(r, c) == no_go_val)
      {
        const int og_col = nx - 1 - r;
        const int og_row = ny - 1 - c;
        mask.data[static_cast<std::size_t>(og_row * nx + og_col)] = 100;
      }
    }
  }

  cached_keepout_mask_ = mask;
  keepout_mask_pub_->publish(mask);

  // Publish filter info only once (transient_local latches it for late
  // subscribers).  Republishing every cycle causes Nav2 KeepoutFilter to
  // re-subscribe to the mask topic each time, blocking the costmap update
  // thread and starving the planner of CPU.
  if (!keepout_filter_info_sent_)
  {
    nav2_msgs::msg::CostmapFilterInfo info;
    info.header.stamp = mask.header.stamp;
    info.header.frame_id = map_frame_;
    info.type = 0;  // KEEPOUT = 0
    info.filter_mask_topic = "/keepout_mask";
    info.base = 0.0F;
    info.multiplier = 1.0F;
    keepout_filter_info_pub_->publish(info);
    keepout_filter_info_sent_ = true;
  }
}

void MapServerNode::publish_speed_mask()
{
  if (areas_.empty())
  {
    return;
  }

  // See publish_keepout_mask for the X/Y→OccupancyGrid convention.
  const int nx = map_.getSize()(0);  // cells along X
  const int ny = map_.getSize()(1);  // cells along Y
  const float res = static_cast<float>(resolution_);

  const double headland_radius = mower_width_;

  nav_msgs::msg::OccupancyGrid mask;
  mask.header.stamp = now();
  mask.header.frame_id = map_frame_;
  mask.info.resolution = res;
  mask.info.width = static_cast<uint32_t>(nx);
  mask.info.height = static_cast<uint32_t>(ny);
  mask.info.origin.position.x = map_.getPosition().x() - map_.getLength().x() * 0.5;
  mask.info.origin.position.y = map_.getPosition().y() - map_.getLength().y() * 0.5;
  mask.info.origin.position.z = 0.0;
  mask.info.origin.orientation.w = 1.0;
  mask.data.resize(static_cast<std::size_t>(nx * ny), 0);  // default: full speed

  for (const auto& area : areas_)
  {
    const auto& pts = area.polygon.points;
    const std::size_t n = pts.size();
    if (n < 3)
      continue;

    for (int r = 0; r < nx; ++r)
    {
      for (int c = 0; c < ny; ++c)
      {
        grid_map::Position pos;
        const grid_map::Index idx(r, c);
        if (!map_.getPosition(idx, pos))
        {
          continue;
        }

        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(pos.x());
        pt.y = static_cast<float>(pos.y());
        pt.z = 0.0F;

        if (!point_in_polygon(pt, area.polygon))
        {
          continue;
        }

        double min_dist_sq = std::numeric_limits<double>::max();

        for (std::size_t i = 0, j = n - 1; i < n; j = i++)
        {
          const double ax = static_cast<double>(pts[j].x);
          const double ay = static_cast<double>(pts[j].y);
          const double bx = static_cast<double>(pts[i].x);
          const double by = static_cast<double>(pts[i].y);

          const double dx = bx - ax;
          const double dy = by - ay;
          const double len_sq = dx * dx + dy * dy;

          double dist_sq = 0.0;
          if (len_sq < 1e-12)
          {
            const double ex = pos.x() - ax;
            const double ey = pos.y() - ay;
            dist_sq = ex * ex + ey * ey;
          }
          else
          {
            const double t =
                std::clamp(((pos.x() - ax) * dx + (pos.y() - ay) * dy) / len_sq, 0.0, 1.0);
            const double proj_x = ax + t * dx - pos.x();
            const double proj_y = ay + t * dy - pos.y();
            dist_sq = proj_x * proj_x + proj_y * proj_y;
          }

          if (dist_sq < min_dist_sq)
          {
            min_dist_sq = dist_sq;
          }
        }

        if (min_dist_sq <= headland_radius * headland_radius)
        {
          const int og_col = nx - 1 - r;
          const int og_row = ny - 1 - c;
          mask.data[static_cast<std::size_t>(og_row * nx + og_col)] = 50;
        }
      }
    }
  }

  cached_speed_mask_ = mask;
  speed_mask_pub_->publish(mask);

  if (!speed_filter_info_sent_)
  {
    nav2_msgs::msg::CostmapFilterInfo info;
    info.header.stamp = mask.header.stamp;
    info.header.frame_id = map_frame_;
    info.type = 1;  // SPEED_LIMIT = 1 (percentage mode)
    info.filter_mask_topic = "/speed_mask";
    info.base = 100.0F;
    info.multiplier = -1.0F;
    speed_filter_info_pub_->publish(info);
    speed_filter_info_sent_ = true;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Boundary monitoring
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::check_boundary_violation(double x, double y)
{
  if (areas_.empty())
  {
    return;
  }

  geometry_msgs::msg::Point32 pt;
  pt.x = static_cast<float>(x);
  pt.y = static_cast<float>(y);
  pt.z = 0.0F;

  bool inside_any = false;
  double min_edge_dist = std::numeric_limits<double>::max();
  for (const auto& area : areas_)
  {
    if (point_in_polygon(pt, area.polygon))
    {
      inside_any = true;
      break;
    }
    // Only track distance-to-edge for areas we're outside of; used to
    // classify the violation as "soft" (still recoverable) vs "lethal"
    // (blade/motor hazard — stop immediately).
    const double d = point_to_polygon_distance(x, y, area.polygon);
    if (d < min_edge_dist)
    {
      min_edge_dist = d;
    }
  }

  std_msgs::msg::Bool soft_msg;
  soft_msg.data = !inside_any;
  boundary_violation_pub_->publish(soft_msg);

  std_msgs::msg::Bool lethal_msg;
  lethal_msg.data = !inside_any && (min_edge_dist > lethal_boundary_margin_m_);
  lethal_boundary_violation_pub_->publish(lethal_msg);

  if (!inside_any)
  {
    if (lethal_msg.data)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "LETHAL BOUNDARY VIOLATION: robot at (%.2f, %.2f) — %.2fm outside "
          "nearest allowed area (margin=%.2fm)",
          x, y, min_edge_dist, lethal_boundary_margin_m_);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "BOUNDARY VIOLATION: robot at (%.2f, %.2f) — %.2fm outside nearest "
          "allowed area (lethal at %.2fm)",
          x, y, min_edge_dist, lethal_boundary_margin_m_);
    }
  }
}

void MapServerNode::on_get_recovery_point(
    const mowgli_interfaces::srv::GetRecoveryPoint::Request::SharedPtr /*req*/,
    mowgli_interfaces::srv::GetRecoveryPoint::Response::SharedPtr res)
{
  res->success = false;
  res->distance_outside = 0.0;

  if (areas_.empty())
  {
    res->message = "no areas defined";
    return;
  }

  // Look up current robot pose in the map frame. Same path as
  // check_boundary_violation — the BT only invokes this service when a
  // violation is latched, so TF should be fresh.
  double rx = 0.0;
  double ry = 0.0;
  if (!tf_buffer_)
  {
    res->message = "tf buffer unavailable";
    return;
  }
  try
  {
    auto tf = tf_buffer_->lookupTransform(map_frame_, "base_link", tf2::TimePointZero);
    rx = tf.transform.translation.x;
    ry = tf.transform.translation.y;
  }
  catch (const tf2::TransformException& ex)
  {
    res->message = std::string("tf lookup failed: ") + ex.what();
    return;
  }

  // Already inside an area? No recovery needed.
  geometry_msgs::msg::Point32 robot_pt;
  robot_pt.x = static_cast<float>(rx);
  robot_pt.y = static_cast<float>(ry);
  robot_pt.z = 0.0F;
  for (const auto& area : areas_)
  {
    if (point_in_polygon(robot_pt, area.polygon))
    {
      res->message = "already inside a mowing area";
      // Still return the current pose as a safe recovery — callers can
      // ignore if success=false.
      res->recovery_pose.position.x = rx;
      res->recovery_pose.position.y = ry;
      res->recovery_pose.orientation.w = 1.0;
      return;
    }
  }

  // Find the globally-closest edge point across all area polygons.
  ClosestEdge best;
  for (const auto& area : areas_)
  {
    auto cand = closest_edge_point(rx, ry, area.polygon);
    if (cand.distance < best.distance)
    {
      best = cand;
    }
  }

  if (best.distance == std::numeric_limits<double>::max())
  {
    res->message = "no polygon edges found";
    return;
  }

  // Inward direction: from robot toward the closest edge, continuing past
  // the edge into the polygon interior.
  const double vx = best.x - rx;
  const double vy = best.y - ry;
  const double vlen = std::hypot(vx, vy);
  double nx = 0.0;
  double ny = 0.0;
  if (vlen > 1e-6)
  {
    nx = vx / vlen;
    ny = vy / vlen;
  }

  const double tx = best.x + boundary_recovery_offset_m_ * nx;
  const double ty = best.y + boundary_recovery_offset_m_ * ny;

  // Yaw facing inward — same direction as the offset.
  const double yaw = std::atan2(ny, nx);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  res->recovery_pose.position.x = tx;
  res->recovery_pose.position.y = ty;
  res->recovery_pose.position.z = 0.0;
  res->recovery_pose.orientation.x = 0.0;
  res->recovery_pose.orientation.y = 0.0;
  res->recovery_pose.orientation.z = sy;
  res->recovery_pose.orientation.w = cy;
  res->distance_outside = best.distance;
  res->success = true;
  res->message = "recovery pose computed";

  RCLCPP_INFO(get_logger(),
              "GetRecoveryPoint: robot=(%.2f, %.2f) outside by %.2fm → "
              "target=(%.2f, %.2f) yaw=%.2f",
              rx, ry, best.distance, tx, ty, yaw);
}

// ─────────────────────────────────────────────────────────────────────────────
// Obstacle diff and replan triggering
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::diff_and_update_obstacles(
    const std::vector<mowgli_interfaces::msg::TrackedObstacle>& incoming)
{
  // Always update obstacle polygons for costmap/keepout.
  obstacle_polygons_.clear();
  for (const auto& obs : incoming)
  {
    if (obs.polygon.points.size() >= 3)
    {
      obstacle_polygons_.push_back(obs.polygon);
    }
  }

  // Update classification layer with obstacle cells.
  // First clear previous obstacle marks (reset to UNKNOWN), then re-mark.
  auto& cls = map_[std::string(layers::CLASSIFICATION)];
  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
  {
    auto val = static_cast<CellType>(static_cast<int>(cls((*it)(0), (*it)(1))));
    if (val == CellType::OBSTACLE_PERMANENT || val == CellType::OBSTACLE_TEMPORARY)
    {
      cls((*it)(0), (*it)(1)) = static_cast<float>(CellType::UNKNOWN);
    }
  }
  // Mark obstacle cells from tracked obstacles
  for (const auto& obs : incoming)
  {
    if (obs.polygon.points.size() < 3)
      continue;
    auto cell_val = (obs.status == mowgli_interfaces::msg::TrackedObstacle::PERSISTENT)
                        ? static_cast<float>(CellType::OBSTACLE_PERMANENT)
                        : static_cast<float>(CellType::OBSTACLE_TEMPORARY);
    grid_map::Polygon gm_poly;
    for (const auto& pt : obs.polygon.points)
    {
      gm_poly.addVertex(grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
    }
    for (grid_map::PolygonIterator pit(map_, gm_poly); !pit.isPastEnd(); ++pit)
    {
      cls((*pit)(0), (*pit)(1)) = cell_val;
    }
  }

  // Check for new persistent obstacles not yet planned around.
  // Only persistent obstacles with stable IDs trigger replanning.
  bool has_new_persistent = false;
  std::set<uint32_t> current_persistent_ids;
  for (const auto& obs : incoming)
  {
    if (obs.status == mowgli_interfaces::msg::TrackedObstacle::PERSISTENT)
    {
      current_persistent_ids.insert(obs.id);
      if (planned_obstacle_ids_.find(obs.id) == planned_obstacle_ids_.end())
      {
        has_new_persistent = true;
        RCLCPP_INFO(get_logger(),
                    "New persistent obstacle #%u detected (not yet planned around)",
                    obs.id);
      }
    }
  }

  const std::size_t incoming_count = current_persistent_ids.size();
  if (incoming_count != last_obstacle_count_)
  {
    RCLCPP_INFO(get_logger(),
                "Obstacle change detected: %zu -> %zu persistent obstacles",
                last_obstacle_count_,
                incoming_count);
    last_obstacle_count_ = incoming_count;
    masks_dirty_ = true;
  }

  // Handle deferred replans.
  if (!has_new_persistent)
  {
    if (replan_pending_ && (now() - last_replan_time_).seconds() >= replan_cooldown_sec_)
    {
      replan_pending_ = false;
      planned_obstacle_ids_ = current_persistent_ids;
      std_msgs::msg::Bool msg;
      msg.data = true;
      replan_needed_pub_->publish(msg);
      last_replan_time_ = now();
      RCLCPP_INFO(get_logger(), "Deferred replan triggered (cooldown expired)");
    }
    return;
  }

  // New persistent obstacle — trigger or defer replan.
  if ((now() - last_replan_time_).seconds() < replan_cooldown_sec_)
  {
    replan_pending_ = true;
    RCLCPP_INFO(get_logger(),
                "Replan deferred (cooldown %.0fs, remaining %.0fs)",
                replan_cooldown_sec_,
                replan_cooldown_sec_ - (now() - last_replan_time_).seconds());
    return;
  }

  replan_pending_ = false;
  planned_obstacle_ids_ = current_persistent_ids;
  std_msgs::msg::Bool msg;
  msg.data = true;
  replan_needed_pub_->publish(msg);
  last_replan_time_ = now();
  RCLCPP_INFO(get_logger(),
              "Replan triggered: %zu new persistent obstacles",
              current_persistent_ids.size() -
                  (planned_obstacle_ids_.size() - current_persistent_ids.size()));
}

// ─────────────────────────────────────────────────────────────────────────────
// Docking point service
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_set_docking_point(
    const mowgli_interfaces::srv::SetDockingPoint::Request::SharedPtr req,
    mowgli_interfaces::srv::SetDockingPoint::Response::SharedPtr res)
{
  docking_pose_ = req->docking_pose;
  docking_pose_set_ = true;

  // Publish the docking pose for other nodes (e.g., behavior tree).
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = now();
  pose_msg.header.frame_id = map_frame_;
  pose_msg.pose = docking_pose_;
  docking_pose_pub_->publish(pose_msg);

  RCLCPP_INFO(get_logger(),
              "Docking point set: (%.3f, %.3f, %.3f) orientation (%.3f, %.3f, %.3f, %.3f)",
              docking_pose_.position.x,
              docking_pose_.position.y,
              docking_pose_.position.z,
              docking_pose_.orientation.x,
              docking_pose_.orientation.y,
              docking_pose_.orientation.z,
              docking_pose_.orientation.w);

  // Auto-save if persistence path is set.
  if (!areas_file_path_.empty())
  {
    try
    {
      save_areas_to_file(areas_file_path_);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(get_logger(), "Auto-save after docking point change failed: %s", ex.what());
    }
  }

  res->success = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Area persistence services
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_save_areas(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (areas_file_path_.empty())
  {
    res->success = false;
    res->message = "areas_file_path parameter is empty; cannot save.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try
  {
    save_areas_to_file(areas_file_path_);
    res->success = true;
    res->message = "Areas saved to " + areas_file_path_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("Save failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void MapServerNode::on_load_areas(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (areas_file_path_.empty())
  {
    res->success = false;
    res->message = "areas_file_path parameter is empty; cannot load.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try
  {
    load_areas_from_file(areas_file_path_);
    apply_area_classifications();
    res->success = true;
    res->message = "Areas loaded from " + areas_file_path_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("Load failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Area persistence helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string MapServerNode::polygon_to_string(const geometry_msgs::msg::Polygon& poly)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < poly.points.size(); ++i)
  {
    if (i > 0)
    {
      oss << ";";
    }
    oss << poly.points[i].x << "," << poly.points[i].y;
  }
  return oss.str();
}

void MapServerNode::save_areas_to_file(const std::string& path)
{
  std::ofstream out(path);
  if (!out.is_open())
  {
    throw std::runtime_error("Cannot open " + path + " for writing");
  }

  out << "# Mowgli ROS2 — Persisted areas and docking point\n";
  out << "# Auto-generated by map_server_node. Do not edit manually.\n\n";

  out << "area_count: " << areas_.size() << "\n\n";

  for (std::size_t i = 0; i < areas_.size(); ++i)
  {
    const auto& area = areas_[i];
    out << "area_" << i << "_name: " << area.name << "\n";
    out << "area_" << i << "_polygon: " << polygon_to_string(area.polygon) << "\n";
    out << "area_" << i << "_is_navigation: " << (area.is_navigation_area ? 1 : 0) << "\n";
    out << "area_" << i << "_obstacle_count: " << area.obstacles.size() << "\n";
    for (std::size_t j = 0; j < area.obstacles.size(); ++j)
    {
      out << "area_" << i << "_obstacle_" << j << ": " << polygon_to_string(area.obstacles[j])
          << "\n";
    }
    out << "\n";
  }

  out << "docking_pose_set: " << (docking_pose_set_ ? 1 : 0) << "\n";
  if (docking_pose_set_)
  {
    out << "dock_x: " << docking_pose_.position.x << "\n";
    out << "dock_y: " << docking_pose_.position.y << "\n";
    out << "dock_z: " << docking_pose_.position.z << "\n";
    out << "dock_qx: " << docking_pose_.orientation.x << "\n";
    out << "dock_qy: " << docking_pose_.orientation.y << "\n";
    out << "dock_qz: " << docking_pose_.orientation.z << "\n";
    out << "dock_qw: " << docking_pose_.orientation.w << "\n";
  }

  out.close();
}

void MapServerNode::load_areas_from_file(const std::string& path)
{
  std::ifstream in(path);
  if (!in.is_open())
  {
    throw std::runtime_error("Cannot open " + path);
  }

  // Parse all key-value pairs into a map.
  std::map<std::string, std::string> kv;
  std::string line;
  while (std::getline(in, line))
  {
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos)
    {
      continue;
    }
    std::string key = line.substr(0, colon_pos);
    std::string val = line.substr(colon_pos + 1);
    // Trim leading whitespace from value.
    auto start = val.find_first_not_of(" \t");
    if (start != std::string::npos)
    {
      val = val.substr(start);
    }
    else
    {
      val.clear();
    }
    kv[key] = val;
  }
  in.close();

  auto get_int = [&](const std::string& key, int def) -> int
  {
    auto it = kv.find(key);
    return (it != kv.end()) ? std::stoi(it->second) : def;
  };

  auto get_double = [&](const std::string& key, double def) -> double
  {
    auto it = kv.find(key);
    return (it != kv.end()) ? std::stod(it->second) : def;
  };

  auto get_str = [&](const std::string& key) -> std::string
  {
    auto it = kv.find(key);
    return (it != kv.end()) ? it->second : std::string{};
  };

  // Clear existing areas and reload from file.
  areas_.clear();
  obstacle_polygons_.clear();

  const int area_count = get_int("area_count", 0);
  for (int i = 0; i < area_count; ++i)
  {
    const std::string prefix = "area_" + std::to_string(i);
    AreaEntry entry;
    entry.name = get_str(prefix + "_name");
    entry.polygon = parse_polygon_string(get_str(prefix + "_polygon"));
    entry.is_navigation_area = (get_int(prefix + "_is_navigation", 0) != 0);

    const int obs_count = get_int(prefix + "_obstacle_count", 0);
    for (int j = 0; j < obs_count; ++j)
    {
      auto obs_poly = parse_polygon_string(get_str(prefix + "_obstacle_" + std::to_string(j)));
      if (obs_poly.points.size() >= 3)
      {
        entry.obstacles.push_back(obs_poly);
      }
    }

    if (entry.polygon.points.size() >= 3)
    {
      RCLCPP_INFO(get_logger(),
                  "Loaded area '%s': %zu vertices, %s, %zu obstacles",
                  entry.name.c_str(),
                  entry.polygon.points.size(),
                  entry.is_navigation_area ? "navigation" : "mowing",
                  entry.obstacles.size());
      areas_.push_back(std::move(entry));
    }
  }

  // Load docking point.
  docking_pose_set_ = (get_int("docking_pose_set", 0) != 0);
  if (docking_pose_set_)
  {
    docking_pose_.position.x = get_double("dock_x", 0.0);
    docking_pose_.position.y = get_double("dock_y", 0.0);
    docking_pose_.position.z = get_double("dock_z", 0.0);
    docking_pose_.orientation.x = get_double("dock_qx", 0.0);
    docking_pose_.orientation.y = get_double("dock_qy", 0.0);
    docking_pose_.orientation.z = get_double("dock_qz", 0.0);
    docking_pose_.orientation.w = get_double("dock_qw", 1.0);

    RCLCPP_INFO(get_logger(),
                "Loaded docking point: (%.3f, %.3f)",
                docking_pose_.position.x,
                docking_pose_.position.y);
  }

  // Resize map to fit new areas and reset masks.
  resize_map_to_areas();
  keepout_filter_info_sent_ = false;
  speed_filter_info_sent_ = false;
  masks_dirty_ = true;
}

void MapServerNode::apply_area_classifications()
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  const float lawn_val = static_cast<float>(CellType::LAWN);
  const float no_go_val = static_cast<float>(CellType::NO_GO_ZONE);

  for (const auto& area : areas_)
  {
    grid_map::Polygon gm_polygon;
    for (const auto& pt : area.polygon.points)
    {
      gm_polygon.addVertex(
          grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
    }

    // Mowing areas are LAWN, not NO_GO_ZONE.
    for (grid_map::PolygonIterator it(map_, gm_polygon); !it.isPastEnd(); ++it)
    {
      map_.at(std::string(layers::CLASSIFICATION), *it) = lawn_val;
    }

    for (const auto& obstacle : area.obstacles)
    {
      grid_map::Polygon obs_gm;
      for (const auto& pt : obstacle.points)
      {
        obs_gm.addVertex(grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
      }
      for (grid_map::PolygonIterator it(map_, obs_gm); !it.isPastEnd(); ++it)
      {
        map_.at(std::string(layers::CLASSIFICATION), *it) = no_go_val;
      }
    }
  }

  // Mark dock exclusion zone as NO_GO_ZONE — no mowing strips planned here.
  if (has_dock_exclusion_ && dock_exclusion_polygon_.points.size() >= 3)
  {
    grid_map::Polygon dock_gm;
    for (const auto& pt : dock_exclusion_polygon_.points)
    {
      dock_gm.addVertex(grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
    }
    for (grid_map::PolygonIterator it(map_, dock_gm); !it.isPastEnd(); ++it)
    {
      map_.at(std::string(layers::CLASSIFICATION), *it) = no_go_val;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test-support methods
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::tick_once(double elapsed_seconds)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  apply_decay(elapsed_seconds);
}

void MapServerNode::mark_mowed(double x, double y)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  mark_cells_mowed(x, y);
}

// ─────────────────────────────────────────────────────────────────────────────
// Strip planner
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::ensure_strip_layout(size_t area_index)
{
  if (area_index >= areas_.size())
    return;

  // Grow cache if needed
  if (strip_layouts_.size() <= area_index)
    strip_layouts_.resize(area_index + 1);

  auto& layout = strip_layouts_[area_index];
  if (layout.valid)
    return;

  const auto& area = areas_[area_index];
  const auto& poly = area.polygon;
  if (poly.points.size() < 3)
    return;

  // Compute bounding box and optimal mow angle (MBB = longest edge direction)
  double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
  for (const auto& p : poly.points)
  {
    min_x = std::min(min_x, static_cast<double>(p.x));
    max_x = std::max(max_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    max_y = std::max(max_y, static_cast<double>(p.y));
  }

  layout.mow_angle = 0.0;

  // Inset boundary by strip_boundary_margin_m_. Previously used
  // mower_width_ (~0.18 m), but real FTC tracking error during transit
  // exceeds 0.5 m — strip endpoints that close to the polygon edge get
  // overshot and trigger boundary violations. A larger margin sacrifices
  // a small strip of un-mowed perimeter for reliable inside-polygon
  // coverage paths.
  double inset = strip_boundary_margin_m_;
  double inner_min_x = min_x + inset;
  double inner_max_x = max_x - inset;

  if (inner_min_x >= inner_max_x)
  {
    layout.valid = true;
    return;
  }

  // Generate strips clipped to the actual polygon boundary.
  // For each vertical scan line at x=const, find Y intersections with polygon edges.
  layout.strips.clear();
  int col = 0;
  int n_pts = static_cast<int>(poly.points.size());

  for (double x = inner_min_x + mower_width_ / 2; x <= inner_max_x; x += mower_width_)
  {
    // Find Y intersections of vertical line x=const with polygon edges
    std::vector<double> y_intersections;
    for (int i = 0; i < n_pts; ++i)
    {
      int j = (i + 1) % n_pts;
      double x1 = static_cast<double>(poly.points[i].x);
      double y1 = static_cast<double>(poly.points[i].y);
      double x2 = static_cast<double>(poly.points[j].x);
      double y2 = static_cast<double>(poly.points[j].y);

      if ((x1 < x && x2 >= x) || (x2 < x && x1 >= x))
      {
        double t = (x - x1) / (x2 - x1);
        y_intersections.push_back(y1 + t * (y2 - y1));
      }
    }

    if (y_intersections.size() < 2)
      continue;

    std::sort(y_intersections.begin(), y_intersections.end());

    // Take the first pair as the interior interval (even-odd fill)
    // Inset by mower_width/2 to stay inside boundary
    double y_lo = y_intersections.front() + inset;
    double y_hi = y_intersections.back() - inset;

    if (y_hi - y_lo < mower_width_)
      continue;

    Strip strip;
    strip.start.x = x;
    strip.start.y = y_lo;
    strip.start.z = 0.0;
    strip.end.x = x;
    strip.end.y = y_hi;
    strip.end.z = 0.0;
    strip.column_index = col++;
    layout.strips.push_back(strip);
  }

  layout.valid = true;
  RCLCPP_INFO(get_logger(),
              "Strip layout for area '%s': %zu strips, mow_angle=%.1f°, "
              "bbox=(%.2f,%.2f)-(%.2f,%.2f), inner_x=[%.2f, %.2f]",
              area.name.c_str(),
              layout.strips.size(),
              layout.mow_angle * 180.0 / M_PI,
              min_x,
              min_y,
              max_x,
              max_y,
              inner_min_x,
              inner_max_x);
  if (!layout.strips.empty())
  {
    const auto& first = layout.strips.front();
    const auto& last = layout.strips.back();
    RCLCPP_INFO(get_logger(),
                "  First strip: x=%.2f y=[%.2f, %.2f], Last strip: x=%.2f y=[%.2f, %.2f]",
                first.start.x,
                first.start.y,
                first.end.y,
                last.start.x,
                last.start.y,
                last.end.y);
  }
}

bool MapServerNode::is_strip_mowed(const Strip& strip, double threshold_pct) const
{
  // Sample mow_progress along the strip centerline
  double dx = strip.end.x - strip.start.x;
  double dy = strip.end.y - strip.start.y;
  double length = std::hypot(dx, dy);
  if (length < resolution_)
    return true;

  int samples = std::max(3, static_cast<int>(length / resolution_));
  int mowed_count = 0;
  int total_count = 0;

  const auto& progress_layer = map_[std::string(layers::MOW_PROGRESS)];
  const auto& class_layer = map_[std::string(layers::CLASSIFICATION)];

  for (int i = 0; i <= samples; ++i)
  {
    double t = static_cast<double>(i) / samples;
    double px = strip.start.x + t * dx;
    double py = strip.start.y + t * dy;

    grid_map::Position pos(px, py);
    if (!map_.isInside(pos))
      continue;

    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
      continue;

    auto cell_type = static_cast<CellType>(static_cast<int>(class_layer(idx(0), idx(1))));
    if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY)
      continue;  // Skip obstacle cells

    // Check if inside the mowing area polygon
    geometry_msgs::msg::Point32 pt32;
    pt32.x = static_cast<float>(px);
    pt32.y = static_cast<float>(py);

    total_count++;
    if (progress_layer(idx(0), idx(1)) >= 0.3f)
      mowed_count++;
  }

  if (total_count == 0)
    return true;  // No cells to mow

  return static_cast<double>(mowed_count) / total_count >= threshold_pct;
}

bool MapServerNode::is_strip_blocked(const Strip& strip, double blocked_threshold) const
{
  // Check if a strip is mostly blocked by obstacles, making it unreachable.
  // A strip with >blocked_threshold fraction of obstacle cells is "frontier".
  double dx = strip.end.x - strip.start.x;
  double dy = strip.end.y - strip.start.y;
  double length = std::hypot(dx, dy);
  if (length < resolution_)
    return false;

  int samples = std::max(3, static_cast<int>(length / resolution_));
  int obstacle_count = 0;
  int total_count = 0;

  const auto& class_layer = map_[std::string(layers::CLASSIFICATION)];

  for (int i = 0; i <= samples; ++i)
  {
    double t = static_cast<double>(i) / samples;
    double px = strip.start.x + t * dx;
    double py = strip.start.y + t * dy;

    grid_map::Position pos(px, py);
    if (!map_.isInside(pos))
      continue;

    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
      continue;

    total_count++;
    auto cell_type = static_cast<CellType>(static_cast<int>(class_layer(idx(0), idx(1))));
    if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY)
      obstacle_count++;
  }

  if (total_count == 0)
    return false;

  return static_cast<double>(obstacle_count) / total_count >= blocked_threshold;
}

bool MapServerNode::find_next_unmowed_strip(
    size_t area_index, double robot_x, double robot_y, Strip& out_strip, bool /*prefer_headland*/)
{
  ensure_strip_layout(area_index);

  if (area_index >= strip_layouts_.size() || !strip_layouts_[area_index].valid)
    return false;

  const auto& layout = strip_layouts_[area_index];
  int n = static_cast<int>(layout.strips.size());
  if (n == 0)
    return false;

  // Grow tracking vector if needed
  if (current_strip_idx_.size() <= area_index)
    current_strip_idx_.resize(area_index + 1, -1);

  int& cur_idx = current_strip_idx_[area_index];

  // First call: start from the strip nearest to the robot
  if (cur_idx < 0)
  {
    double nearest_dist = 1e9;
    for (int i = 0; i < n; ++i)
    {
      double mid_x = (layout.strips[i].start.x + layout.strips[i].end.x) / 2;
      double d = std::abs(mid_x - robot_x);
      if (d < nearest_dist)
      {
        nearest_dist = d;
        cur_idx = i;
      }
    }
  }
  else
  {
    // Advance to next strip (sequential boustrophedon)
    cur_idx++;
  }

  // Search forward from current index for the next unmowed strip.
  // Skip strips that are blocked by obstacles (>50% obstacle cells) — these
  // are treated as "frontier" strips that can't be mowed.
  for (int i = 0; i < n; ++i)
  {
    int idx = (cur_idx + i) % n;
    const auto& strip = layout.strips[idx];
    if (!is_strip_mowed(strip) && !is_strip_blocked(strip))
    {
      cur_idx = idx;
      out_strip = strip;

      // Boustrophedon: alternate Y direction per column
      if (idx % 2 == 1)
        std::swap(out_strip.start, out_strip.end);

      return true;
    }
  }

  return false;  // All strips mowed or blocked
}

nav_msgs::msg::Path MapServerNode::strip_to_path(const Strip& strip, size_t /*area_index*/) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = now();

  double dx = strip.end.x - strip.start.x;
  double dy = strip.end.y - strip.start.y;
  double length = std::hypot(dx, dy);
  double yaw = std::atan2(dy, dx);

  // Quaternion from yaw
  double cy = std::cos(yaw / 2);
  double sy = std::sin(yaw / 2);

  int n_poses = std::max(2, static_cast<int>(length / resolution_) + 1);

  const auto& class_layer = map_[std::string(layers::CLASSIFICATION)];

  for (int i = 0; i < n_poses; ++i)
  {
    double t = static_cast<double>(i) / (n_poses - 1);
    double px = strip.start.x + t * dx;
    double py = strip.start.y + t * dy;

    // Skip cells inside obstacles
    grid_map::Position pos(px, py);
    if (map_.isInside(pos))
    {
      grid_map::Index idx;
      if (map_.getIndex(pos, idx))
      {
        auto cell_type = static_cast<CellType>(static_cast<int>(class_layer(idx(0), idx(1))));
        if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY)
          continue;
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = px;
    pose.pose.position.y = py;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = cy;
    pose.pose.orientation.z = sy;
    path.poses.push_back(pose);
  }

  return path;
}

void MapServerNode::compute_coverage_stats(size_t area_index,
                                           uint32_t& total,
                                           uint32_t& mowed,
                                           uint32_t& obstacle_cells) const
{
  total = 0;
  mowed = 0;
  obstacle_cells = 0;

  if (area_index >= areas_.size())
    return;

  const auto& area = areas_[area_index];
  const auto& progress_layer = map_[std::string(layers::MOW_PROGRESS)];
  const auto& class_layer = map_[std::string(layers::CLASSIFICATION)];

  // Iterate all cells and check if inside the area polygon
  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position pos;
    map_.getPosition(*it, pos);

    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(pos.x());
    pt.y = static_cast<float>(pos.y());

    if (!point_in_polygon(pt, area.polygon))
      continue;

    auto cell_type = static_cast<CellType>(static_cast<int>(class_layer((*it)(0), (*it)(1))));
    if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY)
    {
      obstacle_cells++;
      continue;
    }

    total++;
    if (progress_layer((*it)(0), (*it)(1)) >= 0.3f)
      mowed++;
  }
}

void MapServerNode::on_get_next_strip(
    const mowgli_interfaces::srv::GetNextStrip::Request::SharedPtr req,
    mowgli_interfaces::srv::GetNextStrip::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_index >= areas_.size())
  {
    res->success = false;
    res->coverage_complete = false;
    return;
  }

  Strip strip;
  if (!find_next_unmowed_strip(
          req->area_index, req->robot_x, req->robot_y, strip, req->prefer_headland))
  {
    // All strips mowed
    res->success = true;
    res->coverage_complete = true;
    res->coverage_percent = 100.0f;
    res->strips_remaining = 0;
    res->phase = "complete";
    return;
  }

  res->strip_path = strip_to_path(strip, req->area_index);
  res->success = !res->strip_path.poses.empty();
  res->coverage_complete = false;
  res->phase = "interior";

  // Transit goal = first pose of the strip
  if (!res->strip_path.poses.empty())
  {
    res->transit_goal = res->strip_path.poses.front();
  }

  // Coverage stats
  uint32_t total = 0, mowed_cells = 0, obs = 0;
  compute_coverage_stats(req->area_index, total, mowed_cells, obs);
  res->coverage_percent = total > 0 ? 100.0f * mowed_cells / total : 0.0f;

  // Count remaining strips
  uint32_t remaining = 0;
  if (req->area_index < strip_layouts_.size())
  {
    for (const auto& s : strip_layouts_[req->area_index].strips)
    {
      if (!is_strip_mowed(s))
        remaining++;
    }
  }
  res->strips_remaining = remaining;

  RCLCPP_INFO(get_logger(),
              "GetNextStrip: col=%d, %.1f%% coverage, %u strips remaining",
              strip.column_index,
              res->coverage_percent,
              remaining);
}

void MapServerNode::on_get_coverage_status(
    const mowgli_interfaces::srv::GetCoverageStatus::Request::SharedPtr req,
    mowgli_interfaces::srv::GetCoverageStatus::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_index >= areas_.size())
  {
    res->success = false;
    return;
  }

  compute_coverage_stats(req->area_index, res->total_cells, res->mowed_cells, res->obstacle_cells);
  res->coverage_percent =
      res->total_cells > 0 ? 100.0f * res->mowed_cells / res->total_cells : 0.0f;

  // Count remaining strips
  ensure_strip_layout(req->area_index);
  res->strips_remaining = 0;
  if (req->area_index < strip_layouts_.size())
  {
    for (const auto& s : strip_layouts_[req->area_index].strips)
    {
      if (!is_strip_mowed(s))
        res->strips_remaining++;
    }
  }

  res->success = true;
}

}  // namespace mowgli_map
