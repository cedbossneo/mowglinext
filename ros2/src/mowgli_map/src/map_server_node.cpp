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
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
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

// Path to the runtime mowgli_robot.yaml — bind-mounted into the container
// so writes survive across redeploys. mowgli_robot.yaml is the single
// source of truth for dock_pose_x/y/yaw; this helper rewrites the three
// scalar values in place via per-line substring splicing so comments
// and surrounding structure are preserved (yaml-cpp would round-trip
// and strip them).
constexpr const char* kRuntimeRobotYaml = "/ros2_ws/config/mowgli_robot.yaml";

// Splice a new numeric value into a "<indent><key>:<spaces><number><rest>"
// line, anchored on the indent so a key whose name happens to contain ours
// (e.g. dock_pose_x_offset) is not matched.
inline void splice_yaml_scalar(std::string& content,
                               const std::string& key,
                               const std::string& new_value)
{
  size_t scan = 0;
  while (scan < content.size())
  {
    const size_t line_start = scan;
    size_t cursor = line_start;
    while (cursor < content.size() && (content[cursor] == ' ' || content[cursor] == '\t'))
      ++cursor;
    const size_t indent_end = cursor;
    if (indent_end > line_start && cursor + key.size() < content.size() &&
        content.compare(cursor, key.size(), key) == 0 && content[cursor + key.size()] == ':')
    {
      cursor += key.size() + 1;
      while (cursor < content.size() && (content[cursor] == ' ' || content[cursor] == '\t'))
        ++cursor;
      const size_t val_start = cursor;
      while (cursor < content.size())
      {
        const char c = content[cursor];
        const bool is_num =
            (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
        if (!is_num)
          break;
        ++cursor;
      }
      if (cursor > val_start)
      {
        content.replace(val_start, cursor - val_start, new_value);
        return;
      }
    }
    const size_t nl = content.find('\n', line_start);
    if (nl == std::string::npos)
      break;
    scan = nl + 1;
  }
}

inline bool update_dock_pose_in_robot_yaml(const std::string& path,
                                           double x,
                                           double y,
                                           double yaw_rad)
{
  std::ifstream in(path);
  if (!in.good())
    return false;
  std::stringstream buf;
  buf << in.rdbuf();
  std::string content = buf.str();
  in.close();

  auto fmt = [](double v)
  {
    std::ostringstream s;
    s << std::fixed << std::setprecision(6) << v;
    return s.str();
  };
  splice_yaml_scalar(content, "dock_pose_x", fmt(x));
  splice_yaml_scalar(content, "dock_pose_y", fmt(y));
  splice_yaml_scalar(content, "dock_pose_yaw", fmt(yaw_rad));

  const std::string tmp_path = path + ".tmp";
  {
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out.good())
      return false;
    out << content;
    if (!out.good())
      return false;
  }
  std::error_code ec;
  std::filesystem::rename(tmp_path, path, ec);
  return !ec;
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
  // Path C — fail-count tuning. See header for semantics.
  dead_promote_threshold_ = declare_parameter<double>("dead_promote_threshold", 3.0);
  dead_decay_rate_per_hour_ = declare_parameter<double>("dead_decay_rate_per_hour", 0.5);
  dead_unblock_threshold_ = declare_parameter<double>("dead_unblock_threshold", 1.0);
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
  // Soft boundary deadband: distance the robot must be outside ANY area
  // before /boundary_violation fires. Without this, RTK noise (~3 mm)
  // and FTC tracking error around strip endpoints (which sit
  // strip_boundary_margin_m_ inside the polygon) triggers recovery the
  // moment the robot grazes the edge, producing endless transit/abort
  // recovery loops with 30-60 s gaps between strips.
  soft_boundary_margin_m_ = declare_parameter<double>("soft_boundary_margin_m", 0.10);
  boundary_recovery_offset_m_ = declare_parameter<double>("boundary_recovery_offset_m", 0.8);
  boundary_inner_margin_m_ = declare_parameter<double>("boundary_inner_margin_m", 0.3);
  strip_boundary_margin_m_ = declare_parameter<double>("strip_boundary_margin_m", 0.5);
  mow_angle_override_deg_ =
      declare_parameter<double>("mow_angle_deg", std::numeric_limits<double>::quiet_NaN());

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

  auto odom_topic = declare_parameter<std::string>("odom_topic", "/odometry/filtered_map");
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

  // Path C cell-based coverage. New service kept side-by-side with
  // get_next_strip during the migration.
  get_next_segment_srv_ = create_service<mowgli_interfaces::srv::GetNextSegment>(
      "~/get_next_segment",
      [this](const mowgli_interfaces::srv::GetNextSegment::Request::SharedPtr req,
             mowgli_interfaces::srv::GetNextSegment::Response::SharedPtr res)
      {
        on_get_next_segment(req, res);
      });

  mark_segment_blocked_srv_ = create_service<mowgli_interfaces::srv::MarkSegmentBlocked>(
      "~/mark_segment_blocked",
      [this](const mowgli_interfaces::srv::MarkSegmentBlocked::Request::SharedPtr req,
             mowgli_interfaces::srv::MarkSegmentBlocked::Response::SharedPtr res)
      {
        on_mark_segment_blocked(req, res);
      });

  clear_dead_cells_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_dead_cells",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res)
      {
        on_clear_dead_cells(req, res);
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
      create_publisher<std_msgs::msg::Bool>("~/lethal_boundary_violation", rclcpp::QoS(1));

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

  // Dock pose: single source of truth is mowgli_robot.yaml. Calibration
  // (calibrate_imu_yaw_node) and manual GUI placement (~/set_docking_point
  // below) write back to that file, so the parameters declared here are
  // always the latest persisted values.
  double dock_x = declare_parameter<double>("dock_pose_x", 0.0);
  double dock_y = declare_parameter<double>("dock_pose_y", 0.0);
  double dock_yaw = declare_parameter<double>("dock_pose_yaw", 0.0);

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
                "Dock pose from mowgli_robot.yaml: (%.3f, %.3f) yaw=%.3f",
                dock_x,
                dock_y,
                dock_yaw);
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
    const double d_yaw = 2.0 * std::atan2(docking_pose_.orientation.z, docking_pose_.orientation.w);
    const double cy = std::cos(d_yaw);
    const double sy = std::sin(d_yaw);
    const double corners[][2] = {
        {dock_forward, half_width},
        {dock_forward, -half_width},
        {-approach_back, -half_width},
        {-approach_back, half_width},
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
                d_x,
                d_y,
                d_yaw,
                dock_forward,
                approach_back,
                half_width);
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
                            std::string(layers::CONFIDENCE),
                            std::string(layers::FAIL_COUNT)});

  map_.setFrameId(map_frame_);
  map_.setGeometry(grid_map::Length(map_size_x_, map_size_y_),
                   resolution_,
                   grid_map::Position(0.0, 0.0));

  map_[std::string(layers::OCCUPANCY)].setConstant(defaults::OCCUPANCY);
  map_[std::string(layers::CLASSIFICATION)].setConstant(defaults::CLASSIFICATION);
  map_[std::string(layers::MOW_PROGRESS)].setConstant(defaults::MOW_PROGRESS);
  map_[std::string(layers::CONFIDENCE)].setConstant(defaults::CONFIDENCE);
  map_[std::string(layers::FAIL_COUNT)].setConstant(defaults::FAIL_COUNT);

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
  map_[std::string(layers::FAIL_COUNT)].setConstant(defaults::FAIL_COUNT);

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
      auto tf = tf_buffer_->lookupTransform(map_frame_, "base_footprint", tf2::TimePointZero);
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

    // Only publish masks when something changed. The publishers use
    // transient_local QoS so late subscribers (e.g. costmap_filter)
    // automatically receive the most recent mask. Republishing a
    // stale cached mask each tick was triggering the global_costmap
    // KeepoutFilter to reload its filter every second ("New filter
    // mask arrived" log), invalidating active plans and causing
    // docking nav-to-staging to never settle.
    if (masks_dirty_)
    {
      publish_keepout_mask();
      publish_speed_mask();
      masks_dirty_ = false;
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
    strip_layouts_.clear();
    current_strip_idx_.clear();

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
  strip_layouts_.clear();
  current_strip_idx_.clear();
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

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    areas_.push_back(std::move(entry));
  }
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
  map_[std::string(layers::FAIL_COUNT)].setConstant(defaults::FAIL_COUNT);
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
      else if (cell_type == CellType::LAWN_DEAD)
      {
        // Distinct value for cells the segment selector has given up
        // on. The GUI map renderer can pick a separate color (e.g.
        // amber) so the operator sees the "blocked but not a real
        // obstacle" zones at a glance. 80 sits between mowed (0) and
        // hard obstacle (100).
        grid.data[flat_idx] = 80;
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
  if (elapsed_seconds <= 0.0)
  {
    return;
  }

  if (decay_rate_per_hour_ > 0.0)
  {
    const double decay_per_second = decay_rate_per_hour_ / 3600.0;
    const float decay = static_cast<float>(decay_per_second * elapsed_seconds);
    auto& prog = map_[std::string(layers::MOW_PROGRESS)];
    prog = (prog.array() - decay).max(0.0F).matrix();
  }

  // Path C — fail_count decay. Cells that haven't been re-blocked
  // recently bleed off their failure count, so a transient obstacle
  // doesn't permanently disable a region. When fail_count drops below
  // the unblock threshold a previously-promoted LAWN_DEAD cell
  // reverts to LAWN.
  if (dead_decay_rate_per_hour_ > 0.0)
  {
    const double decay_per_second = dead_decay_rate_per_hour_ / 3600.0;
    const float decay = static_cast<float>(decay_per_second * elapsed_seconds);
    auto& fc = map_[std::string(layers::FAIL_COUNT)];
    fc = (fc.array() - decay).max(0.0F).matrix();

    auto& cls = map_[std::string(layers::CLASSIFICATION)];
    const float unblock = static_cast<float>(dead_unblock_threshold_);
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
    {
      auto t = static_cast<CellType>(static_cast<int>(cls((*it)(0), (*it)(1))));
      if (t == CellType::LAWN_DEAD && fc((*it)(0), (*it)(1)) < unblock)
        cls((*it)(0), (*it)(1)) = static_cast<float>(CellType::LAWN);
    }
  }
}

void MapServerNode::mark_cells_mowed(double x, double y)
{
  const grid_map::Position center(x, y);
  const double radius = mower_width_ * 0.5;

  for (grid_map::CircleIterator it(map_, center, radius); !it.isPastEnd(); ++it)
  {
    map_.at(std::string(layers::MOW_PROGRESS), *it) = 1.0F;
    map_.at(std::string(layers::CONFIDENCE), *it) += 1.0F;
    // Path C — successful mow resets the failure counter so the next
    // obstacle encounter starts from zero, not from a stale count.
    map_.at(std::string(layers::FAIL_COUNT), *it) = 0.0F;
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

static ClosestEdge closest_edge_point(double px,
                                      double py,
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
  // Deadband: only flag a soft violation once the robot is outside by
  // more than soft_boundary_margin_m_. Avoids the transit/abort loop
  // where RTK noise + FTC tracking error around strip endpoints keeps
  // the robot oscillating across the polygon edge.
  soft_msg.data = !inside_any && (min_edge_dist > soft_boundary_margin_m_);
  boundary_violation_pub_->publish(soft_msg);

  std_msgs::msg::Bool lethal_msg;
  lethal_msg.data = !inside_any && (min_edge_dist > lethal_boundary_margin_m_);
  lethal_boundary_violation_pub_->publish(lethal_msg);

  // Only escalate logging when the blade is actively running. When the blade
  // is off the robot is either idle on the dock or transiting between areas —
  // both states legitimately place the robot outside any defined polygon, so
  // an ERROR-level log would just spam the rosout. The /boundary_violation
  // topics are still published unconditionally so the BT can react.
  if (!inside_any && mow_blade_enabled_)
  {
    if (lethal_msg.data)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(),
                            *get_clock(),
                            2000,
                            "LETHAL BOUNDARY VIOLATION: robot at (%.2f, %.2f) — %.2fm outside "
                            "nearest allowed area (margin=%.2fm)",
                            x,
                            y,
                            min_edge_dist,
                            lethal_boundary_margin_m_);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           2000,
                           "BOUNDARY VIOLATION: robot at (%.2f, %.2f) — %.2fm outside nearest "
                           "allowed area (lethal at %.2fm)",
                           x,
                           y,
                           min_edge_dist,
                           lethal_boundary_margin_m_);
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
    auto tf = tf_buffer_->lookupTransform(map_frame_, "base_footprint", tf2::TimePointZero);
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
              rx,
              ry,
              best.distance,
              tx,
              ty,
              yaw);
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
  size_t new_count = 0;
  for (const auto& id : current_persistent_ids)
  {
    if (planned_obstacle_ids_.find(id) == planned_obstacle_ids_.end())
      ++new_count;
  }
  planned_obstacle_ids_ = current_persistent_ids;
  std_msgs::msg::Bool msg;
  msg.data = true;
  replan_needed_pub_->publish(msg);
  last_replan_time_ = now();
  RCLCPP_INFO(get_logger(), "Replan triggered: %zu new persistent obstacles", new_count);
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

  // Persist to mowgli_robot.yaml — single source of truth for dock pose.
  // Manual placements via the GUI land here; calibrate_imu_yaw_node writes
  // the same file when its dock pre-phase finishes. A line-regex update
  // preserves the surrounding comments / structure.
  try
  {
    const double yaw_rad =
        2.0 * std::atan2(docking_pose_.orientation.z, docking_pose_.orientation.w);
    if (!update_dock_pose_in_robot_yaml(
            kRuntimeRobotYaml, docking_pose_.position.x, docking_pose_.position.y, yaw_rad))
    {
      RCLCPP_WARN(get_logger(),
                  "Could not persist dock pose to %s — file missing or "
                  "not writable. Pose still applied in-memory.",
                  kRuntimeRobotYaml);
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Persisted dock pose to %s: (%.3f, %.3f) yaw=%.3f rad",
                  kRuntimeRobotYaml,
                  docking_pose_.position.x,
                  docking_pose_.position.y,
                  yaw_rad);
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN(get_logger(),
                "Failed to persist dock pose to %s: %s",
                kRuntimeRobotYaml,
                ex.what());
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

  // Dock pose intentionally NOT serialized here. The single source of truth
  // is mowgli_robot.yaml — written by calibrate_imu_yaw_node and
  // on_set_docking_point. Storing it in areas.dat too led to a stale
  // all-zero pose taking precedence over the calibrated value.

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
  strip_layouts_.clear();
  current_strip_idx_.clear();

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

  // Dock pose is loaded from mowgli_robot.yaml at construction, never
  // from areas.dat. Old areas.dat files may still contain dock_x/dock_qw
  // keys — they are ignored on purpose.

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

std::vector<std::pair<double, double>> MapServerNode::convex_hull(
    std::vector<std::pair<double, double>> pts)
{
  auto cross = [](const auto& O, const auto& A, const auto& B)
  {
    return (A.first - O.first) * (B.second - O.second) -
           (A.second - O.second) * (B.first - O.first);
  };

  int n = static_cast<int>(pts.size());
  if (n < 3)
    return pts;

  std::sort(pts.begin(), pts.end());

  std::vector<std::pair<double, double>> hull(2 * n);
  int k = 0;

  // Lower hull
  for (int i = 0; i < n; ++i)
  {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0)
      k--;
    hull[k++] = pts[i];
  }

  // Upper hull
  for (int i = n - 2, t = k + 1; i >= 0; i--)
  {
    while (k >= t && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0)
      k--;
    hull[k++] = pts[i];
  }

  hull.resize(k - 1);
  return hull;
}

double MapServerNode::compute_optimal_mow_angle(const geometry_msgs::msg::Polygon& poly)
{
  std::vector<std::pair<double, double>> pts;
  pts.reserve(poly.points.size());
  for (const auto& p : poly.points)
    pts.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y));

  auto hull = convex_hull(std::move(pts));
  if (hull.size() < 3)
    return 0.0;

  double best_angle = 0.0;
  double min_perp_extent = 1e9;
  int nh = static_cast<int>(hull.size());

  for (int i = 0; i < nh; ++i)
  {
    int j = (i + 1) % nh;
    double edge_dx = hull[j].first - hull[i].first;
    double edge_dy = hull[j].second - hull[i].second;
    double edge_angle = std::atan2(edge_dy, edge_dx);

    double cos_a = std::cos(-edge_angle);
    double sin_a = std::sin(-edge_angle);

    // Compute bounding box of hull rotated to align this edge with X axis
    double min_y = 1e9, max_y = -1e9;
    for (const auto& [hx, hy] : hull)
    {
      double ry = sin_a * hx + cos_a * hy;
      min_y = std::min(min_y, ry);
      max_y = std::max(max_y, ry);
    }

    double perp_extent = max_y - min_y;
    if (perp_extent < min_perp_extent)
    {
      min_perp_extent = perp_extent;
      best_angle = edge_angle;
    }
  }

  return best_angle;
}

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

  // Navigation-only areas: never generate strips. The planner uses the
  // polygon for transit costmap/keepout but the BT must not pick this
  // area as a mowing target.
  if (area.is_navigation_area)
  {
    layout.valid = true;
    layout.strips.clear();
    return;
  }

  // ── 1. Determine mow angle ────────────────────────────────────────────────
  // Auto-compute from polygon MBR or use manual override.
  double mow_angle;
  if (std::isnan(mow_angle_override_deg_))
  {
    mow_angle = compute_optimal_mow_angle(poly);
  }
  else
  {
    mow_angle = mow_angle_override_deg_ * M_PI / 180.0;
  }
  layout.mow_angle = mow_angle;

  // ── 2. Rotate polygon so optimal strip direction aligns with Y axis ───────
  // Strips currently run along Y (vertical scan). Rotation angle:
  //   rot = π/2 - mow_angle
  // maps the desired strip direction onto the Y axis.
  double rot = M_PI / 2.0 - mow_angle;
  double cos_r = std::cos(rot);
  double sin_r = std::sin(rot);

  int n_pts = static_cast<int>(poly.points.size());
  std::vector<std::pair<double, double>> rotated_pts;
  rotated_pts.reserve(n_pts);
  for (const auto& p : poly.points)
  {
    double rx = cos_r * static_cast<double>(p.x) - sin_r * static_cast<double>(p.y);
    double ry = sin_r * static_cast<double>(p.x) + cos_r * static_cast<double>(p.y);
    rotated_pts.emplace_back(rx, ry);
  }

  // Bounding box of rotated polygon (X only — for scan line range)
  double min_x = 1e9, max_x = -1e9;
  for (const auto& [rx, ry] : rotated_pts)
  {
    min_x = std::min(min_x, rx);
    max_x = std::max(max_x, rx);
  }

  // ── 3. Inset and scan ─────────────────────────────────────────────────────
  double inset = strip_boundary_margin_m_;
  double inner_min_x = min_x + inset;
  double inner_max_x = max_x - inset;

  if (inner_min_x >= inner_max_x)
  {
    layout.valid = true;
    return;
  }

  // Inverse rotation to map strip endpoints back to the original frame.
  // cos(-rot) = cos(rot), sin(-rot) = -sin(rot)
  double cos_back = cos_r;
  double sin_back = -sin_r;

  layout.strips.clear();
  int col = 0;

  for (double x = inner_min_x + mower_width_ / 2; x <= inner_max_x; x += mower_width_)
  {
    // Find Y intersections of vertical line x=const with rotated polygon edges
    std::vector<double> y_intersections;
    for (int i = 0; i < n_pts; ++i)
    {
      int j = (i + 1) % n_pts;
      double x1 = rotated_pts[i].first;
      double y1 = rotated_pts[i].second;
      double x2 = rotated_pts[j].first;
      double y2 = rotated_pts[j].second;

      if ((x1 < x && x2 >= x) || (x2 < x && x1 >= x))
      {
        double t = (x - x1) / (x2 - x1);
        y_intersections.push_back(y1 + t * (y2 - y1));
      }
    }

    if (y_intersections.size() < 2)
    {
      col++;
      continue;
    }

    std::sort(y_intersections.begin(), y_intersections.end());

    // Even-odd fill: pair consecutive intersections [0,1], [2,3], ...
    // This correctly handles concave polygons (L, U shapes) by producing
    // multiple strip segments per scan line instead of spanning the gap.
    for (size_t k = 0; k + 1 < y_intersections.size(); k += 2)
    {
      double y_lo = y_intersections[k] + inset;
      double y_hi = y_intersections[k + 1] - inset;

      if (y_hi - y_lo < mower_width_)
        continue;

      // Rotate strip endpoints back to map frame
      Strip strip;
      strip.start.x = cos_back * x - sin_back * y_lo;
      strip.start.y = sin_back * x + cos_back * y_lo;
      strip.start.z = 0.0;
      strip.end.x = cos_back * x - sin_back * y_hi;
      strip.end.y = sin_back * x + cos_back * y_hi;
      strip.end.z = 0.0;
      strip.column_index = col;
      layout.strips.push_back(strip);
    }
    col++;
  }

  layout.valid = true;
  RCLCPP_INFO(get_logger(),
              "Strip layout for area '%s': %zu strips, mow_angle=%.1f° (%s), "
              "rotated bbox X=[%.2f, %.2f], inner_x=[%.2f, %.2f]",
              area.name.c_str(),
              layout.strips.size(),
              layout.mow_angle * 180.0 / M_PI,
              std::isnan(mow_angle_override_deg_) ? "auto-MBR" : "manual",
              min_x,
              max_x,
              inner_min_x,
              inner_max_x);
  if (!layout.strips.empty())
  {
    const auto& first = layout.strips.front();
    const auto& last = layout.strips.back();
    RCLCPP_INFO(get_logger(),
                "  First strip: (%.2f,%.2f)→(%.2f,%.2f), "
                "Last strip: (%.2f,%.2f)→(%.2f,%.2f)",
                first.start.x,
                first.start.y,
                first.end.x,
                first.end.y,
                last.start.x,
                last.start.y,
                last.end.x,
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
    // Skip cells the robot must never mow: tracked obstacles AND
    // operator/dock-defined exclusion zones. Without the NO_GO_ZONE
    // skip, strips passing over the dock exclusion polygon (or any
    // exclusion drawn inside a mowing area) count as "not mowed"
    // forever — the robot keeps replanning the same strip and never
    // marks the surrounding cells complete.
    if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY ||
        cell_type == CellType::NO_GO_ZONE)
      continue;

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

void MapServerNode::select_nearest_endpoint_strip(const std::vector<Strip>& strips,
                                                  const std::vector<bool>& eligible,
                                                  double robot_x,
                                                  double robot_y,
                                                  int& out_index,
                                                  Strip& out_strip)
{
  out_index = -1;
  double best_dist = std::numeric_limits<double>::infinity();
  bool best_flip = false;

  const int n = static_cast<int>(strips.size());
  for (int i = 0; i < n; ++i)
  {
    if (i >= static_cast<int>(eligible.size()) || !eligible[i])
      continue;

    const auto& s = strips[i];
    const double d_start = std::hypot(s.start.x - robot_x, s.start.y - robot_y);
    const double d_end = std::hypot(s.end.x - robot_x, s.end.y - robot_y);

    const double d = std::min(d_start, d_end);
    if (d < best_dist)
    {
      best_dist = d;
      out_index = i;
      best_flip = (d_end < d_start);
    }
  }

  if (out_index < 0)
    return;

  out_strip = strips[out_index];
  if (best_flip)
    std::swap(out_strip.start, out_strip.end);
}

bool MapServerNode::find_next_unmowed_strip(
    size_t area_index, double robot_x, double robot_y, Strip& out_strip, bool /*prefer_headland*/)
{
  ensure_strip_layout(area_index);

  if (area_index >= strip_layouts_.size() || !strip_layouts_[area_index].valid)
    return false;

  const auto& layout = strip_layouts_[area_index];
  const int n = static_cast<int>(layout.strips.size());
  if (n == 0)
    return false;

  // Grow tracking vector if needed (kept for compatibility / debugging — the
  // selector itself no longer consumes it).
  if (current_strip_idx_.size() <= area_index)
    current_strip_idx_.resize(area_index + 1, -1);

  // Build eligibility mask: a strip is eligible iff it isn't already mowed and
  // isn't blocked by obstacles (>50% obstacle cells — those are treated as
  // frontier and are skipped during planning).
  std::vector<bool> eligible(n, false);
  for (int i = 0; i < n; ++i)
  {
    const auto& strip = layout.strips[i];
    eligible[i] = !is_strip_mowed(strip) && !is_strip_blocked(strip);
  }

  // Pick the eligible strip whose nearest endpoint is closest to the current
  // robot pose, and orient it so the robot enters from that endpoint. This
  // produces a serpentine/boustrophedon order naturally when adjacent strips
  // are eligible (the previously-mowed strip ended at one column edge, so the
  // adjacent strip's matching endpoint is the nearest by ~one swath width)
  // while gracefully handling skipped or partially-blocked strips.
  int picked = -1;
  select_nearest_endpoint_strip(layout.strips, eligible, robot_x, robot_y, picked, out_strip);
  if (picked < 0)
    return false;

  current_strip_idx_[area_index] = picked;
  return true;
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

  // Navigation-only areas are transit corridors, not lawn — they
  // exist so the planner has a passage between mowing zones, not so
  // the robot tonds them. Report 0 strips remaining and 100% coverage
  // so GetNextUnmowedArea moves on without ever generating a strip
  // through them.
  if (areas_[req->area_index].is_navigation_area)
  {
    res->success = true;
    res->total_cells = 0;
    res->mowed_cells = 0;
    res->obstacle_cells = 0;
    res->coverage_percent = 100.0f;
    res->strips_remaining = 0;
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

// ─────────────────────────────────────────────────────────────────────────────
// Path C — cell-based coverage selector
// ─────────────────────────────────────────────────────────────────────────────
//
// Replaces the strip-based plan with a per-call short segment, picked
// from the live mow_progress + classification grid. Handles obstacles
// in the middle of a row by stopping the segment short, and ignores
// LAWN_DEAD cells entirely (let them decay back to LAWN if the
// blocking obstacle ever clears).
//
// The selector is intentionally simple: walk along prefer_dir from
// the robot's projected row position until we hit a non-mowable cell.
// The BT calls back after each segment, so we pick up obstacle changes
// observed during the previous segment automatically — no need for
// the planner itself to subscribe to obstacle updates.

namespace
{
struct RowBasis
{
  // Unit vector along the mowing row (the direction strips run).
  double ux, uy;
  // Unit vector across rows (perpendicular).
  double vx, vy;
};

// Project (x, y) onto the row basis. u = along-row, v = across-row.
inline void project_to_basis(const RowBasis& b, double x, double y, double& u, double& v)
{
  u = x * b.ux + y * b.uy;
  v = x * b.vx + y * b.vy;
}

// Inverse projection: from (u, v) basis coords back to map (x, y).
inline void basis_to_map(const RowBasis& b, double u, double v, double& x, double& y)
{
  x = u * b.ux + v * b.vx;
  y = u * b.uy + v * b.vy;
}

inline double wrap_pi(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

inline RowBasis make_basis(double prefer_dir_yaw)
{
  RowBasis b;
  b.ux = std::cos(prefer_dir_yaw);
  b.uy = std::sin(prefer_dir_yaw);
  b.vx = -b.uy;
  b.vy = b.ux;
  return b;
}
}  // namespace

bool MapServerNode::find_next_segment(size_t area_index,
                                      double robot_x,
                                      double robot_y,
                                      double robot_yaw,
                                      double prefer_dir_yaw,
                                      bool boustrophedon,
                                      double max_segment_length_m,
                                      SegmentResult& out_seg) const
{
  out_seg = SegmentResult{};

  if (area_index >= areas_.size())
    return false;
  const auto& area = areas_[area_index];
  if (area.is_navigation_area)
  {
    out_seg.coverage_complete = true;
    return true;
  }
  const auto& poly = area.polygon;
  if (poly.points.size() < 3)
    return false;

  // Row pitch — same as inter-strip distance: tool_width / mower_width.
  // Falls back to the resolution if mower_width_ wasn't set (sim).
  const double row_pitch = mower_width_ > 1e-3 ? mower_width_ : (resolution_ * 2.0);
  // Step length along u: the resolution gives us per-cell granularity.
  const double step = resolution_;
  // Default cap when caller passes 0 — keeps each FollowSegment short
  // enough that obstacle changes get reflected by the next call.
  const double cap = max_segment_length_m > 0.0 ? max_segment_length_m : 3.0;

  const RowBasis B = make_basis(prefer_dir_yaw);

  // ── 1. Robot's row index (snap to nearest row centreline) ────────────
  double r_u, r_v;
  project_to_basis(B, robot_x, robot_y, r_u, r_v);
  const long current_row = std::lround(r_v / row_pitch);

  // ── 2. Layer accessors. We read mowed/classification through the
  //      grid_map at world positions, NOT cell indices, so no
  //      coordinate-system flipping bugs (see CLAUDE.md note 14). ─────
  const auto& cls = map_[std::string(layers::CLASSIFICATION)];
  const auto& prog = map_[std::string(layers::MOW_PROGRESS)];

  // Returns true when (x, y) is mowable AND not yet mowed AND inside
  // the area polygon. Used both to pick a starting cell and to walk
  // along the row.
  auto is_mowable_unmowed = [&](double x, double y) -> bool
  {
    geometry_msgs::msg::Point32 pt32;
    pt32.x = static_cast<float>(x);
    pt32.y = static_cast<float>(y);
    if (!point_in_polygon(pt32, poly))
      return false;
    grid_map::Position pos(x, y);
    if (!map_.isInside(pos))
      return false;
    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
      return false;
    auto t = static_cast<CellType>(static_cast<int>(cls(idx(0), idx(1))));
    if (t == CellType::OBSTACLE_PERMANENT || t == CellType::OBSTACLE_TEMPORARY ||
        t == CellType::NO_GO_ZONE || t == CellType::LAWN_DEAD)
      return false;
    return prog(idx(0), idx(1)) < 0.3f;
  };

  // Returns true when (x, y) hits a hard stop boundary (outside
  // polygon OR in an obstacle / DEAD cell). MOWED cells are NOT a
  // hard stop — we just walk past them.
  auto is_blocking = [&](double x, double y, std::string& reason) -> bool
  {
    geometry_msgs::msg::Point32 pt32;
    pt32.x = static_cast<float>(x);
    pt32.y = static_cast<float>(y);
    if (!point_in_polygon(pt32, poly))
    {
      reason = "boundary";
      return true;
    }
    grid_map::Position pos(x, y);
    if (!map_.isInside(pos))
    {
      reason = "boundary";
      return true;
    }
    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
    {
      reason = "boundary";
      return true;
    }
    auto t = static_cast<CellType>(static_cast<int>(cls(idx(0), idx(1))));
    if (t == CellType::OBSTACLE_PERMANENT || t == CellType::OBSTACLE_TEMPORARY ||
        t == CellType::NO_GO_ZONE)
    {
      reason = "obstacle";
      return true;
    }
    if (t == CellType::LAWN_DEAD)
    {
      reason = "dead_zone";
      return true;
    }
    return false;
  };

  // ── 3. Direction along u for the current row ────────────────────────
  // Boustrophedon: alternate per row index. Otherwise +u always.
  // Heuristic override: if the robot is currently facing closer to -u
  // than +u, flip the sign so we don't force a 180° rotation up front.
  double dir = boustrophedon && (current_row & 1L) ? -1.0 : 1.0;
  {
    const double yaw_to_dir = std::atan2(dir * B.uy, dir * B.ux);  // direction of u in world
    if (std::fabs(wrap_pi(robot_yaw - yaw_to_dir)) > M_PI / 2.0)
      dir = -dir;
  }

  // ── 4. Pick a start point on the current row ────────────────────────
  // Snap robot u to the nearest grid step, then march in dir until we
  // find an unmowed cell (we may have just driven over mowed cells).
  const double row_v = static_cast<double>(current_row) * row_pitch;
  double walk_u = std::round(r_u / step) * step;
  bool found_start = false;
  double start_x = robot_x;
  double start_y = robot_y;
  for (int i = 0; i < static_cast<int>(cap / step) + 1; ++i)
  {
    double cx, cy;
    basis_to_map(B, walk_u, row_v, cx, cy);
    if (is_mowable_unmowed(cx, cy))
    {
      start_x = cx;
      start_y = cy;
      found_start = true;
      break;
    }
    // If we cross a hard block before finding any unmowed cell, the
    // current row in this direction is exhausted — fall through to
    // the row-search path.
    std::string reason;
    if (is_blocking(cx, cy, reason))
      break;
    walk_u += dir * step;
  }

  // ── 5. If the current row is exhausted, scan rows for the closest
  //      unmowed reachable cell. Brute-force iteration over the
  //      polygon-bounded grid is fine — typical area is ≤30×30 m at
  //      0.1 m resolution = 90 k cells, traversed once. ────────────
  if (!found_start)
  {
    double best_dist2 = std::numeric_limits<double>::infinity();
    grid_map::Position best_pos(0.0, 0.0);
    long best_row = current_row;
    grid_map::Polygon gm_poly;
    for (const auto& p : poly.points)
      gm_poly.addVertex(grid_map::Position(static_cast<double>(p.x), static_cast<double>(p.y)));
    for (grid_map::PolygonIterator it(map_, gm_poly); !it.isPastEnd(); ++it)
    {
      grid_map::Position p;
      if (!map_.getPosition(*it, p))
        continue;
      if (!is_mowable_unmowed(p.x(), p.y()))
        continue;
      const double dx = p.x() - robot_x;
      const double dy = p.y() - robot_y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_dist2)
      {
        best_dist2 = d2;
        best_pos = p;
        double cu, cv;
        project_to_basis(B, p.x(), p.y(), cu, cv);
        best_row = std::lround(cv / row_pitch);
      }
    }
    if (!std::isfinite(best_dist2))
    {
      out_seg.coverage_complete = true;
      return true;
    }
    start_x = best_pos.x();
    start_y = best_pos.y();
    out_seg.is_long_transit = std::sqrt(best_dist2) > 0.5;
    // Recompute direction for this row (boustrophedon snake).
    dir = boustrophedon && (best_row & 1L) ? -1.0 : 1.0;
    // Reset walk position to the chosen cell's u coordinate.
    project_to_basis(B, best_pos.x(), best_pos.y(), walk_u, r_v);
  }

  out_seg.start_x = start_x;
  out_seg.start_y = start_y;

  // ── 6. Walk along the row in dir until a stop condition fires ─────
  double end_x = start_x;
  double end_y = start_y;
  std::string reason;
  int cells = 0;
  double walked = 0.0;
  while (walked < cap)
  {
    walk_u += dir * step;
    double cx, cy;
    basis_to_map(B, walk_u, row_v, cx, cy);
    if (is_blocking(cx, cy, reason))
      break;
    end_x = cx;
    end_y = cy;
    ++cells;
    walked += step;
  }
  if (reason.empty())
    reason = walked >= cap ? "max_length" : "row_end";

  out_seg.end_x = end_x;
  out_seg.end_y = end_y;
  out_seg.cell_count = cells;
  out_seg.termination_reason = reason;

  // Long transit when the start point is not the robot's current
  // position (or when we landed on a different row). 0.5 m gap is
  // generous — covers the typical row-to-row jump but not in-row
  // micro-correction.
  if (!out_seg.is_long_transit)
  {
    const double gap = std::hypot(start_x - robot_x, start_y - robot_y);
    out_seg.is_long_transit = gap > 0.5;
  }

  return true;
}

void MapServerNode::on_get_next_segment(
    const mowgli_interfaces::srv::GetNextSegment::Request::SharedPtr req,
    mowgli_interfaces::srv::GetNextSegment::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_index >= areas_.size())
  {
    res->success = false;
    return;
  }

  // Navigation-only areas: same short-circuit as on_get_coverage_status.
  if (areas_[req->area_index].is_navigation_area)
  {
    res->success = true;
    res->coverage_complete = true;
    res->coverage_percent = 100.0f;
    res->phase = "complete";
    return;
  }

  SegmentResult seg;
  if (!find_next_segment(req->area_index,
                         req->robot_x,
                         req->robot_y,
                         req->robot_yaw_rad,
                         req->prefer_dir_yaw_rad,
                         req->boustrophedon,
                         req->max_segment_length_m,
                         seg))
  {
    res->success = false;
    return;
  }

  if (seg.coverage_complete)
  {
    res->success = true;
    res->coverage_complete = true;
    res->coverage_percent = 100.0f;
    res->phase = "complete";
    return;
  }

  // Build path: dense linspace from start to end at resolution_ steps.
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = map_frame_;
  const double dx = seg.end_x - seg.start_x;
  const double dy = seg.end_y - seg.start_y;
  const double length = std::hypot(dx, dy);
  const int n_steps = std::max(1, static_cast<int>(std::ceil(length / resolution_)));
  const double seg_yaw = std::atan2(dy, dx);
  for (int i = 0; i <= n_steps; ++i)
  {
    const double t = static_cast<double>(i) / n_steps;
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = seg.start_x + t * dx;
    p.pose.position.y = seg.start_y + t * dy;
    p.pose.position.z = 0.0;
    // Orient each pose along the segment direction so FTC's
    // PRE_ROTATE phase aligns the robot before driving.
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = std::sin(seg_yaw / 2.0);
    p.pose.orientation.w = std::cos(seg_yaw / 2.0);
    path.poses.push_back(p);
  }

  res->success = true;
  res->coverage_complete = false;
  res->segment_path = path;
  res->target_cell_pose =
      path.poses.empty() ? geometry_msgs::msg::PoseStamped() : path.poses.back();
  res->is_long_transit = seg.is_long_transit;
  res->termination_reason = seg.termination_reason;
  res->phase = seg.is_long_transit ? "transit" : "interior";

  // Coverage stats — reuse the strip planner's per-area accumulator
  // since cell semantics are identical (mow_progress >= 0.3 = mowed).
  uint32_t total = 0, mowed_cells = 0, obs = 0;
  compute_coverage_stats(req->area_index, total, mowed_cells, obs);
  res->coverage_percent = total > 0 ? 100.0f * mowed_cells / total : 0.0f;

  // dead_cells_count: walk the area polygon, count LAWN_DEAD cells.
  // Cheap (one polygon iter per call) and the GUI uses it as a
  // session-quality indicator.
  uint32_t dead = 0;
  grid_map::Polygon gm_poly;
  for (const auto& p : areas_[req->area_index].polygon.points)
    gm_poly.addVertex(grid_map::Position(static_cast<double>(p.x), static_cast<double>(p.y)));
  const auto& cls = map_[std::string(layers::CLASSIFICATION)];
  for (grid_map::PolygonIterator it(map_, gm_poly); !it.isPastEnd(); ++it)
  {
    auto t = static_cast<CellType>(static_cast<int>(cls((*it)(0), (*it)(1))));
    if (t == CellType::LAWN_DEAD)
      ++dead;
  }
  res->dead_cells_count = dead;

  // Rough remaining-segments estimate: unmowed cells / cells_per_segment.
  const uint32_t unmowed = total > mowed_cells ? total - mowed_cells - obs - dead : 0;
  const double cells_per_seg = std::max(1.0, 3.0 / resolution_);  // cap=3m default
  res->segments_remaining_estimate =
      static_cast<uint32_t>(std::ceil(static_cast<double>(unmowed) / cells_per_seg));
}

// Test wrapper for find_next_segment — flattens SegmentResult into
// scalar out-params so unit tests can call the selector without
// pulling the struct definition. Caller holds map_mutex_.
bool MapServerNode::find_next_segment_public(size_t area_index,
                                             double robot_x,
                                             double robot_y,
                                             double robot_yaw,
                                             double prefer_dir_yaw,
                                             bool boustrophedon,
                                             double max_segment_length_m,
                                             double& out_start_x,
                                             double& out_start_y,
                                             double& out_end_x,
                                             double& out_end_y,
                                             int& out_cell_count,
                                             std::string& out_termination_reason,
                                             bool& out_is_long_transit,
                                             bool& out_coverage_complete) const
{
  SegmentResult seg;
  const bool ok = find_next_segment(area_index,
                                    robot_x,
                                    robot_y,
                                    robot_yaw,
                                    prefer_dir_yaw,
                                    boustrophedon,
                                    max_segment_length_m,
                                    seg);
  out_start_x = seg.start_x;
  out_start_y = seg.start_y;
  out_end_x = seg.end_x;
  out_end_y = seg.end_y;
  out_cell_count = seg.cell_count;
  out_termination_reason = seg.termination_reason;
  out_is_long_transit = seg.is_long_transit;
  out_coverage_complete = seg.coverage_complete;
  return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// Path C — mark_segment_blocked + clear_dead_cells handlers
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_mark_segment_blocked(
    const mowgli_interfaces::srv::MarkSegmentBlocked::Request::SharedPtr req,
    mowgli_interfaces::srv::MarkSegmentBlocked::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_index >= areas_.size())
  {
    res->success = false;
    return;
  }
  const auto& area = areas_[req->area_index];
  if (area.polygon.points.size() < 3)
  {
    res->success = false;
    return;
  }

  auto& fc = map_[std::string(layers::FAIL_COUNT)];
  auto& cls = map_[std::string(layers::CLASSIFICATION)];
  const float promote = static_cast<float>(dead_promote_threshold_);

  // Track which cells we've already bumped this call so a redundant
  // path that loops over the same cell doesn't get charged twice for
  // a single failure event.
  std::set<std::pair<int, int>> bumped;
  uint32_t bumps = 0;
  uint32_t promotions = 0;

  for (const auto& pose : req->failed_path.poses)
  {
    geometry_msgs::msg::Point32 pt32;
    pt32.x = static_cast<float>(pose.pose.position.x);
    pt32.y = static_cast<float>(pose.pose.position.y);
    if (!point_in_polygon(pt32, area.polygon))
      continue;
    grid_map::Position pos(pose.pose.position.x, pose.pose.position.y);
    if (!map_.isInside(pos))
      continue;
    grid_map::Index idx;
    if (!map_.getIndex(pos, idx))
      continue;
    auto key = std::make_pair(idx(0), idx(1));
    if (!bumped.insert(key).second)
      continue;

    auto t = static_cast<CellType>(static_cast<int>(cls(idx(0), idx(1))));
    // Don't bump cells that are already non-mowable (real obstacles
    // get their own handling via diff_and_update_obstacles); we only
    // care about LAWN cells the robot couldn't reach.
    if (t != CellType::LAWN && t != CellType::UNKNOWN && t != CellType::LAWN_DEAD)
      continue;

    fc(idx(0), idx(1)) += 1.0F;
    ++bumps;

    if (fc(idx(0), idx(1)) >= promote && t != CellType::LAWN_DEAD)
    {
      cls(idx(0), idx(1)) = static_cast<float>(CellType::LAWN_DEAD);
      ++promotions;
    }
  }

  if (promotions > 0)
  {
    masks_dirty_ = true;
    RCLCPP_WARN(get_logger(),
                "MarkSegmentBlocked: %u cells bumped, %u promoted to LAWN_DEAD "
                "(area %u).",
                bumps,
                promotions,
                req->area_index);
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "MarkSegmentBlocked: %u cells bumped (area %u).",
                bumps,
                req->area_index);
  }

  res->success = true;
  res->cells_marked_blocked = bumps;
  res->cells_promoted_dead = promotions;
}

void MapServerNode::on_clear_dead_cells(const std_srvs::srv::Trigger::Request::SharedPtr,
                                        std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  auto& fc = map_[std::string(layers::FAIL_COUNT)];
  auto& cls = map_[std::string(layers::CLASSIFICATION)];
  uint32_t cleared = 0;
  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
  {
    auto t = static_cast<CellType>(static_cast<int>(cls((*it)(0), (*it)(1))));
    if (t == CellType::LAWN_DEAD)
    {
      cls((*it)(0), (*it)(1)) = static_cast<float>(CellType::LAWN);
      ++cleared;
    }
    fc((*it)(0), (*it)(1)) = 0.0F;
  }
  masks_dirty_ = true;

  res->success = true;
  res->message = "cleared " + std::to_string(cleared) + " LAWN_DEAD cells";
  RCLCPP_INFO(get_logger(), "ClearDeadCells: %s", res->message.c_str());
}

}  // namespace mowgli_map
