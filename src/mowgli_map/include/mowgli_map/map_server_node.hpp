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

#ifndef MOWGLI_MAP__MAP_SERVER_NODE_HPP_
#define MOWGLI_MAP__MAP_SERVER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <nav2_msgs/msg/costmap_filter_info.hpp>

#include <std_msgs/msg/bool.hpp>

#include <mowgli_interfaces/msg/obstacle_array.hpp>
#include <mowgli_interfaces/msg/status.hpp>
#include <mowgli_interfaces/srv/add_mowing_area.hpp>
#include <mowgli_interfaces/srv/get_mowing_area.hpp>

#include "mowgli_map/map_types.hpp"

namespace mowgli_map
{

/// @brief Multi-layer map service node for the Mowgli robot mower.
///
/// Maintains a grid_map::GridMap with four semantic layers:
///   - occupancy       : binary free/occupied for Nav2 costmap
///   - classification  : CellType enum stored as float
///   - mow_progress    : [0,1] freshness of mowing, decays over time
///   - confidence      : cumulative sensor observation count
///
/// The node subscribes to SLAM occupancy grids, odometry, and mower status,
/// and publishes the full multi-layer map plus a visualisation OccupancyGrid
/// for mow_progress. Persistence and zone management are offered as services.
class MapServerNode : public rclcpp::Node
{
public:
  /// @brief Construct the node, declare parameters, create map, wire up all
  ///        publishers, subscribers, services, and timers.
  explicit MapServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  ~MapServerNode() override = default;

  // Non-copyable, non-movable (ROS nodes are singletons in practice)
  MapServerNode(const MapServerNode &)            = delete;
  MapServerNode & operator=(const MapServerNode &) = delete;
  MapServerNode(MapServerNode &&)                 = delete;
  MapServerNode & operator=(MapServerNode &&)     = delete;

  // ── Accessors used by unit tests ────────────────────────────────────────

  /// Direct access to the underlying map (test-only, guarded by map_mutex_).
  grid_map::GridMap & map() { return map_; }
  const grid_map::GridMap & map() const { return map_; }

  /// Mutex guarding the map (test-only).
  std::mutex & map_mutex() { return map_mutex_; }

  /// Expose decay rate for unit tests.
  double decay_rate_per_hour() const { return decay_rate_per_hour_; }

  /// Expose mower width for unit tests.
  double mower_width() const { return mower_width_; }

  /// Run the publish/decay timer callback once (test-only).
  void tick_once(double elapsed_seconds);

  /// Mark cells mowed around a given position (test-only).
  void mark_mowed(double x, double y);

  /// Clear all layers to their default values.
  void clear_map_layers();

private:
  // ── ROS callbacks ────────────────────────────────────────────────────────

  /// Convert incoming nav_msgs/OccupancyGrid to the occupancy layer.
  void on_occupancy_grid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  /// Update mow blade state from mower status.
  void on_mower_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg);

  /// Update mow_progress and confidence layers based on robot position.
  /// Also checks boundary violation.
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /// Receive persistent obstacle updates from ObstacleTracker.
  void on_obstacles(mowgli_interfaces::msg::ObstacleArray::ConstSharedPtr msg);

  // ── Timer callback ───────────────────────────────────────────────────────

  /// Apply decay to mow_progress, publish grid_map and progress OccupancyGrid.
  void on_publish_timer();

  // ── Services ─────────────────────────────────────────────────────────────

  void on_save_map(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_load_map(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_clear_map(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void on_add_no_go_zone(
    const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
    mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res);

  void on_get_mowing_area(
    const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr req,
    mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res);

  // ── Helpers ───────────────────────────────────────────────────────────────

  /// Initialise the grid_map with all four layers and correct geometry.
  void init_map();

  /// Convert mow_progress layer to a nav_msgs/OccupancyGrid (0–100 scale).
  nav_msgs::msg::OccupancyGrid mow_progress_to_occupancy_grid() const;

  /// Apply time-based decay to the mow_progress layer.
  /// @param elapsed_seconds Time since last decay application.
  void apply_decay(double elapsed_seconds);

  /// Mark all cells within mower_width_ / 2 of (x, y) as freshly mowed.
  void mark_cells_mowed(double x, double y);

  /// Check whether a point is inside a polygon (ray-casting algorithm).
  static bool point_in_polygon(
    const geometry_msgs::msg::Point32 & pt,
    const geometry_msgs::msg::Polygon & polygon) noexcept;

  /// Build and publish the keepout OccupancyGrid mask and CostmapFilterInfo.
  /// Outside the mowing boundary → 100 (lethal).  No-go zones → 100.
  /// Inside the mowing boundary → 0 (free).
  /// Does nothing if mowing_area_polygon_ has fewer than 3 points.
  /// Caller must hold map_mutex_.
  void publish_keepout_mask();

  /// Check if the robot is outside all allowed polygons and publish violation.
  void check_boundary_violation(double x, double y);

  /// Compare incoming obstacles to current set and trigger replan if needed.
  void diff_and_update_obstacles(
    const std::vector<mowgli_interfaces::msg::TrackedObstacle> & incoming);

  /// Build and publish the speed OccupancyGrid mask and CostmapFilterInfo.
  /// Cells within one tool_width of the mowing boundary → 50 (50 % speed).
  /// All other interior cells → 0 (full speed).
  /// Does nothing if areas_ is empty.
  /// Caller must hold map_mutex_.
  void publish_speed_mask();

  /// Load pre-defined mowing/navigation areas from ROS parameters.
  void load_areas_from_params();

  /// Parse a polygon from "x1,y1;x2,y2;..." string format.
  static geometry_msgs::msg::Polygon parse_polygon_string(const std::string & s);

  // ── Area entry ────────────────────────────────────────────────────────────

  /// A named area (mowing or navigation) with optional interior obstacles.
  struct AreaEntry {
    std::string name;
    geometry_msgs::msg::Polygon polygon;
    std::vector<geometry_msgs::msg::Polygon> obstacles;
    bool is_navigation_area{false};
  };

  // ── Parameters ────────────────────────────────────────────────────────────
  double resolution_;
  double map_size_x_;
  double map_size_y_;
  std::string map_frame_;
  double decay_rate_per_hour_;
  double mower_width_;
  std::string map_file_path_;
  double publish_rate_;

  // ── State ─────────────────────────────────────────────────────────────────
  grid_map::GridMap map_;
  mutable std::mutex map_mutex_;

  bool mow_blade_enabled_{false};
  rclcpp::Time last_decay_time_;

  // ── Replanning state ──────────────────────────────────────────────────────
  /// Number of persistent obstacles at last replan (for change detection).
  std::size_t last_obstacle_count_{0};
  /// Timestamp of last replan trigger (for cooldown).
  rclcpp::Time last_replan_time_;
  /// Minimum seconds between replan triggers.
  double replan_cooldown_sec_{30.0};
  /// Whether a replan is pending (deferred due to cooldown).
  bool replan_pending_{false};

  /// Pre-defined areas (mowing zones + navigation corridors).
  /// Any cell inside ANY area polygon is free in the keepout mask;
  /// everything outside is lethal.
  std::vector<AreaEntry> areas_;

  /// Obstacle polygons: regions within the allowed areas that are off-limits
  /// (trees, flower beds, etc.).  Marked as lethal in the keepout mask.
  std::vector<geometry_msgs::msg::Polygon> obstacle_polygons_;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mow_progress_pub_;

  // Costmap filter mask publishers (transient_local so late subscribers receive
  // the last message immediately — required by Nav2 costmap filter design).
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr keepout_filter_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_mask_pub_;
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr speed_filter_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr speed_mask_pub_;
  bool keepout_filter_info_sent_{false};
  bool speed_filter_info_sent_{false};

  // Replan and boundary violation publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_needed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr boundary_violation_pub_;

  // ── Subscribers ───────────────────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::ObstacleArray>::SharedPtr obstacle_sub_;

  // ── Services ──────────────────────────────────────────────────────────────
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_map_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_map_srv_;
  rclcpp::Service<mowgli_interfaces::srv::AddMowingArea>::SharedPtr add_no_go_zone_srv_;
  rclcpp::Service<mowgli_interfaces::srv::GetMowingArea>::SharedPtr get_mowing_area_srv_;

  // ── Timers ────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__MAP_SERVER_NODE_HPP_
