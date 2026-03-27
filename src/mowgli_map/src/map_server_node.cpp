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
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <geometry_msgs/msg/point.hpp>

namespace mowgli_map
{

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

MapServerNode::MapServerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("map_server", options)
{
  // ── Declare and read parameters ──────────────────────────────────────────
  resolution_          = declare_parameter<double>("resolution",          0.05);
  map_size_x_          = declare_parameter<double>("map_size_x",          50.0);
  map_size_y_          = declare_parameter<double>("map_size_y",          50.0);
  map_frame_           = declare_parameter<std::string>("map_frame",      "map");
  decay_rate_per_hour_ = declare_parameter<double>("decay_rate_per_hour", 0.1);
  mower_width_         = declare_parameter<double>("mower_width",         0.18);
  map_file_path_       = declare_parameter<std::string>("map_file_path",  "");
  publish_rate_        = declare_parameter<double>("publish_rate",        1.0);

  RCLCPP_INFO(
    get_logger(),
    "MapServerNode: resolution=%.3f m, size=%.1f×%.1f m, frame='%s'",
    resolution_, map_size_x_, map_size_y_, map_frame_.c_str());

  // ── Initialise map ───────────────────────────────────────────────────────
  init_map();
  last_decay_time_ = now();

  // ── Publishers ───────────────────────────────────────────────────────────
  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
    "map_server/grid_map", rclcpp::QoS(1));

  mow_progress_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map_server/mow_progress", rclcpp::QoS(1));

  // ── Subscribers ──────────────────────────────────────────────────────────
  occupancy_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      on_occupancy_grid(std::move(msg));
    });

  status_sub_ = create_subscription<mowgli_interfaces::msg::Status>(
    "/mower_status", rclcpp::QoS(1),
    [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg) {
      on_mower_status(std::move(msg));
    });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(1),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      on_odom(std::move(msg));
    });

  // ── Services ─────────────────────────────────────────────────────────────
  save_map_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/save_map",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
      on_save_map(req, res);
    });

  load_map_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/load_map",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
      on_load_map(req, res);
    });

  clear_map_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/clear_map",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
      on_clear_map(req, res);
    });

  add_no_go_zone_srv_ = create_service<mowgli_interfaces::srv::AddMowingArea>(
    "~/add_no_go_zone",
    [this](
      const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
      mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res) {
      on_add_no_go_zone(req, res);
    });

  get_mowing_area_srv_ = create_service<mowgli_interfaces::srv::GetMowingArea>(
    "~/get_mowing_area",
    [this](
      const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr req,
      mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res) {
      on_get_mowing_area(req, res);
    });

  // ── Publish timer ────────────────────────────────────────────────────────
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_));

  publish_timer_ = create_wall_timer(period_ns, [this]() { on_publish_timer(); });

  RCLCPP_INFO(get_logger(), "MapServerNode ready.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Map initialisation
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::init_map()
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  map_ = grid_map::GridMap(
    {std::string(layers::OCCUPANCY),
     std::string(layers::CLASSIFICATION),
     std::string(layers::MOW_PROGRESS),
     std::string(layers::CONFIDENCE)});

  map_.setFrameId(map_frame_);
  map_.setGeometry(
    grid_map::Length(map_size_x_, map_size_y_),
    resolution_,
    grid_map::Position(0.0, 0.0));

  map_[std::string(layers::OCCUPANCY)].setConstant(defaults::OCCUPANCY);
  map_[std::string(layers::CLASSIFICATION)].setConstant(defaults::CLASSIFICATION);
  map_[std::string(layers::MOW_PROGRESS)].setConstant(defaults::MOW_PROGRESS);
  map_[std::string(layers::CONFIDENCE)].setConstant(defaults::CONFIDENCE);

  RCLCPP_DEBUG(
    get_logger(),
    "Grid map created: %zu×%zu cells",
    static_cast<std::size_t>(map_.getSize()(0)),
    static_cast<std::size_t>(map_.getSize()(1)));
}

// ─────────────────────────────────────────────────────────────────────────────
// Subscription callbacks
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_occupancy_grid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  grid_map::GridMap incoming;
  if (!grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "occupancy_in", incoming)) {
    RCLCPP_WARN(get_logger(), "on_occupancy_grid: failed to convert OccupancyGrid");
    return;
  }

  // Iterate over every cell in the incoming map and copy into our occupancy layer.
  // Values in nav_msgs/OccupancyGrid: 0=free, 100=occupied, -1=unknown.
  // We map: 0→0.0 (free), >50→1.0 (occupied), unknown→NaN (keep existing).
  const auto & info = msg->info;
  const float res   = static_cast<float>(info.resolution);
  const float ox    = static_cast<float>(info.origin.position.x) + res * 0.5F;
  const float oy    = static_cast<float>(info.origin.position.y) + res * 0.5F;

  for (uint32_t row = 0; row < info.height; ++row) {
    for (uint32_t col = 0; col < info.width; ++col) {
      const int8_t cell_val = msg->data[static_cast<std::size_t>(row * info.width + col)];
      if (cell_val < 0) {
        continue;  // unknown — leave existing value intact
      }

      const grid_map::Position pos(
        static_cast<double>(ox + static_cast<float>(col) * res),
        static_cast<double>(oy + static_cast<float>(row) * res));

      grid_map::Index idx;
      if (!map_.getIndex(pos, idx)) {
        continue;  // outside our map bounds
      }

      map_.at(std::string(layers::OCCUPANCY), idx) =
        (cell_val > 50) ? 1.0F : 0.0F;
    }
  }
}

void MapServerNode::on_mower_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg)
{
  mow_blade_enabled_ = msg->mow_enabled;
}

void MapServerNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!mow_blade_enabled_) {
    return;
  }

  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    mark_cells_mowed(x, y);
  }
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

    // Publish full grid map
    auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(map_);
    grid_map_pub_->publish(std::move(grid_map_msg));

    // Publish mow_progress as OccupancyGrid for visualisation
    mow_progress_pub_->publish(mow_progress_to_occupancy_grid());
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Service callbacks
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::on_save_map(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (map_file_path_.empty()) {
    res->success = false;
    res->message = "map_file_path parameter is empty; cannot save.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try {
    std::lock_guard<std::mutex> lock(map_mutex_);

    // Serialise using grid_map bag file support (YAML sidecar + binary data).
    // grid_map_ros provides rosbag-based I/O; we use a simpler approach:
    // store the OccupancyGrid for occupancy and CSV for other layers so the
    // file remains human-readable and does not require rosbag2 at runtime.

    const std::string yaml_path = map_file_path_ + ".yaml";
    const std::string data_path = map_file_path_ + ".dat";

    // Write metadata YAML
    std::ofstream yaml(yaml_path);
    if (!yaml.is_open()) {
      throw std::runtime_error("Cannot open " + yaml_path + " for writing");
    }
    yaml << "resolution: "   << resolution_    << "\n"
         << "map_size_x: "   << map_size_x_    << "\n"
         << "map_size_y: "   << map_size_y_    << "\n"
         << "map_frame: "    << map_frame_      << "\n"
         << "rows: "         << map_.getSize()(0) << "\n"
         << "cols: "         << map_.getSize()(1) << "\n"
         << "pos_x: "        << map_.getPosition().x() << "\n"
         << "pos_y: "        << map_.getPosition().y() << "\n";
    yaml.close();

    // Write binary layer data: row-major, all four layers interleaved per cell.
    std::ofstream dat(data_path, std::ios::binary);
    if (!dat.is_open()) {
      throw std::runtime_error("Cannot open " + data_path + " for writing");
    }

    const int rows = map_.getSize()(0);
    const int cols = map_.getSize()(1);

    const auto & occ  = map_[std::string(layers::OCCUPANCY)];
    const auto & cls  = map_[std::string(layers::CLASSIFICATION)];
    const auto & prog = map_[std::string(layers::MOW_PROGRESS)];
    const auto & conf = map_[std::string(layers::CONFIDENCE)];

    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        float vals[4] = {occ(r, c), cls(r, c), prog(r, c), conf(r, c)};
        dat.write(reinterpret_cast<const char *>(vals), sizeof(vals));
      }
    }
    dat.close();

    res->success = true;
    res->message = "Map saved to " + map_file_path_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  } catch (const std::exception & ex) {
    res->success = false;
    res->message = std::string("Save failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void MapServerNode::on_load_map(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (map_file_path_.empty()) {
    res->success = false;
    res->message = "map_file_path parameter is empty; cannot load.";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  try {
    const std::string yaml_path = map_file_path_ + ".yaml";
    const std::string data_path = map_file_path_ + ".dat";

    // Parse YAML sidecar
    std::ifstream yaml(yaml_path);
    if (!yaml.is_open()) {
      throw std::runtime_error("Cannot open " + yaml_path);
    }

    double res_loaded{}, sx{}, sy{};
    std::string frame_loaded{};
    int rows_loaded{}, cols_loaded{};
    double pos_x{}, pos_y{};

    std::string line;
    while (std::getline(yaml, line)) {
      std::istringstream ss(line);
      std::string key;
      if (!(ss >> key)) continue;
      if (key == "resolution:")   ss >> res_loaded;
      else if (key == "map_size_x:") ss >> sx;
      else if (key == "map_size_y:") ss >> sy;
      else if (key == "map_frame:")  ss >> frame_loaded;
      else if (key == "rows:")       ss >> rows_loaded;
      else if (key == "cols:")       ss >> cols_loaded;
      else if (key == "pos_x:")      ss >> pos_x;
      else if (key == "pos_y:")      ss >> pos_y;
    }
    yaml.close();

    if (rows_loaded <= 0 || cols_loaded <= 0) {
      throw std::runtime_error("Invalid map dimensions in " + yaml_path);
    }

    // Rebuild map geometry
    std::lock_guard<std::mutex> lock(map_mutex_);

    resolution_  = res_loaded;
    map_size_x_  = sx;
    map_size_y_  = sy;
    map_frame_   = frame_loaded;

    map_ = grid_map::GridMap(
      {std::string(layers::OCCUPANCY),
       std::string(layers::CLASSIFICATION),
       std::string(layers::MOW_PROGRESS),
       std::string(layers::CONFIDENCE)});

    map_.setFrameId(map_frame_);
    map_.setGeometry(
      grid_map::Length(map_size_x_, map_size_y_),
      resolution_,
      grid_map::Position(pos_x, pos_y));

    // Read binary layer data
    std::ifstream dat(data_path, std::ios::binary);
    if (!dat.is_open()) {
      throw std::runtime_error("Cannot open " + data_path);
    }

    auto & occ  = map_[std::string(layers::OCCUPANCY)];
    auto & cls  = map_[std::string(layers::CLASSIFICATION)];
    auto & prog = map_[std::string(layers::MOW_PROGRESS)];
    auto & conf = map_[std::string(layers::CONFIDENCE)];

    const int actual_rows = map_.getSize()(0);
    const int actual_cols = map_.getSize()(1);

    for (int r = 0; r < actual_rows && r < rows_loaded; ++r) {
      for (int c = 0; c < actual_cols && c < cols_loaded; ++c) {
        float vals[4] = {};
        dat.read(reinterpret_cast<char *>(vals), sizeof(vals));
        occ(r, c)  = vals[0];
        cls(r, c)  = vals[1];
        prog(r, c) = vals[2];
        conf(r, c) = vals[3];
      }
    }
    dat.close();

    last_decay_time_ = now();

    res->success = true;
    res->message = "Map loaded from " + map_file_path_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  } catch (const std::exception & ex) {
    res->success = false;
    res->message = std::string("Load failed: ") + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void MapServerNode::on_clear_map(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    clear_map_layers();
  }
  res->success = true;
  res->message = "All map layers cleared.";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void MapServerNode::on_add_no_go_zone(
  const mowgli_interfaces::srv::AddMowingArea::Request::SharedPtr req,
  mowgli_interfaces::srv::AddMowingArea::Response::SharedPtr res)
{
  const auto & polygon_msg = req->area.area;

  if (polygon_msg.points.size() < 3) {
    res->success = false;
    RCLCPP_WARN(get_logger(), "add_no_go_zone: polygon must have at least 3 points.");
    return;
  }

  // Build grid_map polygon from geometry_msgs polygon
  grid_map::Polygon gm_polygon;
  for (const auto & pt : polygon_msg.points) {
    gm_polygon.addVertex(
      grid_map::Position(static_cast<double>(pt.x), static_cast<double>(pt.y)));
  }

  const float no_go_val = static_cast<float>(CellType::NO_GO_ZONE);

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    for (grid_map::PolygonIterator it(map_, gm_polygon); !it.isPastEnd(); ++it) {
      map_.at(std::string(layers::CLASSIFICATION), *it) = no_go_val;
    }
  }

  // Cache the polygon if it represents the primary mowing area
  if (!req->is_navigation_area) {
    mowing_area_polygon_ = polygon_msg;
  }

  res->success = true;
  RCLCPP_INFO(
    get_logger(), "No-go zone with %zu vertices applied.",
    polygon_msg.points.size());
}

void MapServerNode::on_get_mowing_area(
  const mowgli_interfaces::srv::GetMowingArea::Request::SharedPtr /*req*/,
  mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr res)
{
  res->area.area   = mowing_area_polygon_;
  res->area.name   = "mowing_area";
  res->success     = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

void MapServerNode::clear_map_layers()
{
  // Caller must hold map_mutex_.
  map_[std::string(layers::OCCUPANCY)].setConstant(defaults::OCCUPANCY);
  map_[std::string(layers::CLASSIFICATION)].setConstant(defaults::CLASSIFICATION);
  map_[std::string(layers::MOW_PROGRESS)].setConstant(defaults::MOW_PROGRESS);
  map_[std::string(layers::CONFIDENCE)].setConstant(defaults::CONFIDENCE);
}

nav_msgs::msg::OccupancyGrid MapServerNode::mow_progress_to_occupancy_grid() const
{
  // Caller must hold map_mutex_.
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp    = now();
  grid.header.frame_id = map_frame_;
  grid.info.resolution = static_cast<float>(resolution_);
  grid.info.width      = static_cast<uint32_t>(map_.getSize()(1));
  grid.info.height     = static_cast<uint32_t>(map_.getSize()(0));

  // Set origin at bottom-left corner
  grid.info.origin.position.x = map_.getPosition().x() - map_size_x_ * 0.5;
  grid.info.origin.position.y = map_.getPosition().y() - map_size_y_ * 0.5;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  const auto & prog = map_[std::string(layers::MOW_PROGRESS)];
  const int rows = map_.getSize()(0);
  const int cols = map_.getSize()(1);

  grid.data.resize(static_cast<std::size_t>(rows * cols), 0);

  // grid_map stores data column-major; OccupancyGrid is row-major.
  // grid_map row 0 is the top of the map; OccupancyGrid row 0 is the bottom.
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const float val = prog(r, c);
      // Flip row axis: OccupancyGrid bottom row = grid_map top row
      const int og_row = rows - 1 - r;
      const auto idx = static_cast<std::size_t>(og_row * cols + c);
      const float clamped = std::clamp(val, 0.0F, 1.0F);
      grid.data[idx] = static_cast<int8_t>(std::lround(clamped * 100.0F));
    }
  }

  return grid;
}

void MapServerNode::apply_decay(double elapsed_seconds)
{
  // Caller must hold map_mutex_.
  if (elapsed_seconds <= 0.0 || decay_rate_per_hour_ <= 0.0) {
    return;
  }

  const double decay_per_second = decay_rate_per_hour_ / 3600.0;
  const float decay = static_cast<float>(decay_per_second * elapsed_seconds);

  auto & prog = map_[std::string(layers::MOW_PROGRESS)];
  prog = (prog.array() - decay).max(0.0F).matrix();
}

void MapServerNode::mark_cells_mowed(double x, double y)
{
  // Caller must hold map_mutex_.
  const grid_map::Position center(x, y);
  const double radius = mower_width_ * 0.5;

  for (grid_map::CircleIterator it(map_, center, radius); !it.isPastEnd(); ++it) {
    map_.at(std::string(layers::MOW_PROGRESS), *it) = 1.0F;
    map_.at(std::string(layers::CONFIDENCE),   *it) += 1.0F;
  }
}

bool MapServerNode::point_in_polygon(
  const geometry_msgs::msg::Point32 & pt,
  const geometry_msgs::msg::Polygon & polygon) noexcept
{
  const auto & pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 3) {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const float xi = pts[i].x, yi = pts[i].y;
    const float xj = pts[j].x, yj = pts[j].y;

    const bool intersect =
      ((yi > pt.y) != (yj > pt.y)) &&
      (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi);

    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
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

}  // namespace mowgli_map
