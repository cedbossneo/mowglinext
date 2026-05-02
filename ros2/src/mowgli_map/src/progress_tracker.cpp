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

// Mow-progress / coverage-cells visualization, decay, mark-mowed,
// point-in-polygon helper, boundary monitoring, recovery-point service,
// and persistent obstacle diff/update split out of map_server_node.cpp.
// Behaviour (decay rate, mark radius, recovery offset, replan cooldown,
// /coverage_cells convention) is unchanged.

#include <algorithm>
#include <cmath>
#include <limits>

#include <std_msgs/msg/bool.hpp>

#include "mowgli_map/internal_helpers.hpp"
#include "mowgli_map/map_server_node.hpp"
#include <grid_map_core/iterators/CircleIterator.hpp>

namespace mowgli_map
{

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

}  // namespace mowgli_map
