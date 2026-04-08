// Copyright 2026 Mowgli Project
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
#include "mowgli_brv_planner/grid_coverage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "mowgli_brv_planner/geometry_utils.hpp"

namespace brv
{

CoverageGrid::CoverageGrid(const Polygon2D& boundary,
                           const std::vector<Polygon2D>& obstacles,
                           double resolution,
                           double sweep_angle)
    : resolution_(resolution),
      sweep_angle_(sweep_angle),
      cos_a_(std::cos(-sweep_angle)),
      sin_a_(std::sin(-sweep_angle))
{
  // Rotate boundary into grid-aligned frame
  Polygon2D rot_boundary = rotate_polygon(boundary);

  // Compute AABB of rotated boundary
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();
  for (const auto& p : rot_boundary)
  {
    x_min = std::min(x_min, p.x);
    x_max = std::max(x_max, p.x);
    y_min = std::min(y_min, p.y);
    y_max = std::max(y_max, p.y);
  }

  grid_ox_ = x_min;
  grid_oy_ = y_min;
  cols_ = std::max(1, static_cast<int>(std::ceil((x_max - x_min) / resolution)));
  rows_ = std::max(1, static_cast<int>(std::ceil((y_max - y_min) / resolution)));
  grid_.resize(rows_ * cols_, CellState::UNVISITED);

  init_grid(boundary, obstacles);
}

Polygon2D CoverageGrid::rotate_polygon(const Polygon2D& poly) const
{
  Polygon2D result;
  result.reserve(poly.size());
  for (const auto& p : poly)
  {
    result.push_back({cos_a_ * p.x - sin_a_ * p.y, sin_a_ * p.x + cos_a_ * p.y});
  }
  return result;
}

void CoverageGrid::init_grid(const Polygon2D& boundary, const std::vector<Polygon2D>& obstacles)
{
  for (int r = 0; r < rows_; ++r)
  {
    for (int c = 0; c < cols_; ++c)
    {
      // Cell center in rotated frame
      double rx = grid_ox_ + (c + 0.5) * resolution_;
      double ry = grid_oy_ + (r + 0.5) * resolution_;

      // Unrotate to map frame for polygon test
      double mx = cos_a_ * rx + sin_a_ * ry;  // cos(-(-a)) = cos(a), sin(-(-a)) = -sin(-a) = sin(a)
      double my = -sin_a_ * rx + cos_a_ * ry;

      Point2D map_pt{mx, my};

      if (!point_in_polygon(map_pt, boundary))
      {
        grid_[r * cols_ + c] = CellState::UNVISITABLE;
        continue;
      }

      for (const auto& obs : obstacles)
      {
        if (point_in_polygon(map_pt, obs))
        {
          grid_[r * cols_ + c] = CellState::UNVISITABLE;
          break;
        }
      }
    }
  }
}

Point2D CoverageGrid::cell_to_map(int row, int col) const
{
  double rx = grid_ox_ + (col + 0.5) * resolution_;
  double ry = grid_oy_ + (row + 0.5) * resolution_;
  // Unrotate: rotate by +sweep_angle
  double mx = cos_a_ * rx + sin_a_ * ry;
  double my = -sin_a_ * rx + cos_a_ * ry;
  return {mx, my};
}

std::pair<int, int> CoverageGrid::map_to_cell(double x, double y) const
{
  // Rotate map point into grid-aligned frame
  double rx = cos_a_ * x - sin_a_ * y;
  double ry = sin_a_ * x + cos_a_ * y;
  int col = static_cast<int>((rx - grid_ox_) / resolution_);
  int row = static_cast<int>((ry - grid_oy_) / resolution_);
  return {row, col};
}

bool CoverageGrid::has_unvisited() const
{
  for (auto s : grid_)
  {
    if (s == CellState::UNVISITED)
      return true;
  }
  return false;
}

int CoverageGrid::count_unvisited() const
{
  int count = 0;
  for (auto s : grid_)
  {
    if (s == CellState::UNVISITED)
      ++count;
  }
  return count;
}

std::pair<int, int> CoverageGrid::find_nearest_unvisited(int row, int col) const
{
  int best_r = -1, best_c = -1;
  int best_dist = std::numeric_limits<int>::max();
  for (int r = 0; r < rows_; ++r)
  {
    for (int c = 0; c < cols_; ++c)
    {
      if (grid_[r * cols_ + c] == CellState::UNVISITED)
      {
        int d = std::abs(r - row) + std::abs(c - col);
        if (d < best_dist)
        {
          best_dist = d;
          best_r = r;
          best_c = c;
        }
      }
    }
  }
  return {best_r, best_c};
}

std::vector<std::pair<int, int>> CoverageGrid::get_unvisited_region_starts() const
{
  std::vector<bool> visited(rows_ * cols_, false);
  std::vector<std::pair<int, int>> regions;

  for (int r = 0; r < rows_; ++r)
  {
    for (int c = 0; c < cols_; ++c)
    {
      int idx = r * cols_ + c;
      if (grid_[idx] != CellState::UNVISITED || visited[idx])
        continue;

      // BFS to find first cell of this region (topmost-leftmost)
      int best_r = r, best_c = c;
      std::queue<std::pair<int, int>> q;
      q.push({r, c});
      visited[idx] = true;

      while (!q.empty())
      {
        auto [cr, cc] = q.front();
        q.pop();
        if (cr < best_r || (cr == best_r && cc < best_c))
        {
          best_r = cr;
          best_c = cc;
        }
        static const int dr[] = {-1, 1, 0, 0};
        static const int dc[] = {0, 0, -1, 1};
        for (int d = 0; d < 4; ++d)
        {
          int nr = cr + dr[d];
          int nc = cc + dc[d];
          if (nr < 0 || nr >= rows_ || nc < 0 || nc >= cols_)
            continue;
          int ni = nr * cols_ + nc;
          if (!visited[ni] && grid_[ni] == CellState::UNVISITED)
          {
            visited[ni] = true;
            q.push({nr, nc});
          }
        }
      }
      regions.push_back({best_r, best_c});
    }
  }
  return regions;
}

SweepResult boustrophedon_sweep(CoverageGrid& grid, int start_row, int start_col)
{
  SweepResult result;
  result.swath_breaks.push_back(0);

  int rows = grid.rows();
  int cols = grid.cols();
  int row = start_row;
  int row_dir = 1;
  int col_dir = 1;

  // Determine initial row direction: sweep toward the side with more unvisited
  int up_count = 0, down_count = 0;
  for (int r = 0; r < row; ++r)
  {
    if (grid.cell(r, start_col) == CellState::UNVISITED)
      ++up_count;
  }
  for (int r = row; r < rows; ++r)
  {
    if (grid.cell(r, start_col) == CellState::UNVISITED)
      ++down_count;
  }
  row_dir = (down_count >= up_count) ? 1 : -1;

  // Sweep ALL columns from start_col in col_dir direction.
  // Each column is swept fully (top-to-bottom or bottom-to-top, alternating).
  // Obstacles within a column create swath breaks but don't stop the sweep.
  // Empty columns are skipped (up to 5 column gap).
  constexpr int max_skip_cols = 5;
  int col = start_col;
  bool first_col = true;

  while (col >= 0 && col < cols)
  {
    // Check if this column has any unvisited cells
    bool col_has_unvisited = false;
    for (int r = 0; r < rows; ++r)
    {
      if (grid.cell(r, col) == CellState::UNVISITED)
      {
        col_has_unvisited = true;
        break;
      }
    }

    if (!col_has_unvisited)
    {
      // Skip empty column — try next ones
      int skip_col = col + col_dir;
      bool found = false;
      for (int skip = 0; skip < max_skip_cols; ++skip)
      {
        if (skip_col < 0 || skip_col >= cols)
          break;
        for (int r = 0; r < rows; ++r)
        {
          if (grid.cell(r, skip_col) == CellState::UNVISITED)
          {
            found = true;
            break;
          }
        }
        if (found)
        {
          col = skip_col;
          col_has_unvisited = true;
          break;
        }
        skip_col += col_dir;
      }
      if (!found)
        break;
    }

    // Sweep this column fully.
    // When an obstacle gap is encountered within a column, insert a detour
    // that routes around the obstacle instead of jumping straight through it.
    bool swept_any = false;
    bool in_segment = false;
    int last_segment_end_row = -1;  // row where previous segment ended
    int r_start = (row_dir == 1) ? 0 : rows - 1;
    int r_end = (row_dir == 1) ? rows : -1;

    // First column: start from the given start_row
    if (first_col)
    {
      r_start = start_row;
      first_col = false;
    }

    for (int r = r_start; r != r_end; r += row_dir)
    {
      if (grid.cell(r, col) == CellState::UNVISITED)
      {
        if (!in_segment && swept_any)
        {
          // Resuming after obstacle gap — insert detour around obstacle.
          // Find the nearest column outside the obstacle to route through.
          // Step sideways past the obstacle, traverse the gap, step back.
          if (last_segment_end_row >= 0)
          {
            // Find a clear column: step away from current column until we find
            // one where the gap rows are NOT unvisitable (i.e., no obstacle).
            int detour_col = col + col_dir;
            bool found_clear = false;
            for (int step = 0; step < 10; ++step)
            {
              if (detour_col < 0 || detour_col >= cols)
                break;
              // Check if this column is clear through the gap
              bool clear = true;
              int gap_start = std::min(last_segment_end_row, r);
              int gap_end = std::max(last_segment_end_row, r);
              for (int gr = gap_start; gr <= gap_end; ++gr)
              {
                if (grid.cell(gr, detour_col) == CellState::UNVISITABLE)
                {
                  clear = false;
                  break;
                }
              }
              if (clear)
              {
                found_clear = true;
                break;
              }
              detour_col += col_dir;
            }

            if (found_clear)
            {
              // Step sideways to clear column
              result.waypoints.push_back(grid.cell_to_map(last_segment_end_row, detour_col));
              // Traverse past the obstacle gap
              result.waypoints.push_back(grid.cell_to_map(r, detour_col));
              // Step back to current column happens with next waypoint
            }
          }

          result.swath_breaks.push_back(static_cast<int>(result.waypoints.size()));
        }
        grid.set_cell(r, col, CellState::VISITED);
        result.waypoints.push_back(grid.cell_to_map(r, col));
        swept_any = true;
        in_segment = true;
        row = r;
      }
      else
      {
        if (in_segment)
          last_segment_end_row = row;  // remember where we stopped
        in_segment = false;
      }
    }

    if (swept_any)
    {
      // Record swath break for next column
      result.swath_breaks.push_back(static_cast<int>(result.waypoints.size()));
    }

    // Advance to next column and reverse row direction
    col += col_dir;
    row_dir = -row_dir;
  }

  // Remove trailing empty swath break
  if (!result.swath_breaks.empty() &&
      result.swath_breaks.back() >= static_cast<int>(result.waypoints.size()))
  {
    result.swath_breaks.pop_back();
  }

  result.end_row = row;
  result.end_col = std::clamp(col - col_dir, 0, cols - 1);
  return result;
}

}  // namespace brv
