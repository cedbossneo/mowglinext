#pragma once

#include <vector>

#include "mowgli_brv_planner/types.hpp"

namespace brv
{

class CoverageGrid
{
public:
  CoverageGrid(const Polygon2D& boundary,
               const std::vector<Polygon2D>& obstacles,
               double resolution,
               double sweep_angle);

  // Convert grid cell (row, col) to map frame (x, y).
  Point2D cell_to_map(int row, int col) const;

  // Check if any unvisited cells remain.
  bool has_unvisited() const;

  // Count unvisited cells.
  int count_unvisited() const;

  // Find starting cells for contiguous unvisited regions.
  std::vector<std::pair<int, int>> get_unvisited_region_starts() const;

  // Find nearest unvisited cell to (row, col).
  std::pair<int, int> find_nearest_unvisited(int row, int col) const;

  // Grid accessors.
  int rows() const
  {
    return rows_;
  }
  int cols() const
  {
    return cols_;
  }
  CellState cell(int r, int c) const
  {
    return grid_[r * cols_ + c];
  }
  void set_cell(int r, int c, CellState s)
  {
    grid_[r * cols_ + c] = s;
  }

  double resolution() const
  {
    return resolution_;
  }
  double sweep_angle() const
  {
    return sweep_angle_;
  }

private:
  void init_grid(const Polygon2D& boundary, const std::vector<Polygon2D>& obstacles);

  // Rotate polygon into grid-aligned frame.
  Polygon2D rotate_polygon(const Polygon2D& poly) const;

  double resolution_;
  double sweep_angle_;
  double cos_a_, sin_a_;  // rotation by -sweep_angle
  double grid_ox_, grid_oy_;  // origin in rotated frame
  int rows_, cols_;
  std::vector<CellState> grid_;
};

struct SweepResult
{
  Path2D waypoints;
  std::vector<int> swath_breaks;  // indices where new swaths start
  int end_row;
  int end_col;
};

// Execute one boustrophedon sweep from a starting cell.
SweepResult boustrophedon_sweep(CoverageGrid& grid, int start_row, int start_col);

}  // namespace brv
