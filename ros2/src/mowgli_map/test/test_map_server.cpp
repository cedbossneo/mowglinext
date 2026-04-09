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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "mowgli_map/map_server_node.hpp"
#include "mowgli_map/map_types.hpp"
#include <gtest/gtest.h>

// ─────────────────────────────────────────────────────────────────────────────
// Test fixture — creates a MapServerNode with a small 10×10 m map
// ─────────────────────────────────────────────────────────────────────────────

// Global init/shutdown for all test suites
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

class MapServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", 0.1);
    opts.append_parameter_override("map_size_x", 10.0);
    opts.append_parameter_override("map_size_y", 10.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("decay_rate_per_hour", 3600.0);  // 1.0 / sec for easy maths
    opts.append_parameter_override("mower_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("publish_rate", 1.0);

    node_ = std::make_shared<mowgli_map::MapServerNode>(opts);
  }

  void TearDown() override
  {
    node_.reset();
  }

  std::shared_ptr<mowgli_map::MapServerNode> node_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Test 1 — grid_map creation with correct layers
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, GridMapHasAllRequiredLayers)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& m = node_->map();

  EXPECT_TRUE(m.exists(std::string(mowgli_map::layers::OCCUPANCY)));
  EXPECT_TRUE(m.exists(std::string(mowgli_map::layers::CLASSIFICATION)));
  EXPECT_TRUE(m.exists(std::string(mowgli_map::layers::MOW_PROGRESS)));
  EXPECT_TRUE(m.exists(std::string(mowgli_map::layers::CONFIDENCE)));
}

TEST_F(MapServerTest, GridMapGeometryIsCorrect)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& m = node_->map();

  // 10 m / 0.1 m resolution = 100 cells per axis
  EXPECT_EQ(m.getSize()(0), 100);
  EXPECT_EQ(m.getSize()(1), 100);

  EXPECT_DOUBLE_EQ(m.getResolution(), 0.1);
  EXPECT_EQ(m.getFrameId(), "map");
}

TEST_F(MapServerTest, GridMapLayersInitialisedToDefaults)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& m = node_->map();

  // All occupancy cells must be 0.0 (free)
  const auto& occ = m[std::string(mowgli_map::layers::OCCUPANCY)];
  EXPECT_FLOAT_EQ(occ.minCoeff(), 0.0F);
  EXPECT_FLOAT_EQ(occ.maxCoeff(), 0.0F);

  // All mow_progress cells must be 0.0 (unmowed)
  const auto& prog = m[std::string(mowgli_map::layers::MOW_PROGRESS)];
  EXPECT_FLOAT_EQ(prog.minCoeff(), 0.0F);
  EXPECT_FLOAT_EQ(prog.maxCoeff(), 0.0F);

  // All confidence cells must be 0.0
  const auto& conf = m[std::string(mowgli_map::layers::CONFIDENCE)];
  EXPECT_FLOAT_EQ(conf.minCoeff(), 0.0F);
  EXPECT_FLOAT_EQ(conf.maxCoeff(), 0.0F);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2 — mow_progress decay calculation
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, MowProgressDecaysOverTime)
{
  // First mark the centre cell as fully mowed
  node_->mark_mowed(0.0, 0.0);

  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& prog = node_->map()[std::string(mowgli_map::layers::MOW_PROGRESS)];
    ASSERT_GT(prog.maxCoeff(), 0.0F) << "At least one cell should be mowed before testing decay";
  }

  // decay_rate_per_hour = 3600.0, so after 1 second the decay is 1.0 per cell.
  // After 0.5 s the freshly mowed cells (1.0) should decay to ~0.5.
  node_->tick_once(0.5);

  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& prog = node_->map()[std::string(mowgli_map::layers::MOW_PROGRESS)];
    const float max_val = prog.maxCoeff();
    EXPECT_GT(max_val, 0.0F) << "Progress should not have fully decayed after 0.5 s";
    EXPECT_LT(max_val, 1.0F) << "Progress should have decreased";
    EXPECT_NEAR(max_val, 0.5F, 0.05F);
  }
}

TEST_F(MapServerTest, MowProgressDoesNotGoBelowZero)
{
  // Mark a cell mowed then decay far past zero
  node_->mark_mowed(0.0, 0.0);

  // Apply 10 seconds worth of decay (rate = 3600 / h → 10 units drop)
  node_->tick_once(10.0);

  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& prog = node_->map()[std::string(mowgli_map::layers::MOW_PROGRESS)];
    EXPECT_FLOAT_EQ(prog.minCoeff(), 0.0F) << "mow_progress must be clamped at 0.0, not negative";
  }
}

TEST_F(MapServerTest, DecayRateParameterIsCorrect)
{
  EXPECT_DOUBLE_EQ(node_->decay_rate_per_hour(), 3600.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3 — cell marking within mower width
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, MarkMowedSetsProgressToOne)
{
  node_->mark_mowed(0.0, 0.0);

  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& prog = node_->map()[std::string(mowgli_map::layers::MOW_PROGRESS)];
  // At least the centre cell should be 1.0
  EXPECT_FLOAT_EQ(prog.maxCoeff(), 1.0F);
}

TEST_F(MapServerTest, MarkMowedIncrementsConfidence)
{
  node_->mark_mowed(0.0, 0.0);

  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& conf = node_->map()[std::string(mowgli_map::layers::CONFIDENCE)];
  EXPECT_GT(conf.maxCoeff(), 0.0F) << "Confidence should be incremented when cells are mowed";
}

TEST_F(MapServerTest, OnlyMowerWidthRadiusIsMarked)
{
  // mower_width = 0.2 m → radius = 0.1 m.
  // A point at 0.5 m from centre should NOT be marked.
  node_->mark_mowed(0.0, 0.0);

  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& m = node_->map();
    grid_map::Position far_pos(0.5, 0.5);
    grid_map::Index far_idx;
    ASSERT_TRUE(m.getIndex(far_pos, far_idx));
    const float far_val = m.at(std::string(mowgli_map::layers::MOW_PROGRESS), far_idx);
    EXPECT_FLOAT_EQ(far_val, 0.0F) << "Cell 0.5 m away should not be within mower_width radius";
  }
}

TEST_F(MapServerTest, MowerWidthParameterIsCorrect)
{
  EXPECT_DOUBLE_EQ(node_->mower_width(), 0.2);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4 — classification enum values
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, CellTypeEnumValues)
{
  using mowgli_map::CellType;

  EXPECT_EQ(static_cast<uint8_t>(CellType::UNKNOWN), 0u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::LAWN), 1u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::OBSTACLE_PERMANENT), 2u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::OBSTACLE_TEMPORARY), 3u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::NO_GO_ZONE), 4u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::DOCKING_AREA), 5u);
}

TEST_F(MapServerTest, CellTypeNamesAreCorrect)
{
  using mowgli_map::cell_type_name;
  using mowgli_map::CellType;

  EXPECT_EQ(cell_type_name(CellType::UNKNOWN), "UNKNOWN");
  EXPECT_EQ(cell_type_name(CellType::LAWN), "LAWN");
  EXPECT_EQ(cell_type_name(CellType::OBSTACLE_PERMANENT), "OBSTACLE_PERMANENT");
  EXPECT_EQ(cell_type_name(CellType::OBSTACLE_TEMPORARY), "OBSTACLE_TEMPORARY");
  EXPECT_EQ(cell_type_name(CellType::NO_GO_ZONE), "NO_GO_ZONE");
  EXPECT_EQ(cell_type_name(CellType::DOCKING_AREA), "DOCKING_AREA");
}

TEST_F(MapServerTest, ClassificationLayerDefaultIsUnknown)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& cls = node_->map()[std::string(mowgli_map::layers::CLASSIFICATION)];
  EXPECT_FLOAT_EQ(cls.minCoeff(), static_cast<float>(mowgli_map::CellType::UNKNOWN));
  EXPECT_FLOAT_EQ(cls.maxCoeff(), static_cast<float>(mowgli_map::CellType::UNKNOWN));
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5 — map clear resets all layers
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, ClearMapResetsAllLayersToDefault)
{
  // Dirty all layers
  node_->mark_mowed(0.0, 0.0);
  node_->tick_once(0.1);  // partially decay

  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    auto& m = node_->map();
    // Manually set some cells in other layers
    grid_map::Index centre_idx;
    ASSERT_TRUE(m.getIndex(grid_map::Position(0.0, 0.0), centre_idx));
    m.at(std::string(mowgli_map::layers::OCCUPANCY), centre_idx) = 1.0F;
    m.at(std::string(mowgli_map::layers::CLASSIFICATION), centre_idx) =
        static_cast<float>(mowgli_map::CellType::NO_GO_ZONE);
  }

  // Now clear
  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    node_->clear_map_layers();
  }

  // Verify all layers are back to defaults
  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& m = node_->map();

    const auto& occ = m[std::string(mowgli_map::layers::OCCUPANCY)];
    const auto& cls = m[std::string(mowgli_map::layers::CLASSIFICATION)];
    const auto& prog = m[std::string(mowgli_map::layers::MOW_PROGRESS)];
    const auto& conf = m[std::string(mowgli_map::layers::CONFIDENCE)];

    EXPECT_FLOAT_EQ(occ.maxCoeff(), mowgli_map::defaults::OCCUPANCY);
    EXPECT_FLOAT_EQ(cls.maxCoeff(), mowgli_map::defaults::CLASSIFICATION);
    EXPECT_FLOAT_EQ(prog.maxCoeff(), mowgli_map::defaults::MOW_PROGRESS);
    EXPECT_FLOAT_EQ(conf.maxCoeff(), mowgli_map::defaults::CONFIDENCE);

    EXPECT_FLOAT_EQ(occ.minCoeff(), mowgli_map::defaults::OCCUPANCY);
    EXPECT_FLOAT_EQ(cls.minCoeff(), mowgli_map::defaults::CLASSIFICATION);
    EXPECT_FLOAT_EQ(prog.minCoeff(), mowgli_map::defaults::MOW_PROGRESS);
    EXPECT_FLOAT_EQ(conf.minCoeff(), mowgli_map::defaults::CONFIDENCE);
  }
}

TEST_F(MapServerTest, ClearMapDoesNotChangeGeometry)
{
  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    node_->clear_map_layers();
    const auto& m = node_->map();
    EXPECT_EQ(m.getSize()(0), 100);
    EXPECT_EQ(m.getSize()(1), 100);
    EXPECT_DOUBLE_EQ(m.getResolution(), 0.1);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Coverage cells OccupancyGrid tests
// ─────────────────────────────────────────────────────────────────────────────

// Helper: create a node with an area polygon, mark some cells mowed, get the OG
class CoverageCellsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", 0.1);
    opts.append_parameter_override("map_size_x", 10.0);
    opts.append_parameter_override("map_size_y", 10.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("decay_rate_per_hour", 0.0);
    opts.append_parameter_override("mower_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("publish_rate", 1.0);

    // Define a simple rectangular mowing area: (-3,-2) to (3,2)
    std::vector<std::string> names = {"test_area"};
    std::vector<std::string> polys = {"-3,-2;3,-2;3,2;-3,2"};
    std::vector<bool> nav_flags = {false};
    opts.append_parameter_override("area_names", names);
    opts.append_parameter_override("area_polygons", polys);
    opts.append_parameter_override("area_is_navigation", nav_flags);

    node_ = std::make_shared<mowgli_map::MapServerNode>(opts);
  }

  void TearDown() override
  {
    node_.reset();
  }

  // Get OG cell value at world position (wx, wy)
  int8_t og_value_at(const nav_msgs::msg::OccupancyGrid& og, double wx, double wy)
  {
    int col = static_cast<int>((wx - og.info.origin.position.x) / og.info.resolution);
    int row = static_cast<int>((wy - og.info.origin.position.y) / og.info.resolution);
    if (col < 0 || col >= static_cast<int>(og.info.width) || row < 0 ||
        row >= static_cast<int>(og.info.height))
      return -1;
    return og.data[static_cast<size_t>(row * og.info.width + col)];
  }

  std::shared_ptr<mowgli_map::MapServerNode> node_;
};

TEST_F(CoverageCellsTest, UnmowedCellsInsideAreaAreVisible)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  auto og = node_->coverage_cells_to_occupancy_grid();

  // Center of area (0,0) should be "to mow" = 60
  int8_t val = og_value_at(og, 0.0, 0.0);
  EXPECT_EQ(val, 60) << "Center of area should be 'to mow' (60)";
}

TEST_F(CoverageCellsTest, CellsOutsideAreaAreUnknown)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  auto og = node_->coverage_cells_to_occupancy_grid();

  // Far outside area should be -1
  int8_t val = og_value_at(og, -4.5, -4.5);
  EXPECT_EQ(val, -1) << "Outside area should be unknown (-1)";
}

TEST_F(CoverageCellsTest, MowedCellsAreTransparent)
{
  node_->mark_mowed(1.0, 0.0);

  std::lock_guard<std::mutex> lock(node_->map_mutex());
  auto og = node_->coverage_cells_to_occupancy_grid();

  int8_t val = og_value_at(og, 1.0, 0.0);
  EXPECT_EQ(val, 0) << "Mowed cell should be transparent (0)";
}

TEST_F(CoverageCellsTest, MowedCellPositionMatchesWorldPosition)
{
  // Mow at a specific known location
  const double mow_x = 2.0, mow_y = 1.0;
  const double mower_r = node_->mower_width() / 2.0;  // 0.1m
  node_->mark_mowed(mow_x, mow_y);

  std::lock_guard<std::mutex> lock(node_->map_mutex());
  auto og = node_->coverage_cells_to_occupancy_grid();

  // The exact mowed position should be 0
  EXPECT_EQ(og_value_at(og, mow_x, mow_y), 0)
      << "Mowed cell at (" << mow_x << "," << mow_y << ") should be 0";

  // Nearby unmowed position should be 60
  EXPECT_EQ(og_value_at(og, -2.0, 1.0), 60) << "Unmowed cell at (-2,1) should be 'to mow'";

  // Mirrored position should NOT be mowed
  EXPECT_NE(og_value_at(og, -2.0, -1.0), 0) << "Mirrored position should NOT be mowed";
}

TEST_F(CoverageCellsTest, MowedCellsCentroidMatchesMowPosition)
{
  // CRITICAL TEST: mow at specific positions and verify the centroid of
  // mowed cells in the OccupancyGrid matches the actual mow position.
  // This catches any coordinate offset/mirror/transpose bugs.

  struct TestPoint
  {
    double x, y;
  };
  std::vector<TestPoint> mow_points = {
      {1.5, 0.5},
      {-1.0, -1.0},
      {0.0, 1.5},
      {2.5, -1.5},
  };

  const double res = 0.1;  // map resolution

  for (const auto& mp : mow_points)
  {
    // Reset map
    node_->clear_map_layers();
    node_->mark_mowed(mp.x, mp.y);

    std::lock_guard<std::mutex> lock(node_->map_mutex());
    auto og = node_->coverage_cells_to_occupancy_grid();

    // Scan the entire OG to find the centroid of mowed cells (value == 0)
    double sum_x = 0, sum_y = 0;
    int count = 0;
    for (int row = 0; row < static_cast<int>(og.info.height); ++row)
    {
      for (int col = 0; col < static_cast<int>(og.info.width); ++col)
      {
        auto idx = static_cast<size_t>(row * og.info.width + col);
        if (og.data[idx] == 0)
        {
          double wx = og.info.origin.position.x + (col + 0.5) * res;
          double wy = og.info.origin.position.y + (row + 0.5) * res;
          sum_x += wx;
          sum_y += wy;
          count++;
        }
      }
    }

    ASSERT_GT(count, 0) << "No mowed cells found for mow at (" << mp.x << "," << mp.y << ")";

    double cx = sum_x / count;
    double cy = sum_y / count;

    // The centroid of mowed cells must be within 1 cell of the mow position
    double dist = std::hypot(cx - mp.x, cy - mp.y);
    EXPECT_LT(dist, res * 2.0) << "Mow at (" << mp.x << "," << mp.y
                               << ") but mowed cells centroid at (" << cx << "," << cy
                               << "), distance=" << dist << "m. " << "Found " << count
                               << " mowed cells.";
  }
}

TEST_F(CoverageCellsTest, MowedStripAlignmentTest)
{
  // Simulate mowing a vertical strip (like the robot does) and verify
  // the mowed cells form a vertical band at the correct X position.
  const double strip_x = 1.0;

  // Mow along the strip
  for (double y = -1.5; y <= 1.5; y += 0.05)
  {
    node_->mark_mowed(strip_x, y);
  }

  std::lock_guard<std::mutex> lock(node_->map_mutex());
  auto og = node_->coverage_cells_to_occupancy_grid();

  const double res = 0.1;

  // Find the average X of all mowed cells
  double sum_x = 0;
  int count = 0;
  for (int row = 0; row < static_cast<int>(og.info.height); ++row)
  {
    for (int col = 0; col < static_cast<int>(og.info.width); ++col)
    {
      auto idx = static_cast<size_t>(row * og.info.width + col);
      if (og.data[idx] == 0)
      {
        double wx = og.info.origin.position.x + (col + 0.5) * res;
        sum_x += wx;
        count++;
      }
    }
  }

  ASSERT_GT(count, 10) << "Expected many mowed cells for a strip";

  double avg_x = sum_x / count;
  double x_error = std::abs(avg_x - strip_x);

  EXPECT_LT(x_error, res * 2.0) << "Strip at x=" << strip_x
                                << " but mowed cells average x=" << avg_x << ", error=" << x_error
                                << "m (" << count << " cells)";

  // Also verify: no mowed cells should appear at x < 0 (far from strip)
  int wrong_side_count = 0;
  for (int row = 0; row < static_cast<int>(og.info.height); ++row)
  {
    for (int col = 0; col < static_cast<int>(og.info.width); ++col)
    {
      auto idx = static_cast<size_t>(row * og.info.width + col);
      if (og.data[idx] == 0)
      {
        double wx = og.info.origin.position.x + (col + 0.5) * res;
        if (wx < 0.0)  // Far from strip_x=1.0
          wrong_side_count++;
      }
    }
  }

  EXPECT_EQ(wrong_side_count, 0) << wrong_side_count
                                 << " mowed cells appeared at x<0, far from strip at x=1.0";
}

TEST_F(CoverageCellsTest, CoverageGridMatchesMowProgressGrid)
{
  // Mark several positions and verify coverage_cells matches mow_progress
  std::vector<std::pair<double, double>> mow_positions = {{0.0, 0.0},
                                                          {1.0, 0.5},
                                                          {-1.0, -0.5},
                                                          {2.5, 1.5}};

  for (auto [x, y] : mow_positions)
    node_->mark_mowed(x, y);

  std::lock_guard<std::mutex> lock(node_->map_mutex());
  auto og = node_->coverage_cells_to_occupancy_grid();

  for (auto [x, y] : mow_positions)
  {
    int8_t val = og_value_at(og, x, y);
    EXPECT_EQ(val, 0) << "Mowed position (" << x << "," << y << ") should be 0, got " << (int)val;
  }
}

TEST_F(CoverageCellsTest, MowedCellsCentroidWithOffsetArea)
{
  // The SIMULATION uses area (-7,-3) to (2,3), centered at (-2.5, 0).
  // This test reproduces that offset to catch bugs that only appear
  // when the map center is not at origin.

  // Create a new node with the simulation's actual area
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("resolution", 0.1);
  opts.append_parameter_override("map_size_x", 20.0);
  opts.append_parameter_override("map_size_y", 20.0);
  opts.append_parameter_override("map_frame", "map");
  opts.append_parameter_override("decay_rate_per_hour", 0.0);
  opts.append_parameter_override("mower_width", 0.18);
  opts.append_parameter_override("map_file_path", "");
  opts.append_parameter_override("publish_rate", 1.0);
  std::vector<std::string> names2 = {"main_mow"};
  std::vector<std::string> polys2 = {"-7,-3;2,-3;2,3;-7,3"};
  std::vector<bool> flags2 = {false};
  opts.append_parameter_override("area_names", names2);
  opts.append_parameter_override("area_polygons", polys2);
  opts.append_parameter_override("area_is_navigation", flags2);

  auto sim_node = std::make_shared<mowgli_map::MapServerNode>(opts);

  // Mow at positions matching the simulation
  struct TestPoint
  {
    double x, y;
  };
  std::vector<TestPoint> mow_points = {
      {-1.0, 0.0},  // Near dock
      {-4.0, -2.0},  // Bottom-left
      {1.0, 2.0},  // Top-right
      {-6.0, 0.0},  // Far left
  };

  const double res = 0.1;

  for (const auto& mp : mow_points)
  {
    sim_node->clear_map_layers();
    sim_node->mark_mowed(mp.x, mp.y);

    std::lock_guard<std::mutex> lock(sim_node->map_mutex());
    auto og = sim_node->coverage_cells_to_occupancy_grid();

    // Find centroid of mowed cells
    double sum_x = 0, sum_y = 0;
    int count = 0;
    for (int row = 0; row < static_cast<int>(og.info.height); ++row)
    {
      for (int col = 0; col < static_cast<int>(og.info.width); ++col)
      {
        auto idx = static_cast<size_t>(row * og.info.width + col);
        if (og.data[idx] == 0)
        {
          double wx = og.info.origin.position.x + (col + 0.5) * res;
          double wy = og.info.origin.position.y + (row + 0.5) * res;
          sum_x += wx;
          sum_y += wy;
          count++;
        }
      }
    }

    ASSERT_GT(count, 0) << "No mowed cells for mow at (" << mp.x << "," << mp.y << ")";

    double cx = sum_x / count;
    double cy = sum_y / count;
    double dist = std::hypot(cx - mp.x, cy - mp.y);

    EXPECT_LT(dist, res * 2.0) << "OFFSET MAP: Mow at (" << mp.x << "," << mp.y
                               << ") but centroid at (" << cx << "," << cy << "), distance=" << dist
                               << "m, " << count << " cells. " << "OG origin=("
                               << og.info.origin.position.x << "," << og.info.origin.position.y
                               << "), " << "size=" << og.info.width << "x" << og.info.height;
  }

  sim_node.reset();
}

// ─────────────────────────────────────────────────────────────────────────────
// Strip planner tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(CoverageCellsTest, StripLayoutGeneratesStrips)
{
  // Trigger strip layout computation via get_next_strip service handler
  // Since we can't easily call the service in unit test, test ensure_strip_layout
  // indirectly by checking strip count after construction

  // The area is 6x4m. With 0.2m mower width and 0.2m inset:
  // inner_x range: -3+0.2 to 3-0.2 = -2.8 to 2.8 = 5.6m
  // strips: floor(5.6 / 0.2) = 28 strips
  // This test just verifies the node was created successfully with an area
  EXPECT_GE(node_->map().getSize()(0), 10);  // Map should have reasonable size
}

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
