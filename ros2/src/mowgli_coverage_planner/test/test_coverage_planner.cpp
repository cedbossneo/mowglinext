// Copyright (C) 2024 Cedric
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * @file test_coverage_planner.cpp
 * @brief Integration tests for CoveragePlannerNode action server.
 *
 * Tests spin the CoveragePlannerNode together with a lightweight client node
 * inside a MultiThreadedExecutor running on a background thread.  Each test
 * sends an action goal and verifies the geometric properties of the returned
 * paths.
 *
 * All tests use a simple 10x10 m² square boundary unless stated otherwise.
 */

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "gtest/gtest.h"
#include "mowgli_coverage_planner/coverage_planner_node.hpp"
#include "mowgli_coverage_planner/polygon_utils.hpp"
#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using mowgli_coverage_planner::CoveragePlannerNode;
using PlanCoverageAction = mowgli_interfaces::action::PlanCoverage;
using GoalHandle = rclcpp_action::ClientGoalHandle<PlanCoverageAction>;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class CoveragePlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Build the planner node with test-friendly parameters.
    rclcpp::NodeOptions planner_opts;
    planner_opts.append_parameter_override("tool_width", 0.5);
    planner_opts.append_parameter_override("headland_passes", 1);
    planner_opts.append_parameter_override("headland_width", 0.5);
    planner_opts.append_parameter_override("path_spacing", 0.5);
    planner_opts.append_parameter_override("map_frame", "map");
    planner_opts.append_parameter_override("min_turning_radius", 0.3);
    planner_node_ = std::make_shared<CoveragePlannerNode>(planner_opts);

    // Build a minimal client node.
    client_node_ = rclcpp::Node::make_shared("test_client_node");
    action_client_ =
        rclcpp_action::create_client<PlanCoverageAction>(client_node_, "/mowgli/coverage/plan");

    // Spin both nodes on a background thread.
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(planner_node_);
    executor_->add_node(client_node_);

    spin_thread_ = std::thread(
        [this]()
        {
          executor_->spin();
        });

    // Wait for action server to be ready.
    ASSERT_TRUE(action_client_->wait_for_action_server(5s))
        << "PlanCoverage action server did not become available within 5 s";
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
  }

  // ---- Helpers ------------------------------------------------------------

  /// Build a 10x10 m² square polygon (CCW).
  static geometry_msgs::msg::Polygon make_square_10x10()
  {
    return make_rectangle(0.0f, 0.0f, 10.0f, 10.0f);
  }

  /// Build an axis-aligned rectangular polygon (CCW).
  static geometry_msgs::msg::Polygon make_rectangle(float x0, float y0, float x1, float y1)
  {
    geometry_msgs::msg::Polygon poly;
    for (auto [x, y] : std::vector<std::pair<float, float>>{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}})
    {
      geometry_msgs::msg::Point32 pt;
      pt.x = x;
      pt.y = y;
      pt.z = 0.0f;
      poly.points.push_back(pt);
    }
    return poly;
  }

  /// Send an action goal and block until the result is received.
  std::shared_ptr<const PlanCoverageAction::Result> send_goal(
      const geometry_msgs::msg::Polygon& boundary,
      const std::vector<geometry_msgs::msg::Polygon>& obstacles = {},
      double mow_angle_deg = -1.0,
      bool skip_outline = false)
  {
    PlanCoverageAction::Goal goal;
    goal.outer_boundary = boundary;
    goal.obstacles = obstacles;
    goal.mow_angle_deg = mow_angle_deg;
    goal.skip_outline = skip_outline;

    auto send_goal_options = rclcpp_action::Client<PlanCoverageAction>::SendGoalOptions{};

    std::shared_ptr<const PlanCoverageAction::Result> result;
    std::atomic<bool> result_ready{false};

    send_goal_options.result_callback =
        [&result, &result_ready](const GoalHandle::WrappedResult& wr)
    {
      result = wr.result;
      result_ready.store(true);
    };

    auto goal_handle_future = action_client_->async_send_goal(goal, send_goal_options);

    // Wait for goal acceptance.
    if (goal_handle_future.wait_for(5s) != std::future_status::ready)
    {
      return nullptr;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
      return nullptr;
    }

    // Wait for result.
    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (!result_ready.load() && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(50ms);
    }

    return result;
  }

  // ---- Members ------------------------------------------------------------

  std::shared_ptr<CoveragePlannerNode> planner_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<PlanCoverageAction>::SharedPtr action_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

// ---------------------------------------------------------------------------
// Test: valid path on a simple 10x10 square
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, SimpleSquareProducesValidPath)
{
  const auto res = send_goal(make_square_10x10());

  ASSERT_NE(res, nullptr) << "Action goal timed out";
  ASSERT_TRUE(res->success) << "Action failed: " << res->message;
  EXPECT_GT(res->path.poses.size(), 0u) << "Expected non-empty swath path";
}

// ---------------------------------------------------------------------------
// Test: total distance exceeds a reasonable lower bound
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, TotalDistanceReasonable)
{
  const auto res = send_goal(make_square_10x10());

  ASSERT_NE(res, nullptr) << "Action goal timed out";
  ASSERT_TRUE(res->success) << res->message;

  // Inner polygon after 0.5 m headland inset: ~9x9 = 81 m².
  // Lower bound: inner_area / path_spacing * 0.5 (50% efficiency).
  const double inner_area = res->coverage_area;
  const double path_spacing = 0.5;
  const double min_expected = inner_area / path_spacing * 0.5;

  EXPECT_GT(res->total_distance, min_expected)
      << "total_distance=" << res->total_distance << " expected > " << min_expected;
}

// ---------------------------------------------------------------------------
// Test: outline path generated when skip_outline=false
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, OutlinePathGeneratedWhenNotSkipped)
{
  const auto res = send_goal(make_square_10x10(), {}, -1.0, false);

  ASSERT_NE(res, nullptr) << "Action goal timed out";
  ASSERT_TRUE(res->success) << res->message;
  EXPECT_GT(res->outline_path.poses.size(), 0u)
      << "Expected non-empty outline path when skip_outline=false";
}

// ---------------------------------------------------------------------------
// Test: no outline when skip_outline=true
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, OutlinePathEmptyWhenSkipped)
{
  const auto res = send_goal(make_square_10x10(), {}, -1.0, true);

  ASSERT_NE(res, nullptr) << "Action goal timed out";
  ASSERT_TRUE(res->success) << res->message;
  EXPECT_EQ(res->outline_path.poses.size(), 0u)
      << "Expected empty outline path when skip_outline=true";
}

// ---------------------------------------------------------------------------
// Test: custom mow angle is respected
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, CustomMowAngle45Degrees)
{
  const auto res = send_goal(make_square_10x10(), {}, 45.0, true);

  ASSERT_NE(res, nullptr) << "Action goal timed out";
  ASSERT_TRUE(res->success) << res->message;
  ASSERT_GT(res->path.poses.size(), 0u);

  // Extract yaw from the first pose quaternion.
  const auto& q = res->path.poses.front().pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // The yaw should be close to +/-45 deg (forward) or +/-135 deg (return pass).
  const double abs_yaw = std::abs(yaw);
  const double pi_4 = M_PI / 4.0;

  const bool near_45_or_135 = (std::abs(abs_yaw - pi_4) < 15.0 * M_PI / 180.0) ||
                              (std::abs(abs_yaw - 3.0 * pi_4) < 15.0 * M_PI / 180.0);

  EXPECT_TRUE(near_45_or_135) << "Expected yaw near +/-45 deg or +/-135 deg, got "
                              << yaw * 180.0 / M_PI << " deg";
}

// ---------------------------------------------------------------------------
// Test: coverage_area matches expected inner area
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, CoverageAreaMatchesInnerPolygon)
{
  const auto res = send_goal(make_square_10x10());

  ASSERT_NE(res, nullptr) << "Action goal timed out";
  ASSERT_TRUE(res->success) << res->message;

  // After 0.5 m inset on all sides: (10 - 2*0.5) x (10 - 2*0.5) = 9x9 = 81 m².
  EXPECT_NEAR(res->coverage_area, 81.0, 1.5);  // +/-1.5 m² tolerance
}

// ---------------------------------------------------------------------------
// Test: degenerate polygon (< 3 vertices) is rejected gracefully
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, DegeneratePolygonRejected)
{
  geometry_msgs::msg::Polygon bad_poly;
  geometry_msgs::msg::Point32 pt;
  pt.x = 0.0f;
  pt.y = 0.0f;
  pt.z = 0.0f;
  bad_poly.points.push_back(pt);
  pt.x = 1.0f;
  bad_poly.points.push_back(pt);
  // Only 2 vertices — must be rejected.

  const auto res = send_goal(bad_poly);
  ASSERT_NE(res, nullptr) << "Action goal timed out";
  EXPECT_FALSE(res->success);
  EXPECT_FALSE(res->message.empty());
}

// ---------------------------------------------------------------------------
// Test: auto-angle on a wide rectangle produces a valid path
// ---------------------------------------------------------------------------

TEST_F(CoveragePlannerTest, AutoAngleOnWideRectangle)
{
  const auto res = send_goal(make_rectangle(0.0f, 0.0f, 20.0f, 4.0f));

  ASSERT_NE(res, nullptr) << "Action goal timed out";
  ASSERT_TRUE(res->success) << res->message;
  EXPECT_GT(res->path.poses.size(), 0u);
}

// ---------------------------------------------------------------------------
// Test: boustrophedon alternation — consecutive swath segments flip heading
// ---------------------------------------------------------------------------
//
// At each U-turn boundary the last waypoint of swath N and the first waypoint
// of swath N+1 must differ in heading by roughly π (within ±45°).  This
// verifies that BoustrophedonOrder has actually reversed alternate swaths and
// that the path building loop preserves the alternation.
//
// Strategy: scan consecutive pose pairs whose position delta is ≤ 2× the
// swath spacing (i.e. they are U-turn transitions, not intra-swath steps) and
// check that their headings differ by nearly π.

TEST_F(CoveragePlannerTest, BoustrophedonHeadingAlternates)
{
  // Use a fixed 0° angle so swaths are horizontal and the alternation is
  // well-defined regardless of F2C's BruteForce optimisation.
  const auto res = send_goal(make_square_10x10(), {}, 0.0, true);

  ASSERT_NE(res, nullptr) << "Action goal timed out";
  ASSERT_TRUE(res->success) << "Action failed: " << res->message;

  const auto& poses = res->path.poses;
  ASSERT_GT(poses.size(), 1u) << "Expected non-empty path";

  // Extract yaw from a quaternion (planar rotation about Z).
  auto quat_to_yaw = [](const geometry_msgs::msg::Quaternion& q) -> double
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };

  // Swath spacing used by the test fixture is 0.5 m.
  // A pose-pair is a U-turn boundary if the lateral (Y) distance between the
  // two poses exceeds 0.3 m (more than half the spacing) while the along-swath
  // (X) distance is small.  We use a simpler but robust heuristic: the total
  // Euclidean distance is ≥ 0.3 m and ≤ 1.0 m AND the headings differ by > π/2.
  const double spacing = 0.5;
  int uturn_count = 0;
  int alternation_failures = 0;

  for (size_t i = 1; i < poses.size(); ++i)
  {
    const double dx = poses[i].pose.position.x - poses[i - 1].pose.position.x;
    const double dy = poses[i].pose.position.y - poses[i - 1].pose.position.y;
    const double dist = std::hypot(dx, dy);

    // Skip intra-swath steps (very short) and large gaps (shouldn't exist).
    if (dist < spacing * 0.5 || dist > spacing * 2.0)
    {
      continue;
    }

    ++uturn_count;

    const double yaw_prev = quat_to_yaw(poses[i - 1].pose.orientation);
    const double yaw_curr = quat_to_yaw(poses[i].pose.orientation);
    double delta = std::abs(yaw_curr - yaw_prev);
    while (delta > M_PI)
      delta = std::abs(delta - 2.0 * M_PI);

    // Heading should differ by at least 135° (3π/4) at a U-turn.
    if (delta < M_PI * 3.0 / 4.0)
    {
      ++alternation_failures;
      ADD_FAILURE() << "U-turn at pose " << i << ": heading delta = " << delta * 180.0 / M_PI
                    << "° (expected ≥ 135°)."
                    << "  yaw_prev=" << yaw_prev * 180.0 / M_PI
                    << "° yaw_curr=" << yaw_curr * 180.0 / M_PI << "°";
    }
  }

  EXPECT_GT(uturn_count, 0) << "No U-turn boundaries detected — path may contain only one swath "
                               "or swath spacing heuristic needs adjustment";
  EXPECT_EQ(alternation_failures, 0)
      << alternation_failures << " U-turn(s) did not alternate heading by ~180°";
}

// ---------------------------------------------------------------------------
// GTest main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
