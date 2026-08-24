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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_start_occupied_retry.cpp
 * @brief Regression for issue #487 — one occupied start pose forfeited a whole
 *        field.
 *
 * 2026-08-24, attempt 1 of an area-0 mow cut ZERO grass. The robot undocked to
 * ~(4.47, 4.70), which sits inside the inflated keepout of a 0.25 m obstacle
 * circle, and SmacPlanner2D (no start tolerance) answered every one of 26 plan
 * calls with "Start occupied". FollowStrip treated each refused blade-off
 * transit as an ordinary skipped swath, ran out of sub-paths, and declared the
 * area unmowable. A second attempt 13 minutes later mowed the same area to
 * 100 %.
 *
 * Three things are pinned here:
 *   1. classifyTransitFailure() tells a START_OCCUPIED refusal (about the
 *      ROBOT'S pose) apart from every other transit failure (about the GOAL),
 *      from the nav2 error_code and, as a fallback, the planner's error_msg.
 *   2. IsCoverageStartBlocked consumes the flag exactly once, so the recovery
 *      branch cannot re-fire on an unrelated later failure.
 *   3. main_tree.xml still carries the non-motion recovery branch (clear the
 *      costmaps, wait, retry) and still commands NO escape motion — the
 *      escape-direction decision is deliberately left to the maintainer.
 */

#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_behavior/transit_failure.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::classifyTransitFailure;
using mowgli_behavior::IsCoverageStartBlocked;
using mowgli_behavior::isStartPoseBlocked;
using mowgli_behavior::TransitFailure;
using mowgli_behavior::transitFailureName;

using ComputePath = nav2_msgs::action::ComputePathToPose::Result;
using Navigate = nav2_msgs::action::NavigateToPose::Result;

// ---------------------------------------------------------------------------
// 1. Classification — pure, no ROS spinning.
// ---------------------------------------------------------------------------

// The exact field signature: bt_navigator copies ComputePathToPose's
// START_OCCUPIED (205) into the NavigateToPose result because
// navigate_to_pose.xml wires error_code_id="{compute_path_error_code}" and
// "compute_path" is in bt_navigator's default error_code_name_prefixes.
TEST(TransitFailureClassification, StartOccupiedErrorCodeIsRecognised)
{
  const auto kind = classifyTransitFailure(ComputePath::START_OCCUPIED, "Start occupied");
  EXPECT_EQ(kind, TransitFailure::kStartOccupied);
  EXPECT_TRUE(isStartPoseBlocked(kind));
  EXPECT_STREQ(transitFailureName(kind), "START_OCCUPIED");
}

// A robot whose bt_navigator drops "compute_path" from error_code_name_prefixes
// (or a tree without error_code_id) reports UNKNOWN. nav2_smac_planner still
// throws nav2_core::StartOccupied("Start occupied") and planner_server still
// puts that text in error_msg, so the message is a second, independent signal.
TEST(TransitFailureClassification, StartOccupiedMessageIsRecognisedWithoutACode)
{
  EXPECT_TRUE(isStartPoseBlocked(classifyTransitFailure(
      Navigate::UNKNOWN,
      "GridBasedplugin failed to plan from (4.47, 4.70) to (2.07, 9.54): \"Start occupied\"")));
  // Casing is not part of any API contract.
  EXPECT_TRUE(isStartPoseBlocked(classifyTransitFailure(ComputePath::NONE, "START OCCUPIED")));
}

// The whole point of the change: a failure about the GOAL must stay an ordinary
// skipped swath, because that swath really is unreachable this pass.
TEST(TransitFailureClassification, GoalSideFailuresAreNotStartBlocked)
{
  EXPECT_EQ(classifyTransitFailure(ComputePath::GOAL_OCCUPIED, "Goal occupied"),
            TransitFailure::kGoalOccupied);
  EXPECT_EQ(classifyTransitFailure(ComputePath::NO_VALID_PATH, "No valid path"),
            TransitFailure::kNoValidPath);
  EXPECT_EQ(classifyTransitFailure(ComputePath::TIMEOUT, "timed out"), TransitFailure::kTimeout);
  EXPECT_EQ(classifyTransitFailure(ComputePath::TF_ERROR, "tf"), TransitFailure::kTfError);
  EXPECT_EQ(classifyTransitFailure(Navigate::TIMEOUT, ""), TransitFailure::kTimeout);

  EXPECT_FALSE(isStartPoseBlocked(classifyTransitFailure(ComputePath::GOAL_OCCUPIED, "")));
  EXPECT_FALSE(isStartPoseBlocked(classifyTransitFailure(ComputePath::NO_VALID_PATH, "")));
  EXPECT_FALSE(isStartPoseBlocked(classifyTransitFailure(ComputePath::TIMEOUT, "")));
}

// No result at all (goal rejected, or bt_navigator died before answering) must
// never be guessed into the start-blocked bucket — that bucket suppresses the
// "area not mowable" verdict and must stay evidence-backed.
TEST(TransitFailureClassification, MissingResultIsUnknownNotStartBlocked)
{
  const auto kind = classifyTransitFailure(ComputePath::NONE, "");
  EXPECT_EQ(kind, TransitFailure::kUnknown);
  EXPECT_FALSE(isStartPoseBlocked(kind));
  EXPECT_STREQ(transitFailureName(kind), "UNKNOWN");
}

// An unrelated non-zero code is "other", not a silent start-block.
TEST(TransitFailureClassification, UnrelatedCodeIsOther)
{
  const auto kind = classifyTransitFailure(ComputePath::INVALID_PLANNER, "no such planner");
  EXPECT_EQ(kind, TransitFailure::kOther);
  EXPECT_FALSE(isStartPoseBlocked(kind));
}

// A goal-side code must win over an incidental message match, so a planner that
// mentions the phrase while reporting something else cannot suppress the
// unmowable verdict.
TEST(TransitFailureClassification, ExplicitCodeWinsOverTheMessageFallback)
{
  EXPECT_EQ(classifyTransitFailure(ComputePath::NO_VALID_PATH, "start occupied earlier, then..."),
            TransitFailure::kNoValidPath);
}

// ---------------------------------------------------------------------------
// 2. IsCoverageStartBlocked — consumed exactly once.
// ---------------------------------------------------------------------------

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

class StartBlockedConditionTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_start_occupied_retry");
    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    factory.registerNodeType<IsCoverageStartBlocked>("IsCoverageStartBlocked");
  }

  BT::Tree makeTree()
  {
    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<IsCoverageStartBlocked/>"
        "</BehaviorTree></root>";
    return factory.createTreeFromText(xml, blackboard);
  }
};

TEST_F(StartBlockedConditionTest, FailsWhenNoPassWasStartBlocked)
{
  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(StartBlockedConditionTest, FiresOnceThenConsumesTheFlag)
{
  ctx->coverage_start_blocked = true;
  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->coverage_start_blocked) << "the flag must be consumed on read";
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
      << "a second tick must not re-run the recovery for the same blocked pass";
}

// The two consumers are independent: the tree's condition node must not eat the
// per-area retirement exemption that GetNextUnmowedArea reads.
TEST_F(StartBlockedConditionTest, DoesNotConsumeTheAreaRetirementExemption)
{
  ctx->coverage_start_blocked = true;
  ctx->start_blocked_area = 0u;
  auto tree = makeTree();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(ctx->start_blocked_area.has_value());
  EXPECT_EQ(*ctx->start_blocked_area, 0u);
}

// ---------------------------------------------------------------------------
// 3. main_tree.xml still carries the NON-MOTION recovery branch.
// ---------------------------------------------------------------------------

namespace
{

std::string ReadMainTree()
{
  std::ifstream f(MOWGLI_MAIN_TREE_PATH);
  EXPECT_TRUE(f.is_open()) << "Cannot open " << MOWGLI_MAIN_TREE_PATH;
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

/// Text of the <Sequence name="StartPoseBlockedRetry"> ... </Sequence> block.
std::string ExtractStartBlockedBranch(const std::string& xml)
{
  const std::string open = "<Sequence name=\"StartPoseBlockedRetry\">";
  const auto begin = xml.find(open);
  if (begin == std::string::npos)
  {
    return {};
  }
  const auto end = xml.find("</Sequence>", begin);
  if (end == std::string::npos)
  {
    return {};
  }
  return xml.substr(begin, end - begin);
}

}  // namespace

TEST(StartBlockedTreeStructure, RecoveryBranchExistsAndRetriesInsteadOfGivingUp)
{
  const std::string branch = ExtractStartBlockedBranch(ReadMainTree());
  ASSERT_FALSE(branch.empty()) << "StartPoseBlockedRetry branch missing from main_tree.xml — a "
                                  "START_OCCUPIED pass would forfeit the whole area again (#487)";
  EXPECT_NE(branch.find("<IsCoverageStartBlocked/>"), std::string::npos)
      << "the branch must be gated on the consumed start-blocked signal";
  EXPECT_NE(branch.find("<ClearCostmap/>"), std::string::npos)
      << "the retry must clear the costmaps first (helps only for a TRANSIENT obstacle; a keepout "
         "is a static filter and survives the clear)";
  EXPECT_NE(branch.find("<AlwaysFailure/>"), std::string::npos)
      << "the branch must bubble up FAILURE so FollowStripRetry re-ticks FollowStrip";
}

// SAFETY: this robot has blades and the escape direction is genuinely ambiguous
// (after an undock it REVERSED into the blocked spot, so reversing again drives
// deeper). The recovery must stay non-motion until a maintainer picks a
// strategy — see issue #487.
TEST(StartBlockedTreeStructure, RecoveryBranchCommandsNoMotion)
{
  const std::string branch = ExtractStartBlockedBranch(ReadMainTree());
  ASSERT_FALSE(branch.empty());
  EXPECT_EQ(branch.find("<BackUp"), std::string::npos)
      << "no reverse escape may be added without a maintainer decision (#487)";
  EXPECT_EQ(branch.find("<Spin"), std::string::npos)
      << "no spin escape may be added without a maintainer decision (#487)";
  EXPECT_EQ(branch.find("<NavigateToPose"), std::string::npos)
      << "no drive-out escape may be added without a maintainer decision (#487)";
  EXPECT_NE(branch.find("<SetMowerEnabled enabled=\"false\"/>"), std::string::npos)
      << "the blade must be OFF while the robot sits blocked";
  EXPECT_NE(branch.find("<StopMoving/>"), std::string::npos);
}
