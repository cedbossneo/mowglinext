# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


"""
test_coverage_action.launch.py

launch_testing integration test for the CoveragePlannerNode action server.

Test cases:
  - test_plan_coverage_success    – send a PlanCoverage goal with a 10 x 10 m
                                    square polygon; verify success=True,
                                    path contains poses, total_distance > 0.
  - test_skip_outline             – send goal with skip_outline=True; verify the
                                    result carries an empty outline_path.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Point32, Polygon
from mowgli_interfaces.action import PlanCoverage


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _square_polygon(side_m: float = 10.0) -> Polygon:
    """Return a closed CCW square polygon with the given side length."""
    half = side_m / 2.0
    poly = Polygon()
    poly.points = [
        Point32(x=float(-half), y=float(-half), z=0.0),
        Point32(x=float(half),  y=float(-half), z=0.0),
        Point32(x=float(half),  y=float(half),  z=0.0),
        Point32(x=float(-half), y=float(half),  z=0.0),
    ]
    return poly


def _send_goal(
    node: rclpy.node.Node,
    action_client: ActionClient,
    goal: PlanCoverage.Goal,
    timeout_sec: float = 15.0,
) -> PlanCoverage.Result:
    """Synchronously send an action goal and return the result."""
    if not action_client.wait_for_server(timeout_sec=timeout_sec):
        raise RuntimeError(
            "PlanCoverage action server not available after "
            f"{timeout_sec} s"
        )

    future = action_client.send_goal_async(goal)
    deadline = time.monotonic() + timeout_sec

    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() > deadline:
            raise RuntimeError("Goal send timed out")

    goal_handle = future.result()
    if not goal_handle.accepted:
        raise RuntimeError("Goal was rejected by action server")

    result_future = goal_handle.get_result_async()
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() > deadline:
            raise RuntimeError("Result timed out")

    return result_future.result().result


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

@pytest.mark.launch_test
def generate_test_description():
    coverage_planner = launch_ros.actions.Node(
        package="mowgli_coverage_planner",
        executable="coverage_planner_node",
        name="coverage_planner_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "tool_width": 0.18,
                "headland_passes": 1,
                "headland_width": 0.18,
                "path_spacing": 0.18,
                "map_frame": "map",
                "min_turning_radius": 0.3,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                coverage_planner,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"coverage_planner": coverage_planner},
    )


# ---------------------------------------------------------------------------
# Active-state tests
# ---------------------------------------------------------------------------

class TestCoverageAction(unittest.TestCase):
    """Integration tests for the PlanCoverage action server."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("test_coverage_action_helper")
        cls.action_client = ActionClient(
            cls.node,
            PlanCoverage,
            "/coverage_planner_node/plan_coverage",
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.action_client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # test_plan_coverage_success
    # ------------------------------------------------------------------

    def test_plan_coverage_success(self) -> None:
        """
        A 10 x 10 m square polygon must produce a successful result with
        at least one pose in the coverage path and a positive total distance.
        """
        goal = PlanCoverage.Goal()
        goal.outer_boundary = _square_polygon(side_m=10.0)
        goal.obstacles = []
        goal.mow_angle_deg = -1.0   # auto-optimise
        goal.skip_outline = False

        result = _send_goal(self.node, self.action_client, goal)

        self.assertTrue(
            result.success,
            msg=f"action returned success=False: {result.message}",
        )
        self.assertGreater(
            len(result.path.poses),
            0,
            msg="coverage path must contain at least one pose",
        )
        self.assertGreater(
            result.total_distance,
            0.0,
            msg="total_distance must be positive",
        )

    # ------------------------------------------------------------------
    # test_skip_outline
    # ------------------------------------------------------------------

    def test_skip_outline(self) -> None:
        """
        When skip_outline=True the result must succeed and the
        outline_path must be empty (zero poses).
        """
        goal = PlanCoverage.Goal()
        goal.outer_boundary = _square_polygon(side_m=10.0)
        goal.obstacles = []
        goal.mow_angle_deg = -1.0
        goal.skip_outline = True

        result = _send_goal(self.node, self.action_client, goal)

        self.assertTrue(
            result.success,
            msg=f"action returned success=False with skip_outline=True: "
                f"{result.message}",
        )
        self.assertEqual(
            len(result.outline_path.poses),
            0,
            msg="outline_path must be empty when skip_outline=True",
        )


# ---------------------------------------------------------------------------
# Post-shutdown tests
# ---------------------------------------------------------------------------

@launch_testing.post_shutdown_test()
class TestCoverageActionShutdown(unittest.TestCase):
    """Verify the coverage_planner_node process exited cleanly."""

    def test_exit_code(self, proc_info, coverage_planner) -> None:
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=coverage_planner,
            allowable_exit_codes=[0],
        )
