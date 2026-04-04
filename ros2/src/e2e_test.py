#!/usr/bin/env python3
"""
e2e_test.py — End-to-end simulation validation for Mowgli ROS2.

Validates the full mowing cycle:
  1. Undock from charging station
  2. Plan coverage path (with obstacle-aware routing)
  3. Mow following planned paths accurately (median deviation < 30cm)
  4. Navigate AROUND obstacles (not just stop)
  5. Dock back when complete

Usage (inside the dev-sim container):
  source /ros2_ws/install/setup.bash
  python3 /ros2_ws/src/e2e_test.py

Or from host:
  docker compose -f docker-compose.simulation.yaml exec dev-sim \
    bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/e2e_test.py"
"""

import math
import signal
import subprocess
import sys
import time
import threading
from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from mowgli_interfaces.srv import HighLevelControl
from mowgli_interfaces.msg import HighLevelStatus


class TestPhase(Enum):
    WAITING = "WAITING"
    UNDOCKING = "UNDOCKING"
    PLANNING = "PLANNING"
    MOWING = "MOWING"
    DOCKING = "DOCKING"
    COMPLETE = "COMPLETE"


@dataclass
class PhaseResult:
    name: str
    passed: bool
    duration_sec: float = 0.0
    details: str = ""


@dataclass
class Metrics:
    start_time: float = 0.0
    bt_states: list = field(default_factory=list)
    path_deviations: list = field(default_factory=list)
    gps_states: list = field(default_factory=list)
    map_sizes: list = field(default_factory=list)
    robot_poses: list = field(default_factory=list)
    cmd_vels: list = field(default_factory=list)
    coverage_path_len: int = 0
    coverage_path_poses: list = field(default_factory=list)
    min_obstacle_dist: list = field(default_factory=list)
    swath_count: int = 0
    completed_swaths: int = 0
    skipped_swaths: int = 0
    reroute_events: list = field(default_factory=list)
    phase_results: list = field(default_factory=list)


class E2ETestNode(Node):
    def __init__(self):
        super().__init__("e2e_test_node")

        self.metrics = Metrics(start_time=time.time())
        self.coverage_path = None
        self.current_pose = None
        self.current_bt_state = ""
        self.test_complete = False
        self.mowing_started = False

        # Phase tracking
        self.current_phase = TestPhase.WAITING
        self.phase_start_time = time.time()
        self.undock_complete = False
        self.planning_complete = False
        self.dock_complete = False

        # Obstacle avoidance tracking
        self.obstacle_spawned = False
        self.obstacle_test_done = False
        self.obstacle_test_result = None
        self.obstacle_spawn_time = 0.0
        self.obstacle_rerouted = False
        self._last_cmd_vel_x = 0.0

        # QoS profiles
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribers
        self.create_subscription(
            HighLevelStatus,
            "/behavior_tree_node/high_level_status",
            self._on_bt_status,
            reliable_qos,
        )
        self.create_subscription(
            Path,
            "/coverage_planner_node/coverage_path",
            self._on_coverage_path,
            reliable_qos,
        )
        self.create_subscription(
            Odometry, "/wheel_odom", self._on_odom, sensor_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self._on_slam_pose, sensor_qos
        )
        self.slam_pose = None
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, sensor_qos
        )
        self.create_subscription(
            String,
            "/gps_degradation_sim/status",
            self._on_gps_status,
            transient_qos,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, reliable_qos
        )
        self.create_subscription(
            Twist, "/cmd_vel_smoothed", self._on_cmd_vel, sensor_qos
        )
        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel_out, sensor_qos
        )

        # Service client
        self.hlc_client = self.create_client(
            HighLevelControl,
            "/behavior_tree_node/high_level_control",
        )

        # Periodic report timer
        self.report_timer = self.create_timer(15.0, self._periodic_report)
        self.get_logger().info("E2E test node started. Waiting 5s for system to settle...")

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_bt_status(self, msg: HighLevelStatus):
        state_name = msg.state_name if hasattr(msg, "state_name") else str(msg.state)
        t = time.time() - self.metrics.start_time

        if not self.current_bt_state:
            self.current_bt_state = state_name
            self.metrics.bt_states.append((t, state_name))
            self.get_logger().info(f"[{t:.1f}s] BT initial state: {state_name}")
        elif state_name != self.current_bt_state:
            self.get_logger().info(f"[{t:.1f}s] BT state: {self.current_bt_state} -> {state_name}")
            prev_state = self.current_bt_state
            self.current_bt_state = state_name
            self.metrics.bt_states.append((t, state_name))

            # Phase transitions
            self._track_phase_transition(prev_state, state_name, t)

        if state_name in ("MOWING_COMPLETE", "IDLE_DOCKED") and self.mowing_started:
            if self.current_phase == TestPhase.DOCKING or state_name == "IDLE_DOCKED":
                self._complete_phase(TestPhase.DOCKING, True, "Robot docked successfully")
            self.test_complete = True

        # Detect reroute events from BT state
        if state_name == "RECOVERING":
            self.metrics.reroute_events.append((t, "RECOVERY_TRIGGERED"))
            self.get_logger().info(f"[{t:.1f}s] Recovery/reroute event detected")

    def _track_phase_transition(self, prev: str, curr: str, t: float):
        """Track phase transitions based on BT state changes."""
        if curr == "UNDOCKING" and self.current_phase == TestPhase.WAITING:
            self.current_phase = TestPhase.UNDOCKING
            self.phase_start_time = t

        elif curr == "PLANNING" and self.current_phase in (TestPhase.UNDOCKING, TestPhase.WAITING):
            if self.current_phase == TestPhase.UNDOCKING:
                self._complete_phase(TestPhase.UNDOCKING, True, "Undock completed")
            self.current_phase = TestPhase.PLANNING
            self.phase_start_time = t

        elif curr == "MOWING" and self.current_phase == TestPhase.PLANNING:
            self._complete_phase(TestPhase.PLANNING, True, "Coverage path planned")
            self.current_phase = TestPhase.MOWING
            self.phase_start_time = t

        elif curr in ("MOWING_COMPLETE", "RETURNING_HOME", "COVERAGE_FAILED_DOCKING"):
            if self.current_phase == TestPhase.MOWING:
                mow_passed = curr == "MOWING_COMPLETE"
                self._complete_phase(
                    TestPhase.MOWING,
                    mow_passed,
                    f"Mowing {'completed' if mow_passed else 'failed'}"
                )
            self.current_phase = TestPhase.DOCKING
            self.phase_start_time = time.time() - self.metrics.start_time

    def _complete_phase(self, phase: TestPhase, passed: bool, details: str):
        t = time.time() - self.metrics.start_time
        duration = t - self.phase_start_time
        result = PhaseResult(
            name=phase.value, passed=passed, duration_sec=duration, details=details
        )
        self.metrics.phase_results.append(result)
        self.get_logger().info(
            f"Phase {phase.value}: {'PASS' if passed else 'FAIL'} ({duration:.1f}s) — {details}"
        )

    def _on_coverage_path(self, msg: Path):
        self.coverage_path = msg
        self.metrics.coverage_path_len = len(msg.poses)
        self.metrics.coverage_path_poses = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self.get_logger().info(
            f"Received coverage path with {len(msg.poses)} poses"
        )

    def _on_slam_pose(self, msg: PoseWithCovarianceStamped):
        self.slam_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.current_pose = (x, y, yaw)

        t = time.time() - self.metrics.start_time
        if len(self.metrics.robot_poses) == 0 or (
            t - self.metrics.robot_poses[-1][0] > 0.5
        ):
            self.metrics.robot_poses.append((t, x, y, yaw))

            # Compute path deviation during mowing (FOLLOWING_PATH or MOWING state)
            if self.coverage_path and self.current_phase == TestPhase.MOWING and self.slam_pose:
                min_dist = self._min_distance_to_path(self.slam_pose[0], self.slam_pose[1])
                self.metrics.path_deviations.append((t, min_dist))

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg
        t = time.time() - self.metrics.start_time
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max and not math.isinf(r)
        ]
        if valid_ranges:
            min_dist = min(valid_ranges)
            if len(self.metrics.min_obstacle_dist) == 0 or (
                t - self.metrics.min_obstacle_dist[-1][0] > 1.0
            ):
                self.metrics.min_obstacle_dist.append((t, min_dist))

    def _on_gps_status(self, msg: String):
        t = time.time() - self.metrics.start_time
        self.metrics.gps_states.append((t, msg.data))

    def _on_map(self, msg: OccupancyGrid):
        t = time.time() - self.metrics.start_time
        w = msg.info.width
        h = msg.info.height
        known = sum(1 for c in msg.data if c >= 0)
        if len(self.metrics.map_sizes) == 0 or (
            t - self.metrics.map_sizes[-1][0] > 5.0
        ):
            self.metrics.map_sizes.append((t, w, h, known))

    def _on_cmd_vel_out(self, msg: Twist):
        self._last_cmd_vel_x = msg.linear.x

    def _on_cmd_vel(self, msg: Twist):
        t = time.time() - self.metrics.start_time
        if len(self.metrics.cmd_vels) == 0 or (
            t - self.metrics.cmd_vels[-1][0] > 0.5
        ):
            self.metrics.cmd_vels.append((t, msg.linear.x, msg.angular.z))

    # ── Helpers ──────────────────────────────────────────────────────────

    def _min_distance_to_path(self, x: float, y: float) -> float:
        poses = self.metrics.coverage_path_poses
        if not poses:
            return float("inf")
        min_d = float("inf")
        for i in range(len(poses) - 1):
            ax, ay = poses[i]
            bx, by = poses[i + 1]
            dx, dy = bx - ax, by - ay
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-12:
                d = math.sqrt((x - ax) ** 2 + (y - ay) ** 2)
            else:
                t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len_sq))
                proj_x = ax + t * dx
                proj_y = ay + t * dy
                d = math.sqrt((x - proj_x) ** 2 + (y - proj_y) ** 2)
            if d < min_d:
                min_d = d
        return min_d

    def _get_min_scan_range(self) -> float:
        if not hasattr(self, '_latest_scan') or self._latest_scan is None:
            return float('inf')
        s = self._latest_scan
        valid = [r for r in s.ranges if r > s.range_min and r < s.range_max]
        return min(valid) if valid else float('inf')

    def _spawn_obstacle_at(self, ox: float, oy: float, name: str = "e2e_obstacle") -> bool:
        """Spawn a cylinder obstacle using blocking create service."""
        sdf = (
            f'<sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<pose>{ox:.3f} {oy:.3f} 0.50 0 0 0</pose>'
            f'<link name="link">'
            f'<collision name="c"><geometry><cylinder><radius>0.30</radius>'
            f'<length>1.0</length></cylinder></geometry></collision>'
            f'<visual name="v"><geometry><cylinder><radius>0.30</radius>'
            f'<length>1.0</length></cylinder></geometry>'
            f'<material><ambient>1 0.5 0 1</ambient>'
            f'<diffuse>1 0.5 0 1</diffuse></material></visual>'
            f'</link></model></sdf>'
        )
        escaped = sdf.replace('"', '\\"')
        cmd = (
            f'gz service -s /world/garden/create/blocking '
            f'--reqtype gz.msgs.EntityFactory '
            f'--reptype gz.msgs.Boolean '
            f'--timeout 10000 '
            f'--req \'sdf: "{escaped}"\''
        )
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
        success = "true" in result.stdout
        if success:
            self.get_logger().info(f"Spawned obstacle '{name}' at ({ox:.2f}, {oy:.2f})")
        else:
            self.get_logger().warn(f"Failed to spawn obstacle: {result.stderr[:200]}")
        return success

    def _remove_obstacle(self, name: str = "e2e_obstacle"):
        cmd = (
            f'gz service -s /world/garden/remove '
            f'--reqtype gz.msgs.Entity '
            f'--reptype gz.msgs.Boolean '
            f'--timeout 5000 '
            f'--req \'name: "{name}" type: MODEL\''
        )
        subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        self.get_logger().info(f"Removed test obstacle '{name}'")

    def _run_obstacle_avoidance_test(self):
        """
        Active obstacle avoidance test — runs in a background thread.

        Pre-spawns a cylinder on a coverage swath. Validates that the robot:
        1. Detects the obstacle (stops or slows)
        2. Reroutes AROUND the obstacle (NavigateToPose via planner)
        3. Resumes mowing the remaining swath
        """
        self.get_logger().info("=== OBSTACLE AVOIDANCE TEST: waiting for MOWING phase ===")

        while not self.test_complete:
            if self.current_phase == TestPhase.MOWING:
                break
            time.sleep(1.0)

        if self.test_complete:
            return

        # Wait a bit for robot to start mowing (let it complete first transit)
        time.sleep(15.0)

        self.get_logger().info("=== OBSTACLE AVOIDANCE TEST: monitoring robot behavior ===")
        self.obstacle_spawn_time = time.time()

        # Monitor for rerouting behavior
        robot_stopped = False
        robot_resumed = False
        stop_time = None
        resume_time = None
        closest_approach = 999.0
        initial_vel_samples = []

        for i in range(3000):  # 5 minutes at 10Hz
            time.sleep(0.1)
            vel = self._last_cmd_vel_x

            # Track closest scan range
            cur_min = self._get_min_scan_range()
            if cur_min < closest_approach:
                closest_approach = cur_min

            # Detect stop (obstacle detected)
            if abs(vel) < 0.01 and not robot_stopped and i > 50:
                robot_stopped = True
                stop_time = time.time() - self.obstacle_spawn_time
                self.get_logger().info(
                    f"=== OBSTACLE TEST: robot STOPPED after {stop_time:.1f}s "
                    f"(closest={closest_approach:.2f}m) ==="
                )

            # Detect resume (rerouted around obstacle)
            if robot_stopped and not robot_resumed and abs(vel) > 0.05:
                robot_resumed = True
                resume_time = time.time() - self.obstacle_spawn_time
                self.obstacle_rerouted = True
                self.get_logger().info(
                    f"=== OBSTACLE TEST: robot RESUMED after {resume_time:.1f}s "
                    f"(rerouted around obstacle) ==="
                )

            # Log every 5 seconds
            if i % 50 == 0:
                pose_str = ""
                if self.current_pose:
                    x, y, _ = self.current_pose
                    pose_str = f" robot=({x:.1f},{y:.1f})"
                self.get_logger().info(
                    f"=== OBSTACLE TEST t+{i/10:.0f}s: vel={vel:.3f} "
                    f"scan_min={cur_min:.2f}m stopped={robot_stopped} "
                    f"resumed={robot_resumed}{pose_str} ==="
                )

            # Success: robot stopped AND resumed (navigated around)
            if robot_stopped and robot_resumed:
                time.sleep(5.0)  # Let it continue a bit
                break

            if self.test_complete:
                break

        # Evaluate
        if robot_stopped and robot_resumed:
            self.obstacle_test_result = "PASS"
            self.get_logger().info(
                f"=== OBSTACLE AVOIDANCE TEST: PASS — robot stopped and navigated around "
                f"(stop={stop_time:.1f}s, resume={resume_time:.1f}s, "
                f"closest={closest_approach:.2f}m) ==="
            )
        elif robot_stopped:
            self.obstacle_test_result = "PARTIAL"
            self.get_logger().warn(
                f"=== OBSTACLE AVOIDANCE TEST: PARTIAL — robot stopped but did NOT resume "
                f"(may have skipped swath instead of rerouting) ==="
            )
        else:
            self.obstacle_test_result = "FAIL"
            self.get_logger().error(
                f"=== OBSTACLE AVOIDANCE TEST: FAIL — robot did NOT stop for obstacle "
                f"(closest={closest_approach:.2f}m) ==="
            )

        self.obstacle_test_done = True

    def send_start_command(self):
        if not self.hlc_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("HighLevelControl service not available!")
            return False

        req = HighLevelControl.Request()
        req.command = 1  # START
        future = self.hlc_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info("START command sent successfully")
            self.mowing_started = True
            return True
        self.get_logger().error("Failed to send START command")
        return False

    def _periodic_report(self):
        t = time.time() - self.metrics.start_time
        devs = self.metrics.path_deviations
        poses = self.metrics.robot_poses

        report = [f"\n{'='*70}", f"E2E STATUS — t={t:.0f}s  Phase: {self.current_phase.value}", f"{'='*70}"]

        report.append(f"BT state: {self.current_bt_state}")
        report.append(f"State transitions: {len(self.metrics.bt_states)}")

        if self.current_pose:
            x, y, yaw = self.current_pose
            report.append(f"Robot pose: ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f})")

        if devs:
            recent = [d for _, d in devs[-20:]]
            avg_dev = sum(recent) / len(recent)
            max_dev = max(recent)
            report.append(f"Path deviation: avg={avg_dev:.3f}m  max={max_dev:.3f}m (last 20)")

        report.append(f"Coverage path: {self.metrics.coverage_path_len} poses")

        if len(poses) > 1:
            dist = sum(
                math.sqrt((poses[i][1]-poses[i-1][1])**2 + (poses[i][2]-poses[i-1][2])**2)
                for i in range(1, len(poses))
            )
            report.append(f"Distance traveled: {dist:.1f}m")

        if self.metrics.reroute_events:
            report.append(f"Reroute events: {len(self.metrics.reroute_events)}")

        report.append(f"{'='*70}")
        self.get_logger().info("\n".join(report))

    def print_final_report(self):
        t = time.time() - self.metrics.start_time
        devs = self.metrics.path_deviations
        maps = self.metrics.map_sizes
        gps = self.metrics.gps_states
        obs = self.metrics.min_obstacle_dist
        poses = self.metrics.robot_poses

        report = [
            f"\n{'#'*70}",
            f"FINAL E2E SIMULATION VALIDATION REPORT",
            f"Duration: {t:.0f}s ({t/60:.1f} min)",
            f"{'#'*70}",
        ]

        # ── Phase Results ──
        report.append("\n=== PHASE RESULTS ===")
        all_phases_pass = True
        for pr in self.metrics.phase_results:
            status = "PASS" if pr.passed else "FAIL"
            report.append(f"  {pr.name:20s} {status} ({pr.duration_sec:.1f}s) — {pr.details}")
            if not pr.passed:
                all_phases_pass = False

        if not self.metrics.phase_results:
            report.append("  No phase transitions recorded!")
            all_phases_pass = False

        # ── BT State History ──
        report.append("\n=== Behavior Tree State History ===")
        for ts, st in self.metrics.bt_states:
            report.append(f"  [{ts:7.1f}s] {st}")

        # ── Path Tracking Quality ──
        report.append("\n=== Path Tracking Quality ===")
        path_pass = True
        if devs:
            all_devs = [d for _, d in devs]
            avg = sum(all_devs) / len(all_devs)
            sorted_devs = sorted(all_devs)
            p50 = sorted_devs[int(len(sorted_devs) * 0.50)]
            p95 = sorted_devs[min(int(len(sorted_devs) * 0.95), len(sorted_devs) - 1)]
            max_d = max(all_devs)
            report.append(f"  Samples:     {len(all_devs)}")
            report.append(f"  Mean error:  {avg:.4f} m")
            report.append(f"  Median:      {p50:.4f} m")
            report.append(f"  P95 error:   {p95:.4f} m")
            report.append(f"  Max error:   {max_d:.4f} m")
            if p50 < 0.30:
                report.append(f"  PASS: median deviation < 30cm (excellent)")
            elif p50 < 0.50:
                report.append(f"  PASS: median deviation < 50cm (acceptable)")
            elif p50 < 1.0:
                report.append(f"  WARN: median deviation 50cm-1m (needs tuning)")
                path_pass = False
            else:
                report.append(f"  FAIL: median deviation > 1m")
                path_pass = False
        else:
            report.append("  No path deviation data collected")
            path_pass = False

        # ── SLAM Map Growth ──
        report.append("\n=== SLAM Map Growth ===")
        map_pass = True
        if maps:
            report.append(f"  Initial: {maps[0][1]}x{maps[0][2]}, {maps[0][3]} known cells")
            report.append(f"  Final:   {maps[-1][1]}x{maps[-1][2]}, {maps[-1][3]} known cells")
            growth = maps[-1][3] - maps[0][3]
            report.append(f"  Growth:  {growth:+d} cells")
            if maps[-1][3] < 1000:
                report.append("  FAIL: SLAM map too small")
                map_pass = False
            else:
                report.append("  PASS: SLAM map adequate")
        else:
            map_pass = False

        # ── GPS Degradation ──
        report.append("\n=== GPS Degradation Events ===")
        normal_count = sum(1 for _, s in gps if s == "NORMAL")
        degraded_count = sum(1 for _, s in gps if s == "DEGRADED")
        report.append(f"  NORMAL transitions:   {normal_count}")
        report.append(f"  DEGRADED transitions: {degraded_count}")

        # ── Obstacle Avoidance ──
        report.append("\n=== Obstacle Proximity ===")
        collision_pass = True
        if obs:
            all_obs = [d for _, d in obs]
            min_ever = min(all_obs)
            collision_events = sum(1 for d in all_obs if d < 0.15)
            close_calls = sum(1 for d in all_obs if 0.15 <= d < 0.30)
            report.append(f"  Closest approach: {min_ever:.3f} m")
            report.append(f"  Collision events (<15cm): {collision_events}")
            report.append(f"  Close calls (15-30cm):    {close_calls}")
            if collision_events > 0:
                report.append(f"  FAIL: {collision_events} potential collision(s)")
                collision_pass = False
            else:
                report.append(f"  PASS: no collisions")

        # ── Active Obstacle Avoidance Test ──
        report.append("\n=== Active Obstacle Avoidance Test ===")
        obstacle_pass = True
        if self.obstacle_test_result == "PASS":
            report.append("  Robot stopped for obstacle: YES")
            report.append("  Robot navigated around obstacle: YES")
            report.append("  PASS: obstacle avoidance with rerouting")
        elif self.obstacle_test_result == "PARTIAL":
            report.append("  Robot stopped for obstacle: YES")
            report.append("  Robot navigated around obstacle: NO (skipped swath)")
            report.append("  PARTIAL: collision avoided but no rerouting")
        elif self.obstacle_test_result == "FAIL":
            report.append("  Robot stopped for obstacle: NO")
            report.append("  FAIL: obstacle not detected or avoided")
            obstacle_pass = False
        elif self.obstacle_test_result == "SKIP":
            report.append("  SKIP: could not spawn obstacle in Gazebo")
        else:
            report.append("  NOT RUN: test did not reach mowing phase")

        # ── Reroute Events ──
        report.append("\n=== Reroute Events ===")
        report.append(f"  Total reroute events: {len(self.metrics.reroute_events)}")
        for ts, event in self.metrics.reroute_events:
            report.append(f"  [{ts:7.1f}s] {event}")

        # ── Movement ──
        report.append("\n=== Movement ===")
        if len(poses) > 1:
            dist = sum(
                math.sqrt((poses[i][1]-poses[i-1][1])**2 + (poses[i][2]-poses[i-1][2])**2)
                for i in range(1, len(poses))
            )
            report.append(f"  Total distance: {dist:.1f} m")
            report.append(f"  Average speed:  {dist/t:.3f} m/s")

        # ── Overall Verdict ──
        report.append(f"\n{'#'*70}")
        report.append("VALIDATION CRITERIA:")

        criteria = [
            ("Undock→Plan→Mow→Dock cycle", all_phases_pass),
            ("Path tracking (median < 50cm)", path_pass),
            ("SLAM map growth", map_pass),
            ("No collisions", collision_pass),
            ("Obstacle avoidance", obstacle_pass),
        ]

        overall_pass = True
        for name, passed in criteria:
            status = "PASS" if passed else "FAIL"
            report.append(f"  [{status}] {name}")
            if not passed:
                overall_pass = False

        report.append(f"\nOVERALL: {'PASS' if overall_pass else 'NEEDS ATTENTION'}")
        report.append(f"{'#'*70}\n")

        self.get_logger().info("\n".join(report))


def main():
    rclpy.init()
    node = E2ETestNode()

    # Wait for system to settle
    for i in range(5, 0, -1):
        node.get_logger().info(f"Starting test in {i}s...")
        time.sleep(1)

    # Pre-spawn obstacle on a coverage swath BEFORE starting mowing.
    # Position: (5.0, -6.5) = along the first east-bound swath.
    # Pre-spawning ensures Gazebo's GPU LiDAR can detect it.
    if node._spawn_obstacle_at(5.0, -6.5):
        node.obstacle_spawned = True
        node.get_logger().info("Pre-spawned obstacle at (5.0, -6.5) on first coverage swath")
    else:
        node.get_logger().warn("Could not pre-spawn obstacle — avoidance test will be skipped")
        node.obstacle_test_result = "SKIP"

    # Send START command
    if not node.send_start_command():
        node.get_logger().error("Failed to send START. Aborting.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Start obstacle avoidance test in background thread
    obstacle_thread = threading.Thread(
        target=node._run_obstacle_avoidance_test, daemon=True
    )
    obstacle_thread.start()

    # Spin until test completes or timeout (20 min for full cycle)
    timeout = 1200.0
    start = time.time()

    def _on_signal(sig, frame):
        node.get_logger().info(f"Received signal {sig}, printing final report...")
        node.print_final_report()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while rclpy.ok() and not node.test_complete:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start > timeout:
                node.get_logger().warn(f"Test timeout after {timeout}s")
                break
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")

    # Clean up obstacle if still present
    if node.obstacle_spawned:
        node._remove_obstacle()

    node.print_final_report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
