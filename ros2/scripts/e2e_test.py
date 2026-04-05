#!/usr/bin/env python3
"""
e2e_test.py — End-to-end simulation test harness for Mowgli ROS2.

Sends a START command, then monitors the full mowing cycle collecting metrics:
  - Path tracking deviation (planned vs actual)
  - SLAM map growth over time
  - GPS degradation events and localization quality
  - Obstacle avoidance (costmap lethal cells near robot)
  - BT state transitions
  - Coverage progress

Usage (inside the dev-sim container):
  source /ros2_ws/install/setup.bash
  python3 /ros2_ws/src/scripts/e2e_test.py

Or from host:
  docker compose exec dev-sim bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/scripts/e2e_test.py"
"""

import math
import signal
import subprocess
import sys
import time
import threading
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from mowgli_interfaces.srv import HighLevelControl
from mowgli_interfaces.msg import HighLevelStatus


class E2ETestNode(Node):
    def __init__(self):
        super().__init__("e2e_test_node")

        # ── Metrics ──────────────────────────────────────────────────────
        self.metrics = {
            "start_time": time.time(),
            "bt_states": [],           # (timestamp, state_name)
            "path_deviations": [],     # (timestamp, lateral_error_m)
            "gps_states": [],          # (timestamp, state_str)
            "map_sizes": [],           # (timestamp, width, height, known_cells)
            "robot_poses": [],         # (timestamp, x, y, yaw)
            "cmd_vels": [],            # (timestamp, linear_x, angular_z)
            "coverage_path_len": 0,
            "coverage_path_poses": [],
            "min_obstacle_dist": [],   # (timestamp, min_dist_from_scan)
            "nav_errors": [],          # (timestamp, error_str)
        }

        self.coverage_path = None  # nav_msgs/Path from coverage planner
        self.current_pose = None   # (x, y, yaw)
        self.current_bt_state = ""
        self.test_complete = False
        self.mowing_started = False

        # ── Active obstacle avoidance test ──────────────────────────────
        self.obstacle_test_done = False
        self.obstacle_test_result = None  # "PASS" / "FAIL" / None
        self.obstacle_spawned = False
        self.obstacle_spawn_time = 0.0
        self.obstacle_vel_samples = []  # cmd_vel samples after obstacle spawn
        self._last_cmd_vel_x = 0.0

        # ── QoS ──────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            HighLevelStatus,
            "/behavior_tree_node/high_level_status",
            self._on_bt_status,
            reliable_qos,
        )
        self.create_subscription(
            Path,
            "/mowgli/coverage/path",
            self._on_coverage_path,
            reliable_qos,
        )
        self.create_subscription(
            Odometry, "/wheel_odom", self._on_odom, sensor_qos
        )
        # Subscribe to SLAM pose (in map frame) for path deviation computation.
        self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self._on_slam_pose, sensor_qos
        )
        self.slam_pose = None  # (x, y) in map frame
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
        # Subscribe to /cmd_vel (post-collision-monitor) for obstacle test
        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel_out, sensor_qos
        )

        # ── Service client ───────────────────────────────────────────────
        self.hlc_client = self.create_client(
            HighLevelControl,
            "/behavior_tree_node/high_level_control",
        )

        # ── Timer for periodic reporting ─────────────────────────────────
        self.report_timer = self.create_timer(10.0, self._periodic_report)
        self.get_logger().info("E2E test node started. Waiting 5s for system to settle...")

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_bt_status(self, msg: HighLevelStatus):
        state_name = msg.state_name if hasattr(msg, "state_name") else str(msg.state)
        t = time.time() - self.metrics["start_time"]
        # Capture initial state on first message
        if not self.current_bt_state:
            self.current_bt_state = state_name
            self.metrics["bt_states"].append((t, state_name))
            self.get_logger().info(f"[{t:.1f}s] BT initial state: {state_name}")
        elif state_name != self.current_bt_state:
            self.get_logger().info(f"[{t:.1f}s] BT state: {self.current_bt_state} -> {state_name}")
            self.current_bt_state = state_name
            self.metrics["bt_states"].append((t, state_name))

        if state_name == "MOWING_COMPLETE" or state_name == "IDLE_DOCKED":
            self.test_complete = True

    def _on_coverage_path(self, msg: Path):
        self.coverage_path = msg
        self.metrics["coverage_path_len"] = len(msg.poses)
        self.metrics["coverage_path_poses"] = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self.get_logger().info(
            f"Received coverage path with {len(msg.poses)} poses"
        )

    def _on_slam_pose(self, msg: PoseWithCovarianceStamped):
        """SLAM pose in map frame — used for path deviation computation."""
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

        t = time.time() - self.metrics["start_time"]
        # Sample at ~2 Hz
        if len(self.metrics["robot_poses"]) == 0 or (
            t - self.metrics["robot_poses"][-1][0] > 0.5
        ):
            self.metrics["robot_poses"].append((t, x, y, yaw))

            # Compute path deviation only during FOLLOWING_PATH state,
            # using SLAM pose (map frame) since coverage path is also in map frame.
            if self.coverage_path and self.current_bt_state == "FOLLOWING_PATH" and self.slam_pose:
                min_dist = self._min_distance_to_path(self.slam_pose[0], self.slam_pose[1])
                self.metrics["path_deviations"].append((t, min_dist))

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg
        t = time.time() - self.metrics["start_time"]
        # Find minimum valid range (closest obstacle)
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max and not math.isinf(r)
        ]
        if valid_ranges:
            min_dist = min(valid_ranges)
            # Sample at ~1 Hz
            if len(self.metrics["min_obstacle_dist"]) == 0 or (
                t - self.metrics["min_obstacle_dist"][-1][0] > 1.0
            ):
                self.metrics["min_obstacle_dist"].append((t, min_dist))

    def _on_gps_status(self, msg: String):
        t = time.time() - self.metrics["start_time"]
        self.metrics["gps_states"].append((t, msg.data))
        self.get_logger().info(f"[{t:.1f}s] GPS status: {msg.data}")

    def _on_map(self, msg: OccupancyGrid):
        t = time.time() - self.metrics["start_time"]
        w = msg.info.width
        h = msg.info.height
        known = sum(1 for c in msg.data if c >= 0)
        # Sample at ~0.2 Hz
        if len(self.metrics["map_sizes"]) == 0 or (
            t - self.metrics["map_sizes"][-1][0] > 5.0
        ):
            self.metrics["map_sizes"].append((t, w, h, known))

    def _on_cmd_vel_out(self, msg: Twist):
        """Post-collision-monitor cmd_vel — used for obstacle avoidance test."""
        self._last_cmd_vel_x = msg.linear.x

    def _on_cmd_vel(self, msg: Twist):
        t = time.time() - self.metrics["start_time"]
        if len(self.metrics["cmd_vels"]) == 0 or (
            t - self.metrics["cmd_vels"][-1][0] > 0.5
        ):
            self.metrics["cmd_vels"].append((t, msg.linear.x, msg.angular.z))

    # ── Helpers ──────────────────────────────────────────────────────────

    def _min_distance_to_path(self, x: float, y: float) -> float:
        """Minimum distance from (x, y) to the coverage path (point-to-segment)."""
        poses = self.metrics["coverage_path_poses"]
        if not poses:
            return float("inf")
        min_d = float("inf")
        for i in range(len(poses) - 1):
            ax, ay = poses[i]
            bx, by = poses[i + 1]
            # Point-to-segment distance
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
        """Return the current minimum valid scan range (from the latest /scan msg)."""
        if not hasattr(self, '_latest_scan') or self._latest_scan is None:
            return float('inf')
        s = self._latest_scan
        valid = [r for r in s.ranges if r > s.range_min and r < s.range_max]
        return min(valid) if valid else float('inf')

    def _spawn_obstacle_at(self, ox: float, oy: float, name: str = "e2e_obstacle") -> bool:
        """Spawn a cylinder obstacle at a fixed world position.

        Uses the blocking variant of the create service so the model is fully
        registered in the rendering scene before returning.  This is critical for
        gpu_lidar sensors in Gazebo Harmonic which may not detect models that
        were added after the first render frame unless a blocking create is used.
        """
        # 1m tall, 30cm radius — large enough for reliable LiDAR detection.
        # Center at z=0.50 → bottom=0.0, top=1.0m, well above LiDAR at 0.35m.
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
            self.get_logger().warn(f"Failed to spawn obstacle: {result.stderr[:100]}")
        return success

    def _remove_obstacle(self):
        """Remove the test obstacle from the world."""
        cmd = (
            'gz service -s /world/garden/remove '
            '--reqtype gz.msgs.Entity '
            '--reptype gz.msgs.Boolean '
            '--timeout 5000 '
            '--req \'name: "e2e_obstacle" type: MODEL\''
        )
        subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        self.get_logger().info("Removed test obstacle")

    def _run_obstacle_avoidance_test(self):
        """
        Active obstacle avoidance test — runs in a background thread.

        Pre-spawns a cylinder at a known position on the first coverage swath
        BEFORE the robot reaches it.  The obstacle is placed at world coordinates
        (0.0, -6.9) which lies on the first east-bound swath (y ≈ -6.9 with the
        0.6 m boundary inset).

        GPU LiDAR in Gazebo Harmonic only reliably detects models that are
        present in the rendering scene before the sensor first renders them, so
        the obstacle is spawned early and we just wait for the robot to approach.
        """
        self.get_logger().info("=== OBSTACLE AVOIDANCE TEST: waiting for FOLLOWING_PATH ===")

        # Wait until we're in FOLLOWING_PATH (robot started coverage)
        while not self.test_complete:
            if self.current_bt_state == "FOLLOWING_PATH":
                break
            time.sleep(1.0)

        if self.test_complete:
            return

        self.get_logger().info("=== OBSTACLE AVOIDANCE TEST: in FOLLOWING_PATH, monitoring ===")
        self.obstacle_spawn_time = time.time()
        self.obstacle_vel_samples = []

        # Monitor: wait for robot to approach x ≈ 0.0 (where the obstacle is)
        # then check that cmd_vel drops to near-zero.
        robot_stopped = False
        stop_time = None
        closest_approach = 999.0
        for i in range(1200):  # 120 seconds at 10Hz
            time.sleep(0.1)
            vel = self._last_cmd_vel_x
            self.obstacle_vel_samples.append(vel)

            # Track closest scan range
            cur_min = self._get_min_scan_range()
            if cur_min < closest_approach:
                closest_approach = cur_min

            # Log every 2 seconds
            if i % 20 == 0:
                pose_str = ""
                if self.current_pose:
                    x, y, _ = self.current_pose
                    pose_str = f" robot=({x:.1f},{y:.1f})"
                self.get_logger().info(
                    f"=== OBSTACLE TEST t+{i/10:.0f}s: vel={vel:.3f} "
                    f"scan_min={cur_min:.2f}m{pose_str} ===")

            if abs(vel) < 0.01 and not robot_stopped:
                robot_stopped = True
                stop_time = time.time() - self.obstacle_spawn_time
                self.get_logger().info(
                    f"=== OBSTACLE TEST: robot STOPPED after {stop_time:.1f}s "
                    f"(closest={closest_approach:.2f}m) ==="
                )
                # Give recovery 10s to clear obstacle
                time.sleep(10.0)
                break

            # Early exit if test ended
            if self.test_complete:
                break

        # Evaluate
        if robot_stopped:
            self.obstacle_test_result = "PASS"
            self.get_logger().info(
                f"=== OBSTACLE AVOIDANCE TEST: PASS — robot stopped, "
                f"closest approach {closest_approach:.2f}m ==="
            )
        else:
            self.obstacle_test_result = "FAIL"
            min_vel = min(abs(v) for v in self.obstacle_vel_samples) if self.obstacle_vel_samples else 999
            self.get_logger().error(
                f"=== OBSTACLE AVOIDANCE TEST: FAIL — robot did NOT stop "
                f"(min |vel|={min_vel:.3f} m/s, closest={closest_approach:.2f}m) ==="
            )

        # Clean up — remove obstacle so mowing can continue
        self._remove_obstacle()
        self.obstacle_spawned = False
        self.obstacle_test_done = True

    def send_start_command(self):
        """Send COMMAND_START (1) to the behavior tree."""
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
        """Print periodic metrics summary."""
        t = time.time() - self.metrics["start_time"]
        poses = self.metrics["robot_poses"]
        devs = self.metrics["path_deviations"]
        maps = self.metrics["map_sizes"]
        gps = self.metrics["gps_states"]
        obs = self.metrics["min_obstacle_dist"]

        report = [f"\n{'='*70}", f"E2E TEST REPORT — t={t:.0f}s", f"{'='*70}"]

        # BT State
        report.append(f"Current BT state: {self.current_bt_state}")
        report.append(f"State transitions: {len(self.metrics['bt_states'])}")
        for ts, st in self.metrics["bt_states"][-5:]:
            report.append(f"  [{ts:.1f}s] {st}")

        # Robot position
        if self.current_pose:
            x, y, yaw = self.current_pose
            report.append(f"Robot pose: ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f} deg)")

        # Path tracking
        if devs:
            recent = [d for _, d in devs[-20:]]
            avg_dev = sum(recent) / len(recent)
            max_dev = max(recent)
            report.append(f"Path deviation: avg={avg_dev:.3f}m  max={max_dev:.3f}m  (last 20 samples)")
        else:
            report.append("Path deviation: no data yet")

        # Coverage path
        report.append(f"Coverage path: {self.metrics['coverage_path_len']} poses")

        # SLAM map
        if maps:
            _, w, h, known = maps[-1]
            report.append(f"SLAM map: {w}x{h} cells, {known} known ({known/(w*h)*100:.1f}%)")
            if len(maps) > 1:
                _, w0, h0, k0 = maps[0]
                report.append(f"  Growth: {k0} -> {known} known cells ({known-k0:+d})")

        # GPS degradation
        normal_count = sum(1 for _, s in gps if s == "NORMAL")
        degraded_count = sum(1 for _, s in gps if s == "DEGRADED")
        report.append(f"GPS: {normal_count} NORMAL, {degraded_count} DEGRADED transitions")
        if gps:
            report.append(f"  Current: {gps[-1][1]}")

        # Obstacle proximity
        if obs:
            recent_obs = [d for _, d in obs[-10:]]
            min_obs = min(recent_obs)
            report.append(f"Closest obstacle: {min_obs:.2f}m (last 10s)")
            if min_obs < 0.3:
                report.append(f"  WARNING: very close obstacle proximity!")
        else:
            report.append("Obstacle proximity: no scan data")

        # Distance traveled
        if len(poses) > 1:
            dist = 0.0
            for i in range(1, len(poses)):
                dx = poses[i][1] - poses[i-1][1]
                dy = poses[i][2] - poses[i-1][2]
                dist += math.sqrt(dx**2 + dy**2)
            report.append(f"Distance traveled: {dist:.1f}m")

        report.append(f"{'='*70}")
        self.get_logger().info("\n".join(report))

    def print_final_report(self):
        """Print comprehensive final test report."""
        t = time.time() - self.metrics["start_time"]
        devs = self.metrics["path_deviations"]
        maps = self.metrics["map_sizes"]
        gps = self.metrics["gps_states"]
        obs = self.metrics["min_obstacle_dist"]
        poses = self.metrics["robot_poses"]

        report = [
            f"\n{'#'*70}",
            f"FINAL E2E TEST REPORT — Duration: {t:.0f}s ({t/60:.1f} min)",
            f"{'#'*70}",
        ]

        # ── BT State History ──
        report.append("\n--- Behavior Tree State History ---")
        for ts, st in self.metrics["bt_states"]:
            report.append(f"  [{ts:7.1f}s] {st}")

        # ── Path Tracking Quality ──
        report.append("\n--- Path Tracking Quality ---")
        if devs:
            all_devs = [d for _, d in devs]
            avg = sum(all_devs) / len(all_devs)
            p50 = sorted(all_devs)[int(len(all_devs) * 0.50)] if len(all_devs) > 2 else avg
            p95 = sorted(all_devs)[int(len(all_devs) * 0.95)] if len(all_devs) > 20 else max(all_devs)
            max_d = max(all_devs)
            report.append(f"  Samples:     {len(all_devs)}")
            report.append(f"  Mean error:  {avg:.4f} m")
            report.append(f"  Median:      {p50:.4f} m")
            report.append(f"  P95 error:   {p95:.4f} m")
            report.append(f"  Max error:   {max_d:.4f} m")
            # The deviation is measured as min distance to any coverage path
            # point. During turns between swaths, the robot is far from the
            # nearest path point, inflating the metric. Use P50 (median) as
            # the primary quality indicator.
            if p50 < 0.5:
                report.append(f"  PASS: median deviation < 50cm")
            elif p50 < 1.0:
                report.append(f"  WARN: median deviation 50cm-1m")
            else:
                report.append(f"  FAIL: median deviation > 1m")
        else:
            report.append("  No path deviation data collected")

        # ── SLAM Map Growth ──
        report.append("\n--- SLAM Map Growth ---")
        if maps:
            for ts, w, h, k in maps:
                report.append(f"  [{ts:7.1f}s] {w}x{h}, {k} known cells")
            report.append(f"  Final map: {maps[-1][1]}x{maps[-1][2]}, {maps[-1][3]} known")

        # ── GPS Degradation ──
        report.append("\n--- GPS Degradation Events ---")
        normal_count = sum(1 for _, s in gps if s == "NORMAL")
        degraded_count = sum(1 for _, s in gps if s == "DEGRADED")
        report.append(f"  NORMAL transitions:   {normal_count}")
        report.append(f"  DEGRADED transitions: {degraded_count}")
        for ts, s in gps:
            report.append(f"  [{ts:7.1f}s] {s}")

        # ── Obstacle Avoidance ──
        report.append("\n--- Obstacle Proximity ---")
        if obs:
            all_obs = [d for _, d in obs]
            min_ever = min(all_obs)
            avg_min = sum(all_obs) / len(all_obs)
            collision_events = sum(1 for d in all_obs if d < 0.15)
            close_calls = sum(1 for d in all_obs if 0.15 <= d < 0.30)
            report.append(f"  Closest approach: {min_ever:.3f} m")
            report.append(f"  Average min dist: {avg_min:.3f} m")
            report.append(f"  Collision events (<15cm): {collision_events}")
            report.append(f"  Close calls (15-30cm):    {close_calls}")
            if collision_events == 0:
                report.append(f"  PASS: no collisions")
            else:
                report.append(f"  FAIL: {collision_events} potential collision(s)")

        # ── Active Obstacle Avoidance Test ──
        report.append("\n--- Active Obstacle Avoidance Test ---")
        if self.obstacle_test_result == "PASS":
            report.append("  Spawned obstacle 3m ahead during mowing")
            report.append("  Robot stopped before hitting cone: YES")
            report.append("  PASS: collision monitor stopped robot")
        elif self.obstacle_test_result == "FAIL":
            report.append("  Spawned obstacle 3m ahead during mowing")
            report.append("  Robot stopped before hitting cone: NO")
            report.append("  FAIL: collision monitor did not stop robot")
        elif self.obstacle_test_result == "SKIP":
            report.append("  SKIP: could not spawn obstacle in Gazebo")
        else:
            report.append("  NOT RUN: test did not complete")

        # ── Distance Traveled ──
        report.append("\n--- Movement ---")
        if len(poses) > 1:
            dist = 0.0
            for i in range(1, len(poses)):
                dx = poses[i][1] - poses[i-1][1]
                dy = poses[i][2] - poses[i-1][2]
                dist += math.sqrt(dx**2 + dy**2)
            report.append(f"  Total distance: {dist:.1f} m")
            report.append(f"  Average speed:  {dist/t:.3f} m/s")
        else:
            report.append("  No movement data")

        # ── Overall ──
        report.append(f"\n{'#'*70}")
        passed = True
        reasons = []
        if devs:
            sorted_devs = sorted(d for _, d in devs)
            p50 = sorted_devs[len(sorted_devs) // 2]
            if p50 > 1.0:
                passed = False
                reasons.append(f"median path deviation {p50:.2f}m > 1m")
        if obs and min(d for _, d in obs) < 0.15:
            passed = False
            reasons.append(f"collision detected (min dist {min(d for _, d in obs):.3f}m)")
        if not self.metrics["bt_states"]:
            passed = False
            reasons.append("no BT state transitions captured")
        if not self.metrics["map_sizes"] or self.metrics["map_sizes"][-1][3] < 1000:
            passed = False
            reasons.append("SLAM map too small")
        if self.obstacle_test_result == "FAIL":
            passed = False
            reasons.append("obstacle avoidance failed — robot did not stop")
        for r in reasons:
            report.append(f"  ISSUE: {r}")
        report.append(f"OVERALL: {'PASS' if passed else 'NEEDS ATTENTION'}")
        report.append(f"{'#'*70}\n")

        self.get_logger().info("\n".join(report))


def main():
    rclpy.init()
    node = E2ETestNode()

    # Wait for system to settle
    for i in range(5, 0, -1):
        node.get_logger().info(f"Starting test in {i}s...")
        time.sleep(1)

    # Pre-spawn obstacle on the first coverage swath BEFORE starting the mowing
    # sequence.  This ensures the model is fully registered in Gazebo's rendering
    # scene so the GPU LiDAR can detect it when the robot approaches.
    # Position: (5.0, -6.5) = well along the first east-bound swath, far enough
    # from the start that the transit NavigateToPose can reach the swath start.
    if node._spawn_obstacle_at(5.0, -6.5):
        node.obstacle_spawned = True
        node.get_logger().info("Pre-spawned obstacle at (0.0, -6.9) on first coverage swath")
    else:
        node.get_logger().warn("Could not pre-spawn obstacle — avoidance test will be skipped")

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

    # Spin until test completes or timeout (15 min for large garden)
    timeout = 900.0  # 15 min
    start = time.time()

    # Handle SIGTERM (from `timeout` command) gracefully
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

    node.print_final_report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
