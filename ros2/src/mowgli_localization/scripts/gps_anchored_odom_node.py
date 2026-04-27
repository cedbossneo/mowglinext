#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
#
# SPDX-License-Identifier: GPL-3.0
"""
gps_anchored_odom_node.py

Publishes a GPS-anchored Odometry stream + TF that slam_toolbox uses as
its motion prior, replacing the old wheel-only `wheel_odom_raw` parallel
tree.

Why
---
With the K-ICP-era parallel tree (wheel_odom_raw → base_footprint_wheels,
fed by raw wheel odometry only), slam_toolbox's internal frame `map_slam`
naturally drifts relative to the GPS-anchored `map` frame. The previous
fix was an EWMA in slam_pose_anchor_node that re-aligned `map → map_slam`
on every RTK sample. That worked at rest but left a position lag during
motion (slam_pose tracked the EWMA, RTK tracked the truth → up to 80 cm
divergence in measured tests, and the drift was visible as the slam map
"sliding" under the robot in Foxglove).

Option B (this node): provide slam_toolbox an odom that is *already*
GPS-anchored. Then slam_toolbox's pose graph is in the right frame from
birth and `map → map_slam` reduces to a static identity (or whatever
small offset accumulated since the very first scan). When RTK is healthy
GPS dominates so slam input is accurate. When RTK degrades to Float or
drops out, the dead-reckoning takes over but the map frame is still the
ENU one slam built earlier — slam scan-matches against that pre-built
map and continues guiding.

Filter
------
Tiny complementary filter (no full EKF — overkill for 3-DOF):
  state = (x, y, yaw)
  predict @ ~50 Hz from wheel.linear.x and gyro.z
  correct on RTK fix:
    if σ_xy < 0.10 m: snap x/y to GPS (RTK-Fixed: σ ~3 mm)
    elif σ_xy < 0.50 m: blend α = 0.2 toward GPS (RTK-Float / ambiguous)
    else: ignore (noisy, dead-reckon)

The "snap" is safe for slam_toolbox because:
 - At 5 Hz GPS × σ ≤ 1 cm, position jumps are sub-millimetre
 - slam_toolbox uses delta-pose between scan timestamps as motion prior,
   not absolute pose. As long as the inter-scan delta is smooth (which
   wheel+gyro gives us between GPS samples), slam is happy.

Input topics
------------
  /gps/pose_cov   (PoseWithCovarianceStamped — RTK-derived, frame=map)
  /wheel_odom     (Odometry — body twist, vx, vy ≈ 0, wz)
  /imu/data       (Imu — gyro_z for yaw rate)

Output
------
  TF: gps_odom → base_footprint_slam   (50 Hz)
  Topic: /odometry/gps_anchored        (Odometry, 50 Hz)
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class GpsAnchoredOdom(Node):
    def __init__(self) -> None:
        super().__init__("gps_anchored_odom_node")

        self._parent_frame = self.declare_parameter(
            "parent_frame", "gps_odom"
        ).value
        self._child_frame = self.declare_parameter(
            "child_frame", "base_footprint_slam"
        ).value
        # GPS σ_xy thresholds — see module docstring.
        self._snap_sigma = float(self.declare_parameter(
            "gps_snap_sigma_m", 0.10
        ).value)
        self._blend_sigma = float(self.declare_parameter(
            "gps_blend_sigma_m", 0.50
        ).value)
        self._blend_alpha = float(self.declare_parameter(
            "gps_blend_alpha", 0.2
        ).value)
        self._publish_hz = float(self.declare_parameter(
            "publish_hz", 50.0
        ).value)
        # If GPS hasn't arrived in this long, fall back to pure
        # dead-reckoning. After this we still publish the TF so
        # slam_toolbox stays happy; we just don't snap.
        self._gps_stale_after_sec = float(self.declare_parameter(
            "gps_stale_after_sec", 1.0
        ).value)

        # State.
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx_body = 0.0
        self._wz = 0.0
        self._last_predict_t: float | None = None
        self._last_gps_t: float | None = None
        self._initialised = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            PoseWithCovarianceStamped, "/gps/pose_cov",
            self._on_gps, sensor_qos
        )
        self.create_subscription(
            Odometry, "/wheel_odom", self._on_wheel, sensor_qos
        )
        self.create_subscription(
            Imu, "/imu/data", self._on_imu, sensor_qos
        )
        self._pub = self.create_publisher(
            Odometry, "/odometry/gps_anchored", 10
        )
        self._tfb = TransformBroadcaster(self)

        self.create_timer(1.0 / self._publish_hz, self._tick)
        self.create_timer(30.0, self._log_stats)
        self._n_snaps = 0
        self._n_blends = 0
        self._n_dead = 0
        self.get_logger().info(
            f"gps_anchored_odom_node up — TF {self._parent_frame} → "
            f"{self._child_frame} @ {self._publish_hz:.0f} Hz, snap "
            f"σ<{self._snap_sigma:.2f} m, blend σ<{self._blend_sigma:.2f} m"
        )

    # ──────────────────────────────────────────────────────────────
    # Sensor callbacks
    # ──────────────────────────────────────────────────────────────
    def _on_wheel(self, msg: Odometry) -> None:
        self._vx_body = msg.twist.twist.linear.x

    def _on_imu(self, msg: Imu) -> None:
        self._wz = msg.angular_velocity.z

    def _on_gps(self, msg: PoseWithCovarianceStamped) -> None:
        sxx = msg.pose.covariance[0]
        syy = msg.pose.covariance[7]
        if sxx <= 0.0 or syy <= 0.0:
            return
        sigma_xy = math.sqrt(0.5 * (sxx + syy))
        gx = msg.pose.pose.position.x
        gy = msg.pose.pose.position.y
        # First fix bootstraps the state outright — better than starting
        # at (0, 0) and snapping kilometres on the first arrival, which
        # would create a discontinuity that breaks slam_toolbox's motion
        # prior on its very first scan.
        if not self._initialised:
            self._x = gx
            self._y = gy
            # Yaw is unknown at boot. Leave 0; gyro will integrate from
            # there. cog_to_imu's yaw observation reaches the EKF on the
            # main tree, not us — slam_toolbox tolerates a constant yaw
            # offset because its scan-matching corrects for it.
            self._initialised = True
            self._last_gps_t = time.monotonic()
            self.get_logger().info(
                f"initialised state from first GPS fix: "
                f"({gx:+.3f}, {gy:+.3f}) m  σ={sigma_xy*100:.1f} cm"
            )
            return
        if sigma_xy < self._snap_sigma:
            # RTK-Fixed-grade — snap. The discontinuity is sub-mm.
            self._x = gx
            self._y = gy
            self._n_snaps += 1
        elif sigma_xy < self._blend_sigma:
            # RTK-Float-grade — pull toward GPS but keep dead-reckoning's
            # smoothness so slam_toolbox sees continuous motion.
            a = self._blend_alpha
            self._x = (1 - a) * self._x + a * gx
            self._y = (1 - a) * self._y + a * gy
            self._n_blends += 1
        else:
            self._n_dead += 1
            return
        self._last_gps_t = time.monotonic()

    # ──────────────────────────────────────────────────────────────
    # Predict + publish
    # ──────────────────────────────────────────────────────────────
    def _tick(self) -> None:
        if not self._initialised:
            return
        now = time.monotonic()
        if self._last_predict_t is None:
            self._last_predict_t = now
            self._publish(now)
            return
        dt = now - self._last_predict_t
        self._last_predict_t = now
        if dt <= 0.0 or dt > 0.5:
            self._publish(now)
            return

        # Integrate yaw from gyro.
        self._yaw = wrap(self._yaw + self._wz * dt)
        # Integrate x/y from body twist projected through current yaw.
        # Only dead-reckon when GPS is stale; otherwise the snap/blend
        # in _on_gps already drives x/y, and integrating twist on top
        # would double-count between samples.
        gps_stale = (
            self._last_gps_t is None
            or (now - self._last_gps_t) > self._gps_stale_after_sec
        )
        if gps_stale:
            cy = math.cos(self._yaw)
            sy = math.sin(self._yaw)
            self._x += self._vx_body * cy * dt
            self._y += self._vx_body * sy * dt
        # Else: trust the latest snap/blend; gyro still drives yaw so
        # the TF rotation is continuous between GPS samples.

        self._publish(now)

    def _publish(self, _now: float) -> None:
        stamp = self.get_clock().now().to_msg()
        cy = math.cos(self._yaw / 2.0)
        sy = math.sin(self._yaw / 2.0)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._parent_frame
        tf.child_frame_id = self._child_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z = sy
        tf.transform.rotation.w = cy
        self._tfb.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._parent_frame
        odom.child_frame_id = self._child_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        odom.twist.twist.linear.x = self._vx_body
        odom.twist.twist.angular.z = self._wz
        self._pub.publish(odom)

    def _log_stats(self) -> None:
        self.get_logger().info(
            f"gps_anchored_odom: snaps={self._n_snaps} blends="
            f"{self._n_blends} dead={self._n_dead}  state=("
            f"{self._x:+.2f}, {self._y:+.2f}, "
            f"{math.degrees(self._yaw):+.1f}°)"
        )
        self._n_snaps = self._n_blends = self._n_dead = 0


def main() -> None:
    rclpy.init()
    n = GpsAnchoredOdom()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
