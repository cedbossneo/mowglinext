#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
slam_pose_anchor_node.py

Continuously aligns slam_toolbox's internal `map_slam` frame to the
GPS-anchored `map` frame and republishes slam's pose into the GPS map
frame as a PoseWithCovarianceStamped on `/slam/pose_cov` for
ekf_map_node to fuse as `pose1`.

Architecture
------------
slam_toolbox runs on a parallel TF tree:

    map_slam -> wheel_odom_raw -> base_footprint_wheels -> lidar_link_wheels

The fused EKF tree is independent:

    map -> odom -> base_footprint -> base_link -> lidar_link

`base_footprint` and `base_footprint_wheels` represent the same physical
robot center, just looked up through different TF chains. The rigid
transform that aligns them gives us `T_anchor : map -> map_slam`:

    T_anchor = T_map_to_base_footprint  *  inv(T_map_slam_to_base_footprint_wheels)

Update policy (EWMA on RTK-Fixed)
---------------------------------
On every tick (default 5 Hz):

  1. Look up both TFs at the latest available time.
  2. Read the latest /gps/fix and /odometry/filtered_map.
  3. If status == GBAS_FIX (RTK Fixed) AND ekf cov_xx < gating_threshold,
     compute the candidate T_anchor and blend into the running anchor:
        anchor.x   = (1 - α) anchor.x   + α candidate.x
        anchor.y   = (1 - α) anchor.y   + α candidate.y
        anchor.yaw = anchor.yaw + α * shortest_angle(candidate.yaw - anchor.yaw)
     The first valid candidate seeds the anchor outright (no EWMA).
  4. Whether or not the anchor was updated, broadcast TF
     `map -> map_slam` from the running anchor (so consumers see a
     valid frame chain even during RTK degradation).
  5. Compose `pose_in_map = anchor * slam_pose` and publish
     `/slam/pose_cov` (PoseWithCovarianceStamped, frame=map). Covariance:
        sigma_xy(t) = floor + drift_rate * (now - last_rtk_fixed_t)
     yaw covariance is held very high (1e3) so the EKF fuses slam x/y
     only. cog_to_imu + mag_yaw_publisher already cover absolute yaw,
     and slam yaw drifts in featureless outdoor terrain.

Always publishes /slam/pose_cov — the EKF gates by covariance. With
RTK-Fixed sigma ~3 mm, GPS dominates; with RTK-Float sigma ~30 cm, slam
and GPS compete; with no fix, slam wins.

Safety: read-only consumer of TF, NavSatFix, and Odometry. Publishes
one TF (map -> map_slam) and one topic (/slam/pose_cov). Does not touch
drive commands, the fused odom TF, or any safety topic.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def _shortest_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _quat_to_yaw_2d(qx: float, qy: float, qz: float, qw: float) -> float:
    """2D yaw from a quaternion (assumes roll, pitch ~ 0 — two_d_mode)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class Pose2D:
    """2D rigid transform (x, y, yaw)."""

    x: float
    y: float
    yaw: float

    def compose(self, other: "Pose2D") -> "Pose2D":
        """self * other (interpreted as transforms)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(
            x=self.x + c * other.x - s * other.y,
            y=self.y + s * other.x + c * other.y,
            yaw=_shortest_angle(self.yaw + other.yaw),
        )

    def inverse(self) -> "Pose2D":
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        return Pose2D(
            x=c * (-self.x) - s * (-self.y),
            y=s * (-self.x) + c * (-self.y),
            yaw=-self.yaw,
        )


def _tf_to_pose2d(tf: TransformStamped) -> Pose2D:
    return Pose2D(
        x=tf.transform.translation.x,
        y=tf.transform.translation.y,
        yaw=_quat_to_yaw_2d(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ),
    )


class SlamPoseAnchorNode(Node):
    """EWMA-anchor + always-publish slam pose into the GPS map frame."""

    def __init__(self) -> None:
        super().__init__("slam_pose_anchor_node")

        # ---- Frames ----
        self._gps_map_frame = self.declare_parameter(
            "gps_map_frame", "map"
        ).value
        self._gps_base_frame = self.declare_parameter(
            "gps_base_frame", "base_footprint"
        ).value
        self._slam_map_frame = self.declare_parameter(
            "slam_map_frame", "map_slam"
        ).value
        self._slam_base_frame = self.declare_parameter(
            "slam_base_frame", "base_footprint_wheels"
        ).value

        # ---- Update policy ----
        # Tick rate for anchor update + /slam/pose_cov publish. 5 Hz keeps
        # the EKF fed without spamming.
        self._tick_hz = float(
            self.declare_parameter("tick_hz", 5.0).value
        )
        # EWMA gain per RTK-Fixed update. 0.05 -> ~20-update time constant
        # (~4 s at 5 Hz). Slow enough to reject single-sample TF jitter,
        # fast enough to track slam drift between RTK-Fixed windows.
        self._ewma_alpha = float(
            self.declare_parameter("ewma_alpha", 0.05).value
        )
        # EKF cov_xx threshold to count an RTK-Fixed sample as "good
        # enough" to update the anchor. 1e-2 m^2 ~= 10 cm sigma.
        self._ekf_cov_threshold = float(
            self.declare_parameter("ekf_cov_threshold", 1e-2).value
        )

        # ---- Output covariance ----
        # Floor sigma on /slam/pose_cov.position when the anchor is fresh —
        # represents slam_toolbox matcher uncertainty in well-featured
        # outdoor terrain. Tuned conservatively; can be lowered if
        # session traces show slam pose tracks GPS sub-decimeter.
        self._sigma_xy_floor = float(
            self.declare_parameter("sigma_xy_floor", 0.10).value
        )
        # Per-second growth in sigma_xy as the anchor ages (i.e., as time
        # since the last RTK-Fixed sample increases). Models slam drift.
        self._sigma_xy_drift_rate = float(
            self.declare_parameter("sigma_xy_drift_rate", 0.01).value
        )
        # Cap on sigma_xy so a multi-hour GPS outage doesn't make the EKF
        # ignore slam entirely.
        self._sigma_xy_cap = float(
            self.declare_parameter("sigma_xy_cap", 5.0).value
        )
        # Yaw covariance — held very high so ekf_map_node ignores slam
        # yaw. cog_to_imu and mag_yaw_publisher cover absolute yaw.
        self._yaw_var_unused = float(
            self.declare_parameter("yaw_var_unused", 1.0e3).value
        )

        # ---- TF + subscriptions ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._latest_fix: Optional[NavSatFix] = None
        self._latest_ekf: Optional[Odometry] = None
        self._anchor: Optional[Pose2D] = None
        # Wallclock time of the most recent anchor update from an
        # RTK-Fixed sample. None until the first seed; used to grow
        # /slam/pose_cov's covariance during RTK degradation.
        self._last_rtk_update_t: Optional[float] = None

        self.create_subscription(
            NavSatFix, "/gps/fix", self._on_fix, sensor_qos
        )
        self.create_subscription(
            Odometry,
            "/odometry/filtered_map",
            self._on_ekf,
            reliable_qos,
        )

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/slam/pose_cov", reliable_qos
        )

        # ---- Diagnostics ----
        self._published = 0
        self._anchor_updates = 0
        self._tf_failures = 0
        self._gating_skipped_no_rtk = 0
        self._gating_skipped_high_cov = 0
        self.create_timer(30.0, self._log_stats)

        # ---- Main tick ----
        self.create_timer(1.0 / self._tick_hz, self._on_tick)

        self.get_logger().info(
            "slam_pose_anchor_node ready: anchoring %s -> %s, "
            "publishing /slam/pose_cov in %s frame at %.1f Hz "
            "(alpha=%.3f, sigma_xy_floor=%.2f m, drift=%.3f m/s)"
            % (
                self._gps_map_frame,
                self._slam_map_frame,
                self._gps_map_frame,
                self._tick_hz,
                self._ewma_alpha,
                self._sigma_xy_floor,
                self._sigma_xy_drift_rate,
            )
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _on_ekf(self, msg: Odometry) -> None:
        self._latest_ekf = msg

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    def _on_tick(self) -> None:
        now = self.get_clock().now()

        # Look up both TFs at TimePointZero so we get the latest available
        # transform on each side. Asking for "now" exact would race the
        # broadcasters and fail every tick.
        try:
            tf_gps = self._tf_buffer.lookup_transform(
                self._gps_map_frame,
                self._gps_base_frame,
                rclpy.time.Time(),
            )
            tf_slam = self._tf_buffer.lookup_transform(
                self._slam_map_frame,
                self._slam_base_frame,
                rclpy.time.Time(),
            )
        except Exception:  # noqa: BLE001 - tf2 throws several subclasses
            self._tf_failures += 1
            return

        gps_pose = _tf_to_pose2d(tf_gps)
        slam_pose = _tf_to_pose2d(tf_slam)

        # ---- Anchor update (gated on RTK-Fixed + low EKF cov) ----
        if self._gate_passes():
            candidate = gps_pose.compose(slam_pose.inverse())
            if self._anchor is None:
                # First valid candidate seeds the anchor outright.
                self._anchor = candidate
            else:
                a = self._ewma_alpha
                self._anchor = Pose2D(
                    x=(1.0 - a) * self._anchor.x + a * candidate.x,
                    y=(1.0 - a) * self._anchor.y + a * candidate.y,
                    yaw=_shortest_angle(
                        self._anchor.yaw
                        + a * _shortest_angle(candidate.yaw - self._anchor.yaw)
                    ),
                )
            self._last_rtk_update_t = now.nanoseconds * 1e-9
            self._anchor_updates += 1

        # Without a seeded anchor we can't compose a sane pose yet.
        if self._anchor is None:
            return

        # ---- Broadcast map -> map_slam from the running anchor ----
        self._broadcast_anchor_tf(now.to_msg())

        # ---- Publish /slam/pose_cov ----
        pose_in_map = self._anchor.compose(slam_pose)
        sigma_xy = self._current_sigma_xy(now.nanoseconds * 1e-9)
        self._publish_pose_cov(now.to_msg(), pose_in_map, sigma_xy)

    def _gate_passes(self) -> bool:
        if self._latest_fix is None:
            self._gating_skipped_no_rtk += 1
            return False
        if self._latest_fix.status.status != NavSatStatus.STATUS_GBAS_FIX:
            self._gating_skipped_no_rtk += 1
            return False
        if self._latest_ekf is None:
            self._gating_skipped_high_cov += 1
            return False
        # ekf cov_xx is index 0 of the 6x6 row-major covariance.
        if self._latest_ekf.pose.covariance[0] > self._ekf_cov_threshold:
            self._gating_skipped_high_cov += 1
            return False
        return True

    def _current_sigma_xy(self, now_t: float) -> float:
        if self._last_rtk_update_t is None:
            return self._sigma_xy_cap
        age = max(0.0, now_t - self._last_rtk_update_t)
        sigma = self._sigma_xy_floor + self._sigma_xy_drift_rate * age
        return min(sigma, self._sigma_xy_cap)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def _broadcast_anchor_tf(self, stamp) -> None:
        assert self._anchor is not None
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._gps_map_frame
        tf.child_frame_id = self._slam_map_frame
        tf.transform.translation.x = self._anchor.x
        tf.transform.translation.y = self._anchor.y
        tf.transform.translation.z = 0.0
        half = 0.5 * self._anchor.yaw
        tf.transform.rotation.z = math.sin(half)
        tf.transform.rotation.w = math.cos(half)
        self._tf_broadcaster.sendTransform(tf)

    def _publish_pose_cov(
        self, stamp, pose: Pose2D, sigma_xy: float
    ) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._gps_map_frame
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.position.z = 0.0
        half = 0.5 * pose.yaw
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)

        var_xy = sigma_xy * sigma_xy
        cov = [0.0] * 36
        cov[0] = var_xy           # xx
        cov[7] = var_xy           # yy
        cov[14] = var_xy * 4.0    # zz — looser, two_d_mode ignores
        cov[21] = 1.0e3           # roll — unused
        cov[28] = 1.0e3           # pitch — unused
        cov[35] = self._yaw_var_unused  # yaw — held high so EKF skips it
        msg.pose.covariance = cov
        self._pose_pub.publish(msg)
        self._published += 1

    def _log_stats(self) -> None:
        sigma_now = (
            self._current_sigma_xy(
                self.get_clock().now().nanoseconds * 1e-9
            )
            if self._anchor is not None
            else float("nan")
        )
        anchor_age = (
            (self.get_clock().now().nanoseconds * 1e-9)
            - self._last_rtk_update_t
            if self._last_rtk_update_t is not None
            else float("nan")
        )
        self.get_logger().info(
            "slam_pose_anchor: published=%d, anchor_updates=%d, "
            "tf_failures=%d, skipped_no_rtk=%d, skipped_high_cov=%d, "
            "anchor_age=%.1fs, sigma_xy=%.3f m"
            % (
                self._published,
                self._anchor_updates,
                self._tf_failures,
                self._gating_skipped_no_rtk,
                self._gating_skipped_high_cov,
                anchor_age,
                sigma_now,
            )
        )
        self._published = 0
        self._anchor_updates = 0
        self._tf_failures = 0
        self._gating_skipped_no_rtk = 0
        self._gating_skipped_high_cov = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamPoseAnchorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
