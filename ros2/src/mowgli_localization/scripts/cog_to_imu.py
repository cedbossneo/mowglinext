#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
#
# SPDX-License-Identifier: GPL-3.0
"""
cog_to_imu.py

Continuous yaw correction for ekf_map_node: derives a GPS course-over-ground
heading from successive RTK-Fixed positions and publishes it as a
sensor_msgs/Imu absolute-yaw observation.

Why this node exists:
  The current stack seeds yaw once per session (dock_yaw / undock BackUp /
  SeedYawFromMotion), then relies on pure gyro integration. Over a multi-
  minute mow that drifts by several degrees — enough for the Smac planner
  to start targeting strips slightly off-axis. This node fills that gap
  for robot_localization without needing a custom EKF fork.

Design:
  Input : /gps/fix           (raw NavSatFix — RAW antenna ENU positions
                              are projected here so the lever-arm
                              compensation is NOT applied. Otherwise we
                              create a positive-feedback loop: the
                              compensated base position depends on the
                              fused yaw being estimated → COG inherits
                              that yaw error → ekf_map_node reinforces
                              it. The lever-arm is a constant offset
                              between antenna and base; for translation
                              it does not change the *direction* of
                              motion, only the position, so atan2 on
                              raw antenna ENU == atan2 on lever-arm-
                              corrected base ENU.)
          /wheel_odom        (sign of linear.x to disambiguate direction:
                              the antenna trajectory in ENU is the same
                              line whether we're going forward or reverse,
                              we use the wheel sign to know which end of
                              that line is the robot's nose. Reverse is
                              fully supported — yaw is just flipped 180°.)
  Output: /imu/cog_heading   (sensor_msgs/Imu with orientation set from
                              the heading, orientation_covariance[8] set
                              from the finite-difference uncertainty).

  Gating (all must hold for a publish):
   * RTK-Fixed flag set in AbsolutePose.flags
   * Derived ground speed ≥ min_speed (default 0.30 m/s)
   * Wheel-odom linear.x ≥ 0.10 m/s (robot is unambiguously going forward)
   * Position_accuracy on both samples ≤ 0.05 m (RTK σ bound)
   * Time between samples within [50 ms, 500 ms] (reject stale or duplicate)

  The yaw standard deviation we publish is
     σ_yaw ≈ atan2(2 × σ_pos, displacement)
  which for σ_pos = 7 mm and 0.06 m displacement (0.3 m/s × 0.2 s) is
  about 13° — tight enough to correct gyro drift, loose enough that a
  single noisy sample cannot snap the filter.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


# Flat-earth ENU projection (matches navsat_to_absolute_pose_node.cpp).
DEG_TO_RAD = math.pi / 180.0
METERS_PER_DEG = 111319.49079327357


class CogToImu(Node):
    def __init__(self) -> None:
        super().__init__("cog_to_imu")

        # OpenMower-style adaptive-covariance fusion: we NEVER reject a
        # sample for being slow. Instead σ_yaw grows with 1/displacement
        # so low-speed samples have near-zero weight in the EKF, matching
        # the natural signal-to-noise. The only gate is |wheel_vx| above
        # a deadband — below that the wheel sign is ambiguous (transition,
        # stiction) and atan2 might publish the wrong direction. Reverse
        # motion IS supported: when wheel_vx < 0 we flip the COG by π so
        # the published yaw always represents the robot's forward axis.
        self._min_abs_wheel = self.declare_parameter("min_abs_wheel_ms", 0.05).value
        self._max_pos_accuracy = self.declare_parameter("max_pos_accuracy_m", 0.05).value
        self._min_dt = self.declare_parameter("min_sample_dt_s", 0.05).value
        self._max_dt = self.declare_parameter("max_sample_dt_s", 0.50).value
        # Hard ceiling on published yaw variance — a 100% σ_yaw (~180°
        # equivalent) update adds no information but also does no harm.
        # Keep a cap so pathological samples cannot blow up the filter
        # numerics. Also a hard FLOOR of σ ≥ 0.5° (from GPS noise) to
        # avoid over-trusting any single fix pair.
        self._max_yaw_var = self.declare_parameter("max_yaw_variance", 3.0).value
        self._min_yaw_var = self.declare_parameter("min_yaw_variance", 7.6e-5).value

        # Datum for flat-earth ENU projection. Same convention as
        # navsat_to_absolute_pose_node — read from mowgli_robot.yaml and
        # injected by the launch file. If absent (0.0), we self-seed on
        # the first RTK-Fixed sample.
        self._datum_lat = float(self.declare_parameter("datum_lat", 0.0).value)
        self._datum_lon = float(self.declare_parameter("datum_lon", 0.0).value)
        self._datum_seeded = self._datum_lat != 0.0 or self._datum_lon != 0.0
        self._cos_datum_lat = math.cos(self._datum_lat * DEG_TO_RAD)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(NavSatFix, "/gps/fix", self._on_fix, qos)
        self.create_subscription(Odometry, "/wheel_odom", self._on_wheel, qos)
        self._pub = self.create_publisher(Imu, "/imu/cog_heading", qos)

        self._prev: tuple[float, float, float, float] | None = None  # (t, x, y, pos_acc)
        self._wheel_vx = 0.0
        self._published_fwd = 0
        self._published_rev = 0
        self._rejected_stationary = 0
        self._rejected_accuracy = 0
        self._rejected_fix = 0
        self._rejected_displacement = 0

        # Log stats periodically so we can see whether the node is
        # actually contributing to the filter.
        self.create_timer(30.0, self._log_stats)

        self.get_logger().info(
            "cog_to_imu started — publish /imu/cog_heading on every "
            "RTK-Fixed sample with |wheel_vx| > {:.2f} m/s (forward + "
            "reverse, adaptive covariance, no hard speed gate)".format(
                self._min_abs_wheel))

    def _on_wheel(self, msg: Odometry) -> None:
        self._wheel_vx = msg.twist.twist.linear.x

    def _on_fix(self, msg: NavSatFix) -> None:
        # RTK-Fixed only: NavSatStatus.STATUS_GBAS_FIX (=2) per the
        # convention used in navsat_to_absolute_pose_node.
        if msg.status.status < NavSatStatus.STATUS_GBAS_FIX:
            self._rejected_fix += 1
            return

        # position_accuracy ≈ sqrt(mean(var_lat, var_lon)) — same as
        # navsat_to_absolute_pose_node.
        var_lat = msg.position_covariance[0]
        var_lon = msg.position_covariance[4]
        if var_lat <= 0.0 or var_lon <= 0.0:
            pos_acc = 10.0  # unknown — treat as bad
        else:
            pos_acc = math.sqrt((var_lat + var_lon) * 0.5)
        if pos_acc > self._max_pos_accuracy:
            self._rejected_accuracy += 1
            return

        # Self-seed datum on first valid sample if launch did not provide one.
        if not self._datum_seeded:
            self._datum_lat = msg.latitude
            self._datum_lon = msg.longitude
            self._cos_datum_lat = math.cos(self._datum_lat * DEG_TO_RAD)
            self._datum_seeded = True
            self.get_logger().info(
                "datum self-seeded from first RTK fix: lat=%.8f lon=%.8f"
                % (self._datum_lat, self._datum_lon))

        # RAW antenna ENU — no lever-arm correction. The lever-arm offset
        # is a constant body-frame vector; during translation the antenna
        # and base move in parallel so atan2(dy, dx) is identical for
        # both, and we avoid the feedback loop that the compensated
        # /gps/absolute_pose introduces.
        x = (msg.longitude - self._datum_lon) * self._cos_datum_lat * METERS_PER_DEG
        y = (msg.latitude - self._datum_lat) * METERS_PER_DEG
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos_acc = max(pos_acc, 0.002)  # guard for pathological 0

        if self._prev is None:
            self._prev = (t, x, y, pos_acc)
            return

        t0, x0, y0, pa0 = self._prev
        dt = t - t0
        if dt < self._min_dt or dt > self._max_dt:
            # too close or stale — skip but keep old baseline so fast
            # sample rate does not starve us of valid pairs
            if dt > self._max_dt:
                self._prev = (t, x, y, pos_acc)
            return

        dx = x - x0
        dy = y - y0
        displacement = math.hypot(dx, dy)
        self._prev = (t, x, y, pos_acc)

        # Skip pathologically short displacements — atan2(0, 0) returns
        # 0 which would be a fake yaw. This is a noise-floor rejection,
        # NOT a speed gate: at 3 mm RTK noise, a displacement of 5 mm
        # has σ_yaw ≈ 60° which is fine, but 0 mm is useless.
        if displacement < 0.005:
            self._rejected_displacement += 1
            return

        if abs(self._wheel_vx) < self._min_abs_wheel:
            # below the deadband — wheels are stationary or in a sign-
            # ambiguous transition (stiction, direction change). GPS
            # drift would produce a fake COG. Skip.
            self._rejected_stationary += 1
            return

        # Direction-aware COG. Antenna trajectory in ENU points along the
        # robot's velocity vector. In forward motion that's the heading;
        # in reverse it's heading + π. We flip dx/dy when reversing so
        # the published yaw is always the robot's *forward axis*, which
        # is what robot_localization expects from an orientation
        # observation.
        if self._wheel_vx >= 0:
            yaw = math.atan2(dy, dx)
            self._published_fwd += 1
        else:
            yaw = math.atan2(-dy, -dx)
            self._published_rev += 1

        # σ_yaw ≈ atan2(2σ_pos, displacement). Factor 2 accounts for
        # independent noise on both endpoints. At displacement → 0 this
        # approaches π/2 (no info), which matches OpenMower's vector
        # fusion intuition: low-speed samples contribute almost nothing.
        sigma_pos = math.hypot(pos_acc, pa0)
        sigma_yaw = math.atan2(2.0 * sigma_pos, max(displacement, 1e-3))
        yaw_var = max(
            self._min_yaw_var,
            min(sigma_yaw * sigma_yaw, self._max_yaw_var),
        )

        self._publish_imu(self.get_clock().now().to_msg(), yaw, yaw_var)

    def _publish_imu(self, stamp, yaw: float, yaw_var: float) -> None:
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_footprint"
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        imu.orientation = q
        cov = [0.0] * 9
        cov[8] = yaw_var  # yaw variance; robot_localization reads index 8
        imu.orientation_covariance = cov
        # Leave angular_velocity and linear_acceleration zero with flag
        # -1 covariance [0] to tell downstream it is unused.
        imu.angular_velocity_covariance = [-1.0] + [0.0] * 8
        imu.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        self._pub.publish(imu)

    def _log_stats(self) -> None:
        self.get_logger().info(
            "cog_to_imu stats: published fwd={} rev={}, rejected fix={} "
            "accuracy={} stationary={} displacement={}".format(
                self._published_fwd,
                self._published_rev,
                self._rejected_fix,
                self._rejected_accuracy,
                self._rejected_stationary,
                self._rejected_displacement))
        self._published_fwd = 0
        self._published_rev = 0
        self._rejected_fix = 0
        self._rejected_accuracy = 0
        self._rejected_stationary = 0
        self._rejected_displacement = 0


def main() -> None:
    rclpy.init()
    node = CogToImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
