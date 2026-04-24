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
  to start targeting strips slightly off-axis. FusionCore had a native
  velocity_heading mode for this; robot_localization does not.

Design:
  Input : /gps/absolute_pose (already projected to map-frame ENU, carries
                              RTK fix flags and position_accuracy).
          /wheel_odom        (sign of linear.x to distinguish forward
                              motion from reverse — COG flips 180° in
                              reverse, which would corrupt the yaw seed).
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
from mowgli_interfaces.msg import AbsolutePose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu


FLAG_GPS_RTK_FIXED = AbsolutePose.FLAG_GPS_RTK_FIXED


class CogToImu(Node):
    def __init__(self) -> None:
        super().__init__("cog_to_imu")

        self._min_speed = self.declare_parameter("min_speed_ms", 0.30).value
        self._min_fwd_wheel = self.declare_parameter("min_forward_wheel_ms", 0.10).value
        self._max_pos_accuracy = self.declare_parameter("max_pos_accuracy_m", 0.05).value
        self._min_dt = self.declare_parameter("min_sample_dt_s", 0.05).value
        self._max_dt = self.declare_parameter("max_sample_dt_s", 0.50).value
        # Fallback ceiling on yaw covariance. Even with tight σ_pos, very
        # small displacements inflate the analytic σ_yaw — cap so the EKF
        # still gets a usable correction at low speed.
        self._max_yaw_var = self.declare_parameter("max_yaw_variance", 0.05).value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(AbsolutePose, "/gps/absolute_pose",
                                 self._on_pose, qos)
        self.create_subscription(Odometry, "/wheel_odom", self._on_wheel, qos)
        self._pub = self.create_publisher(Imu, "/imu/cog_heading", qos)

        self._prev: tuple[float, float, float, float] | None = None  # (t, x, y, pos_acc)
        self._wheel_vx = 0.0
        self._published = 0
        self._rejected_speed = 0
        self._rejected_reverse = 0
        self._rejected_accuracy = 0
        self._rejected_fix = 0

        # Log stats periodically so we can see whether the node is
        # actually contributing to the filter.
        self.create_timer(30.0, self._log_stats)

        self.get_logger().info(
            "cog_to_imu started — publish /imu/cog_heading when "
            "RTK-Fixed & speed > {:.2f} m/s & wheel fwd > {:.2f} m/s".format(
                self._min_speed, self._min_fwd_wheel))

    def _on_wheel(self, msg: Odometry) -> None:
        self._wheel_vx = msg.twist.twist.linear.x

    def _on_pose(self, msg: AbsolutePose) -> None:
        if not (msg.flags & FLAG_GPS_RTK_FIXED):
            self._rejected_fix += 1
            return
        if msg.position_accuracy > self._max_pos_accuracy:
            self._rejected_accuracy += 1
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        pos_acc = max(msg.position_accuracy, 0.002)  # guard for pathological 0

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
        speed = displacement / dt
        self._prev = (t, x, y, pos_acc)

        if speed < self._min_speed:
            self._rejected_speed += 1
            return

        if self._wheel_vx < self._min_fwd_wheel:
            # wheels say we are not moving forward — either standing still
            # despite GPS drift, or reversing. Skip so COG doesn't flip.
            self._rejected_reverse += 1
            return

        yaw = math.atan2(dy, dx)
        # σ_yaw ≈ atan2(2σ_pos, displacement). The factor 2 accounts for
        # independent noise on both endpoints.
        sigma_pos = math.hypot(pos_acc, pa0)
        sigma_yaw = math.atan2(2.0 * sigma_pos, max(displacement, 1e-3))
        yaw_var = min(sigma_yaw * sigma_yaw, self._max_yaw_var)

        self._publish_imu(msg.header.stamp, yaw, yaw_var)
        self._published += 1

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
            "cog_to_imu stats: published={}, rejected fix={} accuracy={} "
            "speed={} reverse={}".format(
                self._published,
                self._rejected_fix,
                self._rejected_accuracy,
                self._rejected_speed,
                self._rejected_reverse))
        self._published = 0
        self._rejected_fix = 0
        self._rejected_accuracy = 0
        self._rejected_speed = 0
        self._rejected_reverse = 0


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
