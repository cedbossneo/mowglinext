#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# SPDX-License-Identifier: GPL-3.0
"""
dock_yaw_to_set_pose.py

Bridges the dock-heading publication on /gnss/heading (sensor_msgs/Imu,
emitted by hardware_bridge_node every 1 s while is_charging is true) into a
one-shot set_pose on /ekf_map_node/set_pose for the robot_localization
backend.

Why:
  The global EKF (ekf_map_node) has no absolute heading reference at boot —
  the IMU has no magnetometer and navsat_transform only feeds position, not
  yaw. Without this bridge the filter yaw stays random until the robot has
  driven far enough for GPS position innovations (or /imu/cog_heading) to
  correct it indirectly.

Semantics:
  Fires once per docking event (rising edge of is_charging, plus one firing
  shortly after node start if the robot is already docked at boot). The
  emission does NOT repeat at 1 Hz — continuous set_pose while charging
  would keep snapping the filter back to the dock during the undock reverse
  motion. One shot is enough to give ekf_map a plausible yaw seed that
  CalibrateHeadingFromUndock then refines using the BackUp displacement.
"""

import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from mowgli_interfaces.msg import AbsolutePose, Status as HwStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu

DOCK_CALIBRATION_PATH = "/ros2_ws/maps/dock_calibration.yaml"


class DockYawToSetPose(Node):
    def __init__(self):
        super().__init__("dock_yaw_to_set_pose")

        # Topic config.
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._sub_status = self.create_subscription(
            HwStatus, "/hardware_bridge/status", self._on_status, qos_reliable
        )
        self._sub_heading = self.create_subscription(
            Imu, "/gnss/heading", self._on_heading, qos_reliable
        )
        self._sub_gps = self.create_subscription(
            AbsolutePose, "/gps/absolute_pose", self._on_gps, qos_sensor
        )
        self._pub_map = self.create_publisher(
            PoseWithCovarianceStamped, "/ekf_map_node/set_pose", qos_reliable
        )
        # ekf_odom's set_pose default topic is /set_pose (we only remapped
        # ekf_map). Seeding ekf_odom with the same yaw keeps odom→base yaw
        # aligned with map→base yaw from the start of the session — without
        # this, the lever-arm correction in navsat_to_absolute_pose_node
        # uses odom yaw ≈ 0 while map yaw = dock_yaw, producing a ~0.55 m
        # position error that the EKF then happily integrates as the GPS
        # pose measurement.
        self._pub_odom = self.create_publisher(
            PoseWithCovarianceStamped, "/set_pose", qos_reliable
        )

        # State.
        self._last_is_charging = None  # tri-state: None=unknown, True/False
        self._latest_heading: Imu | None = None
        self._latest_gps: AbsolutePose | None = None
        self._need_to_publish = False
        # Yaw variance for the seed (rad^2). 0.1 rad^2 ≈ σ 18° — loose
        # enough that the EKF still trusts a later, tighter refinement from
        # CalibrateHeadingFromUndock but tight enough to anchor the filter.
        self._yaw_var = self.declare_parameter("seed_yaw_variance", 0.1).value

        # Load /ros2_ws/maps/dock_calibration.yaml if it exists. The file
        # is written by calibrate_imu_yaw_node's dock-yaw pre-phase and
        # contains a ±1° yaw derived from the GPS track during a slow
        # 2 m undock. We prefer that over the phone-compass-based yaw
        # broadcast on /gnss/heading (±10° accuracy).
        self._file_yaw_rad: float | None = None
        self._file_yaw_var: float | None = None
        self._load_dock_calibration()

        self.get_logger().info(
            "dock_yaw_to_set_pose started — waits for rising edge of is_charging"
        )

    def _load_dock_calibration(self) -> None:
        if not os.path.exists(DOCK_CALIBRATION_PATH):
            self.get_logger().info(
                f"No {DOCK_CALIBRATION_PATH} — will fall back to "
                "/gnss/heading (phone-compass dock yaw)."
            )
            return
        try:
            with open(DOCK_CALIBRATION_PATH) as fh:
                data = yaml.safe_load(fh) or {}
            cal = data.get("dock_calibration") or {}
            yaw_rad = float(cal.get("dock_pose_yaw_rad"))
            sigma_rad = float(cal.get("yaw_sigma_rad", 0.035))  # 2° default
        except Exception as exc:
            self.get_logger().error(
                f"Failed to parse {DOCK_CALIBRATION_PATH}: {exc}. "
                "Falling back to /gnss/heading."
            )
            return
        self._file_yaw_rad = yaw_rad
        # Use max(file σ², param floor) so a bad calibration cannot over-trust.
        self._file_yaw_var = max(sigma_rad * sigma_rad, 1e-3)  # ~2° floor
        self.get_logger().info(
            "Loaded dock calibration: yaw={:.2f}° (σ={:.2f}°) from {}".format(
                math.degrees(yaw_rad), math.degrees(sigma_rad),
                DOCK_CALIBRATION_PATH,
            )
        )

    def _on_heading(self, msg: Imu) -> None:
        self._latest_heading = msg
        if self._need_to_publish:
            self._try_publish()

    def _on_gps(self, msg: AbsolutePose) -> None:
        self._latest_gps = msg
        if self._need_to_publish:
            self._try_publish()

    def _on_status(self, msg: HwStatus) -> None:
        is_charging = bool(msg.is_charging)

        if self._last_is_charging is None:
            # First status message — if we boot docked, inject once.
            if is_charging:
                self._need_to_publish = True
                self.get_logger().info(
                    "boot detected docked state → will inject dock yaw on "
                    "next heading+gps pair"
                )
        elif is_charging and not self._last_is_charging:
            # Rising edge of charging.
            self._need_to_publish = True
            self.get_logger().info(
                "charging rising edge → will inject dock yaw on next "
                "heading+gps pair"
            )

        self._last_is_charging = is_charging

        if self._need_to_publish and is_charging:
            self._try_publish()

    def _try_publish(self) -> None:
        # We need GPS for position. Heading comes from either the
        # dock_calibration.yaml file (preferred) or /gnss/heading (fallback).
        if self._latest_gps is None:
            return
        if self._file_yaw_rad is None and self._latest_heading is None:
            return

        # Resolve yaw quaternion + variance: file value takes precedence.
        if self._file_yaw_rad is not None:
            q = Quaternion()
            q.w = math.cos(self._file_yaw_rad / 2.0)
            q.z = math.sin(self._file_yaw_rad / 2.0)
            yaw_quat = q
            yaw_var = self._file_yaw_var
        else:
            yaw_quat = self._latest_heading.orientation
            yaw_var = self._yaw_var

        # Common covariance: tight on x/y/yaw, loose on z/roll/pitch so the
        # filter keeps its prior on the states we are not setting.
        cov = [0.0] * 36
        cov[0] = 0.01                    # x
        cov[7] = 0.01                    # y
        cov[14] = 1e6                    # z
        cov[21] = 1e6                    # roll
        cov[28] = 1e6                    # pitch
        cov[35] = yaw_var                # yaw

        # ---- ekf_map seed: GPS position + dock yaw in the map frame ----
        map_seed = PoseWithCovarianceStamped()
        map_seed.header.stamp = self.get_clock().now().to_msg()
        map_seed.header.frame_id = "map"
        map_seed.pose.pose.position.x = self._latest_gps.pose.pose.position.x
        map_seed.pose.pose.position.y = self._latest_gps.pose.pose.position.y
        map_seed.pose.pose.orientation = yaw_quat
        map_seed.pose.covariance = list(cov)
        self._pub_map.publish(map_seed)

        # ---- ekf_odom seed: origin + dock yaw in the odom frame --------
        # Resetting ekf_odom to (0, 0, dock_yaw) redefines the odom origin
        # at the robot's current physical position with the true heading.
        # The robot is stationary while charging, so the position jump has
        # no physical effect — map→odom recomputes to keep map→base stable
        # — but odom→base yaw now matches the actual robot heading. Without
        # this, navsat_to_absolute_pose rotates the lever arm by odom yaw
        # ≈ 0 instead of the real -144° and /gps/pose_cov drifts 55 cm
        # away from truth, breaking coverage path following.
        odom_seed = PoseWithCovarianceStamped()
        odom_seed.header.stamp = map_seed.header.stamp
        odom_seed.header.frame_id = "odom"
        odom_seed.pose.pose.position.x = 0.0
        odom_seed.pose.pose.position.y = 0.0
        odom_seed.pose.pose.orientation = yaw_quat
        odom_seed.pose.covariance = list(cov)
        self._pub_odom.publish(odom_seed)

        self._need_to_publish = False

        # Extract yaw from the quaternion we just published for logging.
        yaw = math.atan2(2.0 * yaw_quat.w * yaw_quat.z,
                         1.0 - 2.0 * yaw_quat.z * yaw_quat.z)
        source = "file" if self._file_yaw_rad is not None else "/gnss/heading"
        self.get_logger().info(
            "published dock set_pose ({}): map=({:.3f}, {:.3f}) yaw={:.1f}°, "
            "odom=(0, 0) yaw={:.1f}°".format(
                source,
                map_seed.pose.pose.position.x,
                map_seed.pose.pose.position.y,
                math.degrees(yaw),
                math.degrees(yaw),
            )
        )


def main():
    rclpy.init()
    node = DockYawToSetPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
