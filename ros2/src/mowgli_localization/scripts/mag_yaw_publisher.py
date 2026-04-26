#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
#
# SPDX-License-Identifier: GPL-3.0
"""
mag_yaw_publisher.py

Reads /imu/mag_raw + /imu/data, applies the hard-iron / soft-iron
calibration written by calibrate_imu_yaw_node during the rotation
phase, tilt-compensates the horizontal field with roll and pitch from
the accelerometer, then publishes an absolute yaw as a sensor_msgs/Imu
on /imu/mag_yaw.

Pipeline per sample:
 1. b_cal = (b_raw - offset) * scale                    [calibration]
 2. roll = atan2(ay, az), pitch = atan2(-ax, √(ay²+az²)) [tilt from accel]
 3. Rotate b_cal from imu_link to base_footprint via the URDF
    base_footprint → imu_link rotation looked up from TF.
 4. bx', by' = tilt-compensated horizontal components in base_footprint
 5. yaw_magnetic_ENU = atan2(bx', by')
       (mag vector in body pointing to magnetic north → atan2 gives
        body-forward heading measured CCW from East; see derivation
        in the source comment block.)
 6. yaw_true = yaw_magnetic_ENU + declination_rad          [optional]
 7. publish Imu with quaternion(0, 0, yaw_true)

Downstream consumer: ekf_map_node imu1_config fuses index 5
(absolute yaw) only; the rest of the IMU fields are left with -1
covariance flags so robot_localization ignores them.

The node silently skips publishing until a calibration file is loaded —
there is no point pushing raw (uncalibrated) magnetometer data into the
filter. Log a single warning at boot if the file is missing so the
operator knows they need to run /calibrate_imu_yaw_node/calibrate.
"""

from __future__ import annotations

import math
import os

import rclpy
import yaml
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, MagneticField
from tf2_ros import Buffer, TransformListener


class MagYawPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mag_yaw_publisher")

        # Parameters
        self._cal_path = self.declare_parameter(
            "calibration_path", "/ros2_ws/maps/mag_calibration.yaml"
        ).value
        # Magnetic declination in degrees, added to yaw to report w.r.t.
        # true north instead of magnetic north. Paris 2026 ≈ +1.5° E.
        self._declination_rad = math.radians(
            float(self.declare_parameter("declination_deg", 1.5).value)
        )
        # Minimum horizontal field magnitude to publish. Below this the
        # tilt-compensated heading is dominated by noise.
        self._min_horizontal_uT = float(
            self.declare_parameter("min_horizontal_uT", 5.0).value
        )
        # Yaw covariance — tuned to the typical post-cal ±3° scatter on
        # the LIS3MDL once hard/soft iron are compensated. (rad²)
        self._yaw_var = float(self.declare_parameter("yaw_variance", 2.7e-3).value)

        self._cal = self._load_calibration(self._cal_path)

        self._qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._latest_imu: Imu | None = None
        # High-rate subs are created only when a calibration is loaded, so
        # the node sits at ~0% CPU when no calibration exists. Without this,
        # subscribing to /imu/data + /imu/mag_raw at 91 Hz × 2 just to
        # bail out at the top of every callback burned ~35% of one core.
        self._imu_sub = None
        self._mag_sub = None
        self._pub = self.create_publisher(Imu, "/imu/mag_yaw", self._qos_sensor)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Periodic stats log so we can see drift / glitches.
        self._published = 0
        self._rejected_noise = 0
        self._rejected_no_cal = 0
        self._rejected_no_imu = 0
        self._rejected_no_tf = 0
        self.create_timer(30.0, self._log_stats)
        # Watch the calibration file so we pick it up automatically once
        # the user runs the magnetometer calibration via the GUI.
        self._reload_timer = self.create_timer(30.0, self._reload_cal_if_needed)

        if self._cal is None:
            self.get_logger().warn(
                "No mag calibration found at {} — mag_yaw_publisher idle, "
                "polling every 30 s for the calibration file.".format(
                    self._cal_path)
            )
        else:
            self._activate_subs()
            self.get_logger().info(
                "Mag calibration loaded: offset=({:+.2f}, {:+.2f}, {:+.2f}) µT, "
                "scale=({:.3f}, {:.3f}, {:.3f}), |B|cal={:.2f} µT".format(
                    self._cal["offset_x_uT"], self._cal["offset_y_uT"],
                    self._cal["offset_z_uT"],
                    self._cal["scale_x"], self._cal["scale_y"], self._cal["scale_z"],
                    self._cal["magnitude_mean_uT"])
            )

    def _activate_subs(self) -> None:
        if self._imu_sub is not None:
            return
        self._imu_sub = self.create_subscription(
            Imu, "/imu/data", self._on_imu, self._qos_sensor
        )
        self._mag_sub = self.create_subscription(
            MagneticField, "/imu/mag_raw", self._on_mag, self._qos_sensor
        )

    def _reload_cal_if_needed(self) -> None:
        if self._cal is not None:
            return
        cal = self._load_calibration(self._cal_path)
        if cal is None:
            return
        self._cal = cal
        self._activate_subs()
        self.get_logger().info(
            "Mag calibration appeared — subscriptions activated.")

    def _load_calibration(self, path: str) -> dict | None:
        if not os.path.exists(path):
            return None
        try:
            with open(path) as fh:
                data = yaml.safe_load(fh)
            cal = data.get("mag_calibration")
            if not cal:
                self.get_logger().error(
                    f"{path}: missing 'mag_calibration' key")
                return None
            return cal
        except Exception as exc:
            self.get_logger().error(f"Failed to load {path}: {exc}")
            return None

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg

    def _on_mag(self, msg: MagneticField) -> None:
        if self._cal is None:
            self._rejected_no_cal += 1
            return
        if self._latest_imu is None:
            self._rejected_no_imu += 1
            return

        # 1) calibration — convert to µT first then apply offset/scale
        bx_uT = msg.magnetic_field.x * 1e6
        by_uT = msg.magnetic_field.y * 1e6
        bz_uT = msg.magnetic_field.z * 1e6

        bx = (bx_uT - self._cal["offset_x_uT"]) * self._cal["scale_x"]
        by = (by_uT - self._cal["offset_y_uT"]) * self._cal["scale_y"]
        bz = (bz_uT - self._cal["offset_z_uT"]) * self._cal["scale_z"]

        # 2) tilt from the accelerometer currently at rest-ish. If the
        # robot is accelerating hard this biases roll/pitch, but during
        # normal mowing cruise this is well below the noise of the mag
        # itself.
        ax = self._latest_imu.linear_acceleration.x
        ay = self._latest_imu.linear_acceleration.y
        az = self._latest_imu.linear_acceleration.z
        if abs(az) < 1e-6 and abs(ax) < 1e-6 and abs(ay) < 1e-6:
            self._rejected_no_imu += 1
            return
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # 3) Rotate the calibrated mag vector from imu_link to base_footprint
        # so the yaw we extract is in the robot's frame, not the sensor's.
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_footprint", "imu_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception:
            self._rejected_no_tf += 1
            return
        bx_b, by_b, bz_b = self._rotate_vector_by_quaternion(
            bx, by, bz, tf.transform.rotation
        )

        # Also rotate accel's implied roll/pitch into base — for a
        # robot that is level (our 2D case) this is identity but done
        # properly for tilt robustness.
        # We ignore this second rotation in the horizontal-projection
        # formula below — the body frame is assumed to be base_footprint
        # after the mag rotation, and roll/pitch are measured about
        # base_footprint's X/Y already if the IMU is mounted level.
        _ = (roll, pitch)

        # 4) Tilt-compensated horizontal components — from Pololu's
        # MinIMU-9 AHRS reference (Compass.ino) which uses exactly the
        # LIS3MDL on this board. The three-term expansion is necessary
        # whenever the sensor tilts: our first cut missed the roll-pitch
        # cross terms on bx_h which led to a few degrees of residual
        # error on a non-level robot.
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        bx_h = bx_b * cp + by_b * sr * sp + bz_b * cr * sp
        by_h = by_b * cr - bz_b * sr

        horiz = math.sqrt(bx_h * bx_h + by_h * by_h)
        if horiz < self._min_horizontal_uT:
            self._rejected_noise += 1
            return

        # 5) ENU yaw (CCW from East, +X forward of the robot):
        #   B_body for a body at ENU yaw ψ with B_world = (0, Bh, -Bd):
        #     bx_body = Bh sin ψ,  by_body = Bh cos ψ
        #   → ψ = atan2(bx_body, by_body)
        # The Pololu reference publishes heading_NED = atan2(-by, bx);
        # our ROS IMU frame is ENU (X forward, Y left, Z up), so the
        # equivalent ENU formula is atan2(bx, by).
        yaw_mag = math.atan2(bx_h, by_h)

        # 6) Declination: magnetic → true-north ENU.
        yaw_true = yaw_mag + self._declination_rad

        # 7) Publish as Imu with orientation only.
        self._publish_imu(msg.header.stamp, yaw_true)
        self._published += 1

    @staticmethod
    def _rotate_vector_by_quaternion(vx: float, vy: float, vz: float,
                                      q) -> tuple[float, float, float]:
        """Rotate vector (vx,vy,vz) by quaternion q (geometry_msgs/Quaternion).
        Standard v' = q * v * q^-1, expanded into scalar form."""
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        # compute t = 2 * cross(q.xyz, v)
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        # v' = v + q.w * t + cross(q.xyz, t)
        xp = vx + qw * tx + (qy * tz - qz * ty)
        yp = vy + qw * ty + (qz * tx - qx * tz)
        zp = vz + qw * tz + (qx * ty - qy * tx)
        return xp, yp, zp

    def _publish_imu(self, stamp, yaw: float) -> None:
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_footprint"
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        imu.orientation = q
        cov = [0.0] * 9
        cov[8] = self._yaw_var
        imu.orientation_covariance = cov
        # Disable the gyro / accel channels so robot_localization ignores
        # them (this node is an orientation source only).
        imu.angular_velocity_covariance = [-1.0] + [0.0] * 8
        imu.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        self._pub.publish(imu)

    def _log_stats(self) -> None:
        self.get_logger().info(
            "mag_yaw_publisher stats: published={}, rejected no_cal={} "
            "no_imu={} no_tf={} noise={}".format(
                self._published, self._rejected_no_cal, self._rejected_no_imu,
                self._rejected_no_tf, self._rejected_noise))
        self._published = 0
        self._rejected_no_cal = 0
        self._rejected_no_imu = 0
        self._rejected_no_tf = 0
        self._rejected_noise = 0


def main() -> None:
    rclpy.init()
    node = MagYawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
