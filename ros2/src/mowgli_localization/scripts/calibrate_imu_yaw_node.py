#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
calibrate_imu_yaw_node.py

Estimates the IMU chip's mounting yaw relative to base_link.

The WT901 is physically mounted rotated around Z by `imu_yaw` relative to
base_link. Pure rotation does not reveal this angle (gyro_z is invariant
to yaw mount rotation around the common Z axis). Pure linear acceleration
in base_link +X DOES reveal it: accel observed in the chip frame is
    (a * cos(imu_yaw), -a * sin(imu_yaw), g)
so for a non-zero body acceleration a,
    imu_yaw = atan2(-ay_chip, ax_chip)

Usage pattern:
  1. Call ~/calibrate service (mowgli_interfaces/srv/CalibrateImuYaw).
  2. During `duration_sec` seconds drive the robot forward in a straight
     line then stop. Any acceleration or deceleration along the body X
     axis (with |wz| small) contributes a valid sample.
  3. On return, imu_yaw_rad is the computed mounting angle. The caller
     is responsible for persisting it (URDF xacro arg).
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from mowgli_interfaces.srv import CalibrateImuYaw


def _stamp_to_float(stamp) -> float:
    """Convert a ROS Time message into a float seconds value."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ImuYawCalibrator(Node):
    """Collects IMU + wheel odometry samples and computes imu_yaw."""

    DEFAULT_DURATION_SEC = 30.0
    WZ_STRAIGHT_THRESHOLD = 0.05        # rad/s — |wz| below this ⇒ straight motion
    ACCEL_BODY_THRESHOLD = 0.1          # m/s² — |a_body| above this ⇒ useful sample
    MIN_SAMPLES = 50

    def __init__(self) -> None:
        super().__init__("calibrate_imu_yaw_node")

        self._cb_group = ReentrantCallbackGroup()

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # Buffers — each list holds (t, ...) tuples. Appended only while
        # `_collecting` is True; processed after the collection window closes.
        self._lock = threading.Lock()
        self._collecting = False
        self._imu_samples: list[tuple[float, float, float]] = []   # (t, ax, ay)
        self._odom_samples: list[tuple[float, float, float]] = []  # (t, vx, wz)

        self._imu_sub = self.create_subscription(
            Imu, "/imu/data", self._imu_cb, imu_qos, callback_group=self._cb_group
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/wheel_odom", self._odom_cb, odom_qos, callback_group=self._cb_group
        )

        self._srv = self.create_service(
            CalibrateImuYaw,
            "~/calibrate",
            self._calibrate_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "IMU yaw calibration node ready. Call ~/calibrate to begin "
            "(drive the robot forward ~2 m then stop during the window)."
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _imu_cb(self, msg: Imu) -> None:
        with self._lock:
            if not self._collecting:
                return
            t = _stamp_to_float(msg.header.stamp)
            self._imu_samples.append(
                (t, float(msg.linear_acceleration.x), float(msg.linear_acceleration.y))
            )

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            if not self._collecting:
                return
            t = _stamp_to_float(msg.header.stamp)
            self._odom_samples.append(
                (t, float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))
            )

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------

    def _calibrate_cb(
        self,
        request: CalibrateImuYaw.Request,
        response: CalibrateImuYaw.Response,
    ) -> CalibrateImuYaw.Response:
        duration = float(request.duration_sec)
        if duration <= 0.0:
            duration = self.DEFAULT_DURATION_SEC

        with self._lock:
            self._imu_samples.clear()
            self._odom_samples.clear()
            self._collecting = True

        self.get_logger().info(
            f"Calibration started — collecting samples for {duration:.1f}s. "
            "Drive the robot forward ~2 m in a straight line then stop."
        )

        # Sleep without holding GIL too tightly — the ReentrantCallbackGroup +
        # MultiThreadedExecutor guarantees that the subscription callbacks can
        # still fire while this handler is blocked.
        time.sleep(duration)

        with self._lock:
            self._collecting = False
            imu_samples = list(self._imu_samples)
            odom_samples = list(self._odom_samples)

        self.get_logger().info(
            f"Collection window closed — {len(imu_samples)} IMU / "
            f"{len(odom_samples)} odom samples received."
        )

        result = self._compute_imu_yaw(imu_samples, odom_samples)
        response.success = result["success"]
        response.message = result["message"]
        response.imu_yaw_rad = result["imu_yaw_rad"]
        response.imu_yaw_deg = result["imu_yaw_deg"]
        response.samples_used = int(result["samples_used"])
        response.std_dev_deg = result["std_dev_deg"]

        if response.success:
            self.get_logger().info(
                f"imu_yaw = {response.imu_yaw_rad:+.4f} rad "
                f"({response.imu_yaw_deg:+.2f}°) from {response.samples_used} "
                f"samples, stddev {response.std_dev_deg:.2f}°"
            )
        else:
            self.get_logger().warn(f"Calibration failed: {response.message}")

        return response

    # ------------------------------------------------------------------
    # Numerical core — pure function over collected samples
    # ------------------------------------------------------------------

    def _compute_imu_yaw(
        self,
        imu_samples: list[tuple[float, float, float]],
        odom_samples: list[tuple[float, float, float]],
    ) -> dict:
        empty = {
            "success": False,
            "message": "",
            "imu_yaw_rad": 0.0,
            "imu_yaw_deg": 0.0,
            "samples_used": 0,
            "std_dev_deg": 0.0,
        }

        if len(odom_samples) < 3:
            empty["message"] = (
                f"Not enough wheel_odom samples ({len(odom_samples)}). "
                "Is /wheel_odom being published?"
            )
            return empty
        if len(imu_samples) < self.MIN_SAMPLES:
            empty["message"] = (
                f"Not enough /imu/data samples ({len(imu_samples)}). "
                "Is the IMU running?"
            )
            return empty

        odom = np.asarray(odom_samples, dtype=np.float64)  # shape (N, 3)
        imu = np.asarray(imu_samples, dtype=np.float64)    # shape (M, 3)

        odom_t = odom[:, 0]
        odom_vx = odom[:, 1]
        odom_wz = odom[:, 2]

        # Sort by time (subscribers can deliver out of order in edge cases)
        order_o = np.argsort(odom_t)
        odom_t = odom_t[order_o]
        odom_vx = odom_vx[order_o]
        odom_wz = odom_wz[order_o]

        order_i = np.argsort(imu[:, 0])
        imu = imu[order_i]

        # Body-frame acceleration from d(vx)/dt via central difference at
        # the odometry timestamps.
        if len(odom_t) < 3:
            empty["message"] = "Not enough odom samples for central difference."
            return empty
        a_body = np.zeros_like(odom_vx)
        a_body[1:-1] = (odom_vx[2:] - odom_vx[:-2]) / np.maximum(
            odom_t[2:] - odom_t[:-2], 1e-6
        )
        a_body[0] = a_body[1]
        a_body[-1] = a_body[-2]

        # For each IMU sample find nearest odom timestamp and interpolate
        # both vx and wz -> the corresponding a_body and wz.
        imu_t = imu[:, 0]
        idx = np.searchsorted(odom_t, imu_t)
        idx = np.clip(idx, 1, len(odom_t) - 1)
        left = idx - 1
        right = idx
        dt = np.maximum(odom_t[right] - odom_t[left], 1e-9)
        frac = (imu_t - odom_t[left]) / dt
        frac = np.clip(frac, 0.0, 1.0)
        wz_interp = odom_wz[left] + (odom_wz[right] - odom_wz[left]) * frac
        a_interp = a_body[left] + (a_body[right] - a_body[left]) * frac

        # Filter: straight motion AND meaningful acceleration.
        straight = np.abs(wz_interp) < self.WZ_STRAIGHT_THRESHOLD
        moving = np.abs(a_interp) > self.ACCEL_BODY_THRESHOLD
        mask = straight & moving

        valid_count = int(np.count_nonzero(mask))
        if valid_count < self.MIN_SAMPLES:
            empty["samples_used"] = valid_count
            empty["message"] = (
                f"Only {valid_count} valid samples (need ≥ {self.MIN_SAMPLES}). "
                "Drive the robot forward ~2 m in a straight line with a "
                "clear acceleration and then a clear deceleration."
            )
            return empty

        ax_valid = imu[mask, 1]
        ay_valid = imu[mask, 2]
        a_sign = np.sign(a_interp[mask])

        # For each sample, rotate the sensed (ax, ay) by sign(a_body). When
        # the robot is decelerating (a_body < 0) the horizontal accel vector
        # flips 180°; pre-multiplying by sign(a_body) aligns every sample
        # with the same body +X direction before averaging.
        ax_s = ax_valid * a_sign
        ay_s = ay_valid * a_sign

        # Per-sample imu_yaw = atan2(-ay_s, ax_s). Average by projecting to
        # the unit circle so wrap-around is handled correctly.
        per_sample = np.arctan2(-ay_s, ax_s)
        mean_cos = float(np.mean(np.cos(per_sample)))
        mean_sin = float(np.mean(np.sin(per_sample)))
        imu_yaw_rad = math.atan2(mean_sin, mean_cos)

        # Circular standard deviation — Fisher's definition.
        r_bar = math.hypot(mean_cos, mean_sin)
        r_bar = min(max(r_bar, 1e-9), 1.0)
        std_rad = math.sqrt(-2.0 * math.log(r_bar))

        return {
            "success": True,
            "message": f"Calibrated from {valid_count} valid samples.",
            "imu_yaw_rad": imu_yaw_rad,
            "imu_yaw_deg": math.degrees(imu_yaw_rad),
            "samples_used": valid_count,
            "std_dev_deg": math.degrees(std_rad),
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuYawCalibrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
