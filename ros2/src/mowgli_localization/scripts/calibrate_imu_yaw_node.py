#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
calibrate_imu_yaw_node.py

Autonomously estimates the IMU chip's mounting rotation relative to base_link
(yaw from motion, pitch/roll from stationary gravity).

The WT901 is physically mounted rotated around Z by `imu_yaw` relative to
base_link. Pure rotation does not reveal this angle (gyro_z is invariant
to yaw mount rotation around the common Z axis). Pure linear acceleration
in base_link +X DOES reveal it: accel observed in the chip frame is
    (a * cos(imu_yaw), -a * sin(imu_yaw), g)
so for a non-zero body acceleration a,
    imu_yaw = atan2(-ay_chip, ax_chip)

Mounting pitch/roll are independently observable from the gravity vector
while the robot is perfectly still. With URDF rpy applied before imu_yaw,
the chip-frame angles extracted here go directly into mowgli_robot.yaml:
    imu_pitch = atan2(-ax_chip, az_chip)
    imu_roll  = atan2( ay_chip, az_chip)
(identical formula to hardware_bridge_node's per-dock tilt log, just
averaged over the baseline + inter-cycle pauses of this drive).

When the /calibrate service is invoked, this node DRIVES THE ROBOT itself:
forward with a ramp-cruise-ramp profile, pause, then back to start. Users
need only call the service with the robot undocked; no manual teleop.

Safety: the service refuses to run when the robot is charging (on dock)
or when an emergency is active. The collision_monitor stays in the
pipeline, so physical obstacles stop the motion regardless.
"""

import math
import os
import threading
import time
from datetime import datetime, timezone

import numpy as np
import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry

from mowgli_interfaces.msg import AbsolutePose, Emergency
from mowgli_interfaces.msg import HighLevelStatus
from mowgli_interfaces.msg import Status as HwStatus
from mowgli_interfaces.srv import CalibrateImuYaw, HighLevelControl

# HighLevelStatus state codes (mirror HighLevelStatus.msg)
HL_STATE_NULL           = 0
HL_STATE_IDLE           = 1
HL_STATE_AUTONOMOUS     = 2
HL_STATE_RECORDING      = 3
HL_STATE_MANUAL_MOWING  = 4

# HighLevelControl command codes (mirror HighLevelControl.srv)
HL_CMD_RECORD_AREA      = 3
HL_CMD_RECORD_CANCEL    = 6


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def fit_mag_min_max(samples: list[tuple[float, float, float]]) -> dict:
    """Min/max hard-iron + isotropic soft-iron fit. Returns calibration dict.

    This is the simplest robust mag calibration that works on a wheeled
    robot that can only spin around Z. A full ellipsoid fit would need
    tilts in multiple axes which we can't drive cleanly here."""
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    zs = [s[2] for s in samples]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)

    offset_x = (max_x + min_x) / 2.0
    offset_y = (max_y + min_y) / 2.0
    offset_z = (max_z + min_z) / 2.0

    range_x = max_x - min_x
    range_y = max_y - min_y
    range_z = max_z - min_z
    mean_range = (range_x + range_y + range_z) / 3.0

    scale_x = mean_range / range_x if range_x > 1e-6 else 1.0
    scale_y = mean_range / range_y if range_y > 1e-6 else 1.0
    scale_z = mean_range / range_z if range_z > 1e-6 else 1.0

    mags = []
    for bx, by, bz in samples:
        cx = (bx - offset_x) * scale_x
        cy = (by - offset_y) * scale_y
        cz = (bz - offset_z) * scale_z
        mags.append(math.sqrt(cx * cx + cy * cy + cz * cz))
    mag_mean = sum(mags) / len(mags) if mags else 0.0
    var = sum((m - mag_mean) ** 2 for m in mags) / len(mags) if mags else 0.0
    mag_std = math.sqrt(var)

    return {
        "offset_x_uT": float(offset_x),
        "offset_y_uT": float(offset_y),
        "offset_z_uT": float(offset_z),
        "scale_x": float(scale_x),
        "scale_y": float(scale_y),
        "scale_z": float(scale_z),
        "sample_count": len(samples),
        "magnitude_mean_uT": float(mag_mean),
        "magnitude_std_uT": float(mag_std),
        "raw_min_uT": [float(min_x), float(min_y), float(min_z)],
        "raw_max_uT": [float(max_x), float(max_y), float(max_z)],
    }


class ImuYawCalibrator(Node):
    """Collects IMU + wheel odom samples during an autonomous drive profile
    and computes imu_yaw from the horizontal accel direction in chip frame."""

    # --- Sample filter thresholds ---
    WZ_STRAIGHT_THRESHOLD = 0.05        # rad/s — |wz| below ⇒ straight motion
    # ACCEL_BODY_THRESHOLD is defined further down with the motion profile
    # so it can reference the chosen ramp speed.
    MIN_SAMPLES = 50

    # Stationary detection for pitch/roll extraction: robot is considered
    # still when BOTH wheel vx and wz are under these thresholds. The IMU
    # at-rest accel then gives chip-frame gravity direction.
    STATIONARY_VX_THRESHOLD = 0.01       # m/s
    STATIONARY_WZ_THRESHOLD = 0.02       # rad/s
    # 150 samples ≈ 3 s at 50 Hz. Baseline is 1.5 s plus the inter-cycle
    # pauses contribute more, so we'll usually have ≥ 300 samples.
    MIN_STATIONARY_SAMPLES = 150

    # --- Motion profile ---
    # Ramps drive signal strength: at cruise 0.5 m/s over a 0.5 s ramp the
    # peak body acceleration is 1.0 m/s², 2-3× the WT901's accel noise
    # floor. Shorter cruise (1 s) keeps total travel under ~0.9 m each
    # way so ~1.5 m of clearance either side is enough.
    CRUISE_SPEED = 0.5                  # m/s
    RAMP_SEC = 0.5                      # acceleration and deceleration durations
    CRUISE_SEC = 1.0                    # constant-speed segment
    PAUSE_SEC = 1.0                     # between forward and backward
    CMD_RATE_HZ = 20.0                  # cmd_vel publish rate (firmware 200 ms watchdog)
    SETTLE_SEC = 1.0                    # post-motion settle before closing window
    BASELINE_SEC = 1.5                  # stationary baseline before driving (gravity-leak subtraction)
    ACCEL_BODY_THRESHOLD = 0.3          # m/s² — bumped from 0.1: at 1.0 peak, only
                                        # samples clearly in the ramp qualify, filtering
                                        # out cruise noise + coast periods.
    N_CYCLES = 3                        # how many forward+back drives to run per
                                        # service call. A single cycle gives a ~±20°
                                        # per-run spread because the WT901's accel
                                        # noise floor is comparable to the 1 m/s² peak
                                        # body accel. 3 cycles → sqrt(3) ≈ 1.7× noise
                                        # reduction on the vector sum → ±10° typical.

    # --- Dock yaw calibration (OpenMower-style) ---
    # When calibration starts with the robot on the dock, we reverse at a
    # very slow speed under RTK-Fixed GPS for DOCK_UNDOCK_DISTANCE_M, then
    # compute dock_yaw = atan2(-dy, -dx) from the motion vector (negated
    # because the robot drives BACKWARD out of the dock — motion direction
    # is opposite to robot heading). Stored on disk where map_server_node
    # and hardware_bridge read it at startup, replacing the old phone-
    # compass measurement that was only ±10 ° accurate.
    DOCK_UNDOCK_SPEED = 0.15            # m/s (slow enough to average GPS noise)
    DOCK_UNDOCK_DISTANCE_M = 2.0        # target displacement (GPS-measured)
    DOCK_UNDOCK_TIMEOUT_SEC = 25.0      # abort if distance not reached
    DOCK_UNDOCK_MIN_DISPLACEMENT = 0.8  # below this the yaw is too noisy
    DOCK_CALIBRATION_PATH = "/ros2_ws/maps/dock_calibration.yaml"

    # --- Magnetometer calibration (opt-in) ---
    # Rotation phase appended to the drive when do_mag_calibration is true.
    # Disabled by default under the Option A path (OpenMower-style GPS-only
    # yaw). Can be enabled if we want to experiment with LIS3MDL later.
    MAG_ROTATE_WZ = 0.30                # rad/s (~17°/s)
    MAG_ROTATE_SEC = 30.0               # total rotation time (≈ 1.4 turns)
    MAG_MIN_SAMPLES = 150
    MAG_CALIBRATION_PATH = "/ros2_ws/maps/mag_calibration.yaml"

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
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._lock = threading.Lock()
        self._collecting = False
        # IMU samples: (t, ax, ay, az). az is used for pitch/roll computation
        # from gravity during stationary periods; ax/ay drive the yaw solve.
        self._imu_samples: list[tuple[float, float, float, float]] = []
        self._odom_samples: list[tuple[float, float, float]] = []
        # Magnetometer samples during the rotation phase: (Bx, By, Bz) in µT.
        self._mag_samples: list[tuple[float, float, float]] = []
        self._collecting_mag = False

        # Latest GPS pose (map-frame ENU) + RTK fix flag for dock-yaw calibration.
        self._latest_gps_x: float | None = None
        self._latest_gps_y: float | None = None
        self._gps_rtk_fixed = False
        self._gps_position_accuracy = 99.0

        # Preflight state (latest values from safety topics).
        self._is_charging = False
        self._emergency_active = False
        self._bt_state: int = HL_STATE_NULL

        self._imu_sub = self.create_subscription(
            Imu, "/imu/data", self._imu_cb, imu_qos, callback_group=self._cb_group
        )
        self._mag_sub = self.create_subscription(
            MagneticField, "/imu/mag_raw", self._mag_cb, imu_qos,
            callback_group=self._cb_group,
        )
        self._gps_sub = self.create_subscription(
            AbsolutePose, "/gps/absolute_pose", self._gps_cb, imu_qos,
            callback_group=self._cb_group,
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/wheel_odom", self._odom_cb, odom_qos,
            callback_group=self._cb_group
        )
        self._status_sub = self.create_subscription(
            HwStatus, "/hardware_bridge/status", self._status_cb, state_qos,
            callback_group=self._cb_group,
        )
        self._emergency_sub = self.create_subscription(
            Emergency, "/hardware_bridge/emergency", self._emergency_cb, state_qos,
            callback_group=self._cb_group,
        )
        # Track BT state so we can enter RECORDING before driving (firmware
        # discards cmd_vel when the BT reports IDLE — see on_cmd_vel in
        # firmware/stm32/ros_usbnode/src/ros/ros_custom/cpp_main.cpp).
        self._bt_status_sub = self.create_subscription(
            HighLevelStatus, "/behavior_tree_node/high_level_status",
            self._bt_status_cb, state_qos, callback_group=self._cb_group,
        )

        # Client for the BT state-machine control service. Used to enter
        # RECORDING (teleop allowed, no blade) before the drive profile and
        # to CANCEL back to IDLE afterward — the cancel path discards any
        # in-progress trajectory so we don't pollute the area list.
        self._hlc_client = self.create_client(
            HighLevelControl,
            "/behavior_tree_node/high_level_control",
            callback_group=self._cb_group,
        )

        # Teleop-priority velocity command. Matches the GUI joystick path so
        # the twist_mux gives it the right priority and all downstream
        # safety (collision_monitor, velocity_smoother) still applies.
        self._cmd_pub = self.create_publisher(
            TwistStamped, "/cmd_vel_teleop", state_qos
        )

        self._srv = self.create_service(
            CalibrateImuYaw,
            "~/calibrate",
            self._calibrate_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "IMU yaw calibration node ready. Ensure robot is undocked with "
            "~1 m of clear space in front and behind, then call ~/calibrate."
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _imu_cb(self, msg: Imu) -> None:
        if not self._collecting:
            return
        t = _stamp_to_float(msg.header.stamp)
        with self._lock:
            self._imu_samples.append(
                (
                    t,
                    float(msg.linear_acceleration.x),
                    float(msg.linear_acceleration.y),
                    float(msg.linear_acceleration.z),
                )
            )

    def _odom_cb(self, msg: Odometry) -> None:
        if not self._collecting:
            return
        t = _stamp_to_float(msg.header.stamp)
        with self._lock:
            self._odom_samples.append(
                (t, float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))
            )

    def _mag_cb(self, msg: MagneticField) -> None:
        if not self._collecting_mag:
            return
        # Store in µT so fit numbers stay human-readable.
        with self._lock:
            self._mag_samples.append(
                (
                    float(msg.magnetic_field.x) * 1e6,
                    float(msg.magnetic_field.y) * 1e6,
                    float(msg.magnetic_field.z) * 1e6,
                )
            )

    def _gps_cb(self, msg: AbsolutePose) -> None:
        self._latest_gps_x = float(msg.pose.pose.position.x)
        self._latest_gps_y = float(msg.pose.pose.position.y)
        self._gps_position_accuracy = float(msg.position_accuracy)
        self._gps_rtk_fixed = bool(msg.flags & AbsolutePose.FLAG_GPS_RTK_FIXED)

    def _status_cb(self, msg: HwStatus) -> None:
        self._is_charging = bool(msg.is_charging)

    def _emergency_cb(self, msg: Emergency) -> None:
        self._emergency_active = bool(msg.active_emergency or msg.latched_emergency)

    def _bt_status_cb(self, msg: HighLevelStatus) -> None:
        self._bt_state = int(msg.state)

    # ------------------------------------------------------------------
    # Autonomous drive
    # ------------------------------------------------------------------

    def _publish_vx(self, vx: float) -> None:
        """Publish a stamped Twist with the given forward velocity."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = float(vx)
        # All other components stay at 0.
        self._cmd_pub.publish(msg)

    def _drive_profile(self, signed_cruise_speed: float) -> None:
        """Run a ramp-cruise-ramp velocity profile at ±CRUISE_SPEED.

        signed_cruise_speed: +CRUISE_SPEED for forward, -CRUISE_SPEED for
        backward. Commands are published at CMD_RATE_HZ throughout so the
        firmware's 200 ms command-watchdog never trips.
        """
        period = 1.0 / self.CMD_RATE_HZ
        # Acceleration ramp
        n = max(1, int(self.RAMP_SEC * self.CMD_RATE_HZ))
        for i in range(n):
            v = signed_cruise_speed * (i + 1) / n
            self._publish_vx(v)
            time.sleep(period)
        # Cruise
        n = max(1, int(self.CRUISE_SEC * self.CMD_RATE_HZ))
        for _ in range(n):
            self._publish_vx(signed_cruise_speed)
            time.sleep(period)
        # Deceleration ramp
        n = max(1, int(self.RAMP_SEC * self.CMD_RATE_HZ))
        for i in range(n):
            v = signed_cruise_speed * (n - i - 1) / n
            self._publish_vx(v)
            time.sleep(period)
        self._publish_vx(0.0)

    def _pause(self, seconds: float) -> None:
        """Publish zero-velocity for `seconds` so the firmware stays fed."""
        period = 1.0 / self.CMD_RATE_HZ
        n = max(1, int(seconds * self.CMD_RATE_HZ))
        for _ in range(n):
            self._publish_vx(0.0)
            time.sleep(period)

    def _publish_wz(self, wz: float) -> None:
        """Publish a stamped Twist with the given angular velocity."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _rotate_profile(self, wz: float, duration_sec: float) -> None:
        """Spin in place at a fixed wz for `duration_sec`, cmd_vel at
        CMD_RATE_HZ so the firmware watchdog never trips."""
        period = 1.0 / self.CMD_RATE_HZ
        n = max(1, int(duration_sec * self.CMD_RATE_HZ))
        for _ in range(n):
            if self._emergency_active:
                break
            self._publish_wz(wz)
            time.sleep(period)
        # Stop at end.
        self._publish_wz(0.0)

    def _wait_for_rtk_fixed(self, timeout_sec: float = 10.0) -> bool:
        """Block until /gps/absolute_pose reports FLAG_GPS_RTK_FIXED or
        timeout elapses. While waiting we publish zero velocity so the
        firmware watchdog stays fed."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._gps_rtk_fixed and self._gps_position_accuracy < 0.05:
                return True
            self._publish_vx(0.0)
            time.sleep(1.0 / self.CMD_RATE_HZ)
        return False

    def _run_dock_yaw_drive(self) -> dict | None:
        """Reverse DOCK_UNDOCK_DISTANCE_M under RTK-Fixed while logging GPS
        displacement. Compute dock_yaw from the motion vector (robot
        heading = opposite of reverse motion direction). Save to disk.
        Returns the result dict on success, None on failure."""
        self.get_logger().info(
            "Dock yaw calibration: waiting for RTK-Fixed (≤ 10 s)…"
        )
        if not self._wait_for_rtk_fixed(10.0):
            self.get_logger().error(
                f"No RTK-Fixed within 10 s (rtk_fixed={self._gps_rtk_fixed}, "
                f"pos_acc={self._gps_position_accuracy:.3f} m). "
                "Cannot compute dock yaw — aborting."
            )
            return None

        x0, y0 = self._latest_gps_x, self._latest_gps_y
        if x0 is None or y0 is None:
            self.get_logger().error(
                "/gps/absolute_pose never arrived despite RTK Fixed — aborting."
            )
            return None

        self.get_logger().info(
            f"RTK-Fixed acquired. Reversing at {self.DOCK_UNDOCK_SPEED:+.2f} m/s "
            f"target {self.DOCK_UNDOCK_DISTANCE_M:.1f} m "
            f"(timeout {self.DOCK_UNDOCK_TIMEOUT_SEC:.0f} s) — "
            f"start pos=({x0:+.3f}, {y0:+.3f})."
        )

        period = 1.0 / self.CMD_RATE_HZ
        t_deadline = time.monotonic() + self.DOCK_UNDOCK_TIMEOUT_SEC
        displacement = 0.0
        while time.monotonic() < t_deadline:
            if self._emergency_active:
                self._publish_vx(0.0)
                self.get_logger().error("Emergency during dock undock — aborting.")
                return None
            self._publish_vx(-self.DOCK_UNDOCK_SPEED)
            if self._latest_gps_x is not None and self._latest_gps_y is not None:
                dx = self._latest_gps_x - x0
                dy = self._latest_gps_y - y0
                displacement = math.hypot(dx, dy)
                if displacement >= self.DOCK_UNDOCK_DISTANCE_M:
                    break
            time.sleep(period)

        # Stop and settle.
        for _ in range(int(1.0 * self.CMD_RATE_HZ)):
            self._publish_vx(0.0)
            time.sleep(period)

        x1, y1 = self._latest_gps_x, self._latest_gps_y
        if x1 is None or y1 is None:
            self.get_logger().error("Lost GPS during dock undock — aborting.")
            return None
        dx = x1 - x0
        dy = y1 - y0
        displacement = math.hypot(dx, dy)

        if displacement < self.DOCK_UNDOCK_MIN_DISPLACEMENT:
            self.get_logger().error(
                f"Only {displacement:.3f} m of GPS displacement after reverse "
                f"(need ≥ {self.DOCK_UNDOCK_MIN_DISPLACEMENT:.1f} m). "
                "Wheels probably slipped on the dock ramp. Aborting."
            )
            return None

        # Robot moved backward → motion vector direction = opposite of
        # robot heading. So heading = atan2(-dy, -dx).
        dock_yaw = math.atan2(-dy, -dx)

        # σ_yaw ≈ atan2(2·σ_pos, displacement). With σ_pos ≈ 7 mm and
        # displacement ≥ 0.8 m that's ≤ 1 °.
        sigma_pos = max(self._gps_position_accuracy, 0.003)
        sigma_yaw_rad = math.atan2(2.0 * sigma_pos, displacement)

        result = {
            "dock_pose_x": float(x0),
            "dock_pose_y": float(y0),
            "dock_pose_yaw_rad": float(dock_yaw),
            "dock_pose_yaw_deg": float(math.degrees(dock_yaw)),
            "undock_displacement_m": float(displacement),
            "yaw_sigma_rad": float(sigma_yaw_rad),
            "yaw_sigma_deg": float(math.degrees(sigma_yaw_rad)),
            "calibrated_at": datetime.now(timezone.utc).isoformat(),
            "speed_ms": self.DOCK_UNDOCK_SPEED,
        }

        try:
            os.makedirs(os.path.dirname(self.DOCK_CALIBRATION_PATH), exist_ok=True)
            with open(self.DOCK_CALIBRATION_PATH, "w") as fh:
                yaml.safe_dump({"dock_calibration": result}, fh, sort_keys=False)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to persist dock calibration to "
                f"{self.DOCK_CALIBRATION_PATH}: {exc}"
            )
            return None

        self.get_logger().info(
            "Dock yaw calibration: start=({:+.3f}, {:+.3f}) end=({:+.3f}, {:+.3f}) "
            "displacement={:.3f} m dock_yaw={:+.2f}° (σ={:.2f}°). "
            "Saved to {}.".format(
                x0, y0, x1, y1, displacement, math.degrees(dock_yaw),
                math.degrees(sigma_yaw_rad), self.DOCK_CALIBRATION_PATH,
            )
        )
        return result

    def _fit_and_save_mag(self, samples: list[tuple[float, float, float]]) -> None:
        """Fit hard-iron + soft-iron calibration and persist to YAML.
        Non-fatal — logs and returns without raising if samples are
        insufficient or the file write fails."""
        if len(samples) < self.MAG_MIN_SAMPLES:
            self.get_logger().warn(
                f"Too few mag samples for calibration ({len(samples)} < "
                f"{self.MAG_MIN_SAMPLES}). Verify /imu/mag_raw is publishing."
            )
            return
        cal = fit_mag_min_max(samples)
        out = {
            "mag_calibration": {
                **cal,
                "calibrated_at": datetime.now(timezone.utc).isoformat(),
                "rotate_wz_rad_s": self.MAG_ROTATE_WZ,
                "rotate_duration_sec": self.MAG_ROTATE_SEC,
            },
        }
        try:
            os.makedirs(os.path.dirname(self.MAG_CALIBRATION_PATH), exist_ok=True)
            with open(self.MAG_CALIBRATION_PATH, "w") as fh:
                yaml.safe_dump(out, fh, sort_keys=False)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to write mag calibration to "
                f"{self.MAG_CALIBRATION_PATH}: {exc}"
            )
            return
        self.get_logger().info(
            "Mag calibration: samples={}  offset=({:+7.2f}, {:+7.2f}, {:+7.2f}) µT  "
            "scale=({:.3f}, {:.3f}, {:.3f})  |B|={:.2f} ± {:.2f} µT  "
            "(Earth range ≈ 25–65 µT)  saved to {}".format(
                cal["sample_count"],
                cal["offset_x_uT"], cal["offset_y_uT"], cal["offset_z_uT"],
                cal["scale_x"], cal["scale_y"], cal["scale_z"],
                cal["magnitude_mean_uT"], cal["magnitude_std_uT"],
                self.MAG_CALIBRATION_PATH,
            )
        )

    def _call_hlc(self, command: int, label: str) -> bool:
        """Call the BT's HighLevelControl service synchronously.

        Returns True on success. Spin-waits on the future so the service
        call completes even though we're inside another service handler
        (requires MultiThreadedExecutor + ReentrantCallbackGroup, which
        is how this node is set up)."""
        if not self._hlc_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                f"HighLevelControl service not available — cannot {label}."
            )
            return False
        req = HighLevelControl.Request()
        req.command = command
        future = self._hlc_client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().warn(f"HighLevelControl {label} timed out.")
            return False
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().warn(
                f"HighLevelControl {label} rejected by BT."
            )
            return False
        return True

    def _wait_for_bt_state(self, target: int, timeout_sec: float) -> bool:
        """Poll self._bt_state until it matches `target`, up to timeout."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._bt_state == target:
                return True
            time.sleep(0.1)
        return self._bt_state == target

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------

    def _calibrate_cb(
        self,
        request: CalibrateImuYaw.Request,
        response: CalibrateImuYaw.Response,
    ) -> CalibrateImuYaw.Response:
        # The request's duration_sec is now ignored (the motion profile
        # dictates how long collection lasts). Kept in the .srv for
        # backwards compatibility; a future caller that wants to override
        # the total length can still drive that externally.

        # --- Preflight checks ---
        # Charging is now ALLOWED: if the robot is on the dock we run an
        # extra pre-phase that reverses 2 m slowly under RTK-Fixed and
        # captures dock_yaw from the GPS track. Skipping that phase is
        # fine too — the IMU calibration works identically from a
        # stationary off-dock start.
        do_dock_yaw_calibration = bool(self._is_charging)
        if self._emergency_active:
            response.success = False
            response.message = (
                "Refusing to calibrate while an emergency is active/latched. "
                "Clear the emergency first."
            )
            return response
        if self._bt_state == HL_STATE_AUTONOMOUS:
            response.success = False
            response.message = (
                "Refusing to calibrate while BT is AUTONOMOUS (mowing in "
                "progress). Stop mowing first (HOME command)."
            )
            return response

        # --- Enter RECORDING so the firmware accepts cmd_vel_teleop ---
        # The firmware's on_cmd_vel() discards commands when the BT reports
        # HL_MODE_IDLE. RECORDING lets teleop through without starting the
        # blade (unlike MANUAL_MOWING), and the CANCEL path discards the
        # in-progress trajectory so we don't pollute the area list.
        need_exit_recording = False
        if self._bt_state != HL_STATE_RECORDING:
            self.get_logger().info("Entering RECORDING mode for calibration drive.")
            if not self._call_hlc(HL_CMD_RECORD_AREA, "enter recording"):
                response.success = False
                response.message = (
                    "Could not enter RECORDING mode via BT. "
                    "Check that the behavior_tree_node is alive."
                )
                return response
            if not self._wait_for_bt_state(HL_STATE_RECORDING, timeout_sec=5.0):
                # Still try to cancel in case partial transition.
                self._call_hlc(HL_CMD_RECORD_CANCEL, "cancel after failed entry")
                response.success = False
                response.message = (
                    f"BT did not transition to RECORDING within 5s "
                    f"(stuck at state={self._bt_state})."
                )
                return response
            need_exit_recording = True

        with self._lock:
            self._imu_samples.clear()
            self._odom_samples.clear()
            self._collecting = True

        # --- Phase 0: dock-yaw calibration (on-dock starts only) --------
        dock_yaw_result = None
        if do_dock_yaw_calibration:
            dock_yaw_result = self._run_dock_yaw_drive()
            if dock_yaw_result is None:
                # Hard failure — robot could not gather a valid track.
                # Still exit RECORDING so we don't leave the BT stuck.
                if need_exit_recording:
                    self._call_hlc(HL_CMD_RECORD_CANCEL,
                                   "cancel after dock yaw failure")
                response.success = False
                response.message = (
                    "Dock yaw calibration failed: no RTK Fixed or "
                    "insufficient GPS displacement during the 2 m reverse. "
                    "Check the dock area visibility and retry."
                )
                return response

        # --- Stationary baseline ---
        # Collect IMU while perfectly still to measure the DC offset on
        # accel_x / accel_y caused by chassis tilt and sensor bias. That
        # offset is subtracted from the motion samples before computing
        # per-sample imu_yaw, otherwise gravity leaks dominate the result
        # (we observed raw ax readings of ±2 m/s² during a 0.2 m/s² drive
        # — baseline was ~±0.8 m/s² from tilt).
        self.get_logger().info(
            f"Capturing {self.BASELINE_SEC:.1f}s stationary baseline before drive."
        )
        self._pause(self.BASELINE_SEC)

        # Remember how many samples were baseline so we can exclude them
        # from the motion analysis.
        with self._lock:
            baseline_imu = list(self._imu_samples)
        baseline_start_count = len(baseline_imu)

        total_motion_sec = (
            2.0 * (self.RAMP_SEC + self.CRUISE_SEC + self.RAMP_SEC)
            + self.PAUSE_SEC
        )
        self.get_logger().info(
            f"Autonomous calibration drive starting — forward "
            f"{self.CRUISE_SPEED:.2f} m/s then back, ~{total_motion_sec:.0f}s total."
        )

        try:
            for cycle in range(self.N_CYCLES):
                self.get_logger().info(
                    f"Drive cycle {cycle + 1}/{self.N_CYCLES}"
                )
                # Forward leg: ramp up, cruise, ramp down to 0.
                self._drive_profile(+self.CRUISE_SPEED)
                # Pause between directions.
                self._pause(self.PAUSE_SEC)
                # Backward leg: same profile, negative speed.
                self._drive_profile(-self.CRUISE_SPEED)
                # Settle before the next cycle (or before closing the
                # window if this is the last cycle).
                self._pause(self.SETTLE_SEC)
        except Exception as exc:
            # Stop the robot no matter what, then surface the failure.
            for _ in range(5):
                self._publish_vx(0.0)
                time.sleep(0.05)
            with self._lock:
                self._collecting = False
            # Still try to exit recording so we don't leave the BT stuck.
            if need_exit_recording:
                self._call_hlc(HL_CMD_RECORD_CANCEL, "cancel after drive error")
            response.success = False
            response.message = f"Drive profile errored: {exc}"
            return response
        finally:
            # Belt-and-braces stop: one last zero command after the profile.
            self._publish_vx(0.0)

        # --- Magnetometer calibration phase (opt-in, default off) -------
        # Option A path (OpenMower-style) uses GPS for absolute yaw and
        # does not need the magnetometer. Keeping the rotation phase
        # gated behind a parameter so we can re-enable it if we decide
        # to experiment with the LIS3MDL later.
        do_mag_calibration = bool(
            self.declare_parameter("do_mag_calibration", False).value
        ) if not self.has_parameter("do_mag_calibration") else bool(
            self.get_parameter("do_mag_calibration").value
        )
        if do_mag_calibration:
            self.get_logger().info(
                f"Magnetometer calibration: rotating at wz="
                f"{self.MAG_ROTATE_WZ:+.2f} rad/s for {self.MAG_ROTATE_SEC:.0f}s "
                f"(≈{math.degrees(self.MAG_ROTATE_WZ * self.MAG_ROTATE_SEC):.0f}°)."
            )
            with self._lock:
                self._mag_samples.clear()
                self._collecting_mag = True
            try:
                self._rotate_profile(self.MAG_ROTATE_WZ, self.MAG_ROTATE_SEC)
            except Exception as exc:
                for _ in range(5):
                    self._publish_wz(0.0)
                    time.sleep(0.05)
                self.get_logger().error(f"Mag rotate phase errored: {exc}")
            finally:
                with self._lock:
                    self._collecting_mag = False
                self._publish_wz(0.0)

        # --- Exit RECORDING cleanly — CANCEL discards the trajectory ---
        if need_exit_recording:
            self._call_hlc(HL_CMD_RECORD_CANCEL, "cancel recording")
            self._wait_for_bt_state(HL_STATE_IDLE, timeout_sec=3.0)

        with self._lock:
            self._collecting = False
            imu_samples = list(self._imu_samples)
            odom_samples = list(self._odom_samples)
            mag_samples = list(self._mag_samples)

        # Fit and persist magnetometer calibration (non-fatal if it fails
        # — the IMU yaw result is still reported via the service response).
        if do_mag_calibration:
            self._fit_and_save_mag(mag_samples)

        self.get_logger().info(
            f"Drive complete — {len(imu_samples)} IMU / "
            f"{len(odom_samples)} odom samples collected."
        )

        result = self._compute_imu_yaw(
            imu_samples, odom_samples, baseline_count=baseline_start_count
        )
        response.success = result["success"]
        response.message = result["message"]
        response.imu_yaw_rad = result["imu_yaw_rad"]
        response.imu_yaw_deg = result["imu_yaw_deg"]
        response.samples_used = int(result["samples_used"])
        response.std_dev_deg = result["std_dev_deg"]
        response.imu_pitch_rad = result["imu_pitch_rad"]
        response.imu_pitch_deg = result["imu_pitch_deg"]
        response.imu_roll_rad = result["imu_roll_rad"]
        response.imu_roll_deg = result["imu_roll_deg"]
        response.stationary_samples_used = int(result["stationary_samples_used"])
        response.gravity_mag_mps2 = result["gravity_mag_mps2"]

        # Dock pose fields — only filled when the pre-phase actually ran
        # and succeeded. Leave dock_valid false + zeros otherwise so the
        # GUI knows to keep the previously configured dock_pose_yaw.
        if dock_yaw_result is not None:
            response.dock_valid = True
            response.dock_pose_x = float(dock_yaw_result["dock_pose_x"])
            response.dock_pose_y = float(dock_yaw_result["dock_pose_y"])
            response.dock_pose_yaw_rad = float(dock_yaw_result["dock_pose_yaw_rad"])
            response.dock_pose_yaw_deg = float(dock_yaw_result["dock_pose_yaw_deg"])
            response.dock_yaw_sigma_deg = float(dock_yaw_result["yaw_sigma_deg"])
            response.dock_undock_displacement_m = float(
                dock_yaw_result["undock_displacement_m"])
        else:
            response.dock_valid = False

        if response.success:
            self.get_logger().info(
                f"imu_yaw = {response.imu_yaw_rad:+.4f} rad "
                f"({response.imu_yaw_deg:+.2f}°) from {response.samples_used} "
                f"samples, stddev {response.std_dev_deg:.2f}°"
            )
            if response.stationary_samples_used >= self.MIN_STATIONARY_SAMPLES:
                self.get_logger().info(
                    f"imu_pitch = {response.imu_pitch_rad:+.4f} rad "
                    f"({response.imu_pitch_deg:+.2f}°), "
                    f"imu_roll = {response.imu_roll_rad:+.4f} rad "
                    f"({response.imu_roll_deg:+.2f}°) from "
                    f"{response.stationary_samples_used} stationary samples "
                    f"(|g|={response.gravity_mag_mps2:.3f} m/s²). "
                    f"Promote to mowgli_robot.yaml if |pitch| or |roll| > 1°."
                )
            else:
                self.get_logger().warn(
                    f"Pitch/roll not computed: only "
                    f"{response.stationary_samples_used} stationary IMU "
                    f"samples (need ≥ {self.MIN_STATIONARY_SAMPLES})."
                )
            if response.dock_valid:
                self.get_logger().info(
                    f"dock_pose = ({response.dock_pose_x:+.3f}, "
                    f"{response.dock_pose_y:+.3f}) yaw="
                    f"{response.dock_pose_yaw_deg:+.2f}° "
                    f"(σ={response.dock_yaw_sigma_deg:.2f}°, "
                    f"undock displacement={response.dock_undock_displacement_m:.3f} m). "
                    "Promote to mowgli_robot.yaml → dock_pose_yaw "
                    f"= {response.dock_pose_yaw_rad:.4f}."
                )
        else:
            self.get_logger().warn(f"Calibration failed: {response.message}")

        return response

    # ------------------------------------------------------------------
    # Numerical core — pure function over collected samples
    # ------------------------------------------------------------------

    def _compute_imu_yaw(
        self,
        imu_samples: list[tuple[float, float, float, float]],
        odom_samples: list[tuple[float, float, float]],
        baseline_count: int = 0,
    ) -> dict:
        empty = {
            "success": False,
            "message": "",
            "imu_yaw_rad": 0.0,
            "imu_yaw_deg": 0.0,
            "samples_used": 0,
            "std_dev_deg": 0.0,
            "imu_pitch_rad": 0.0,
            "imu_pitch_deg": 0.0,
            "imu_roll_rad": 0.0,
            "imu_roll_deg": 0.0,
            "stationary_samples_used": 0,
            "gravity_mag_mps2": 0.0,
        }

        # Split into baseline (stationary) and motion samples. The baseline
        # gives us the DC offset on ax/ay caused by chassis tilt + sensor
        # bias, which is subtracted from motion samples so we're reading
        # the MOTION-INDUCED horizontal accel only.
        baseline_ax = baseline_ay = 0.0
        baseline_samples = imu_samples[:baseline_count] if baseline_count > 0 else []
        if baseline_count > 0 and len(imu_samples) > baseline_count:
            base = np.asarray(imu_samples[:baseline_count], dtype=np.float64)
            baseline_ax = float(np.mean(base[:, 1]))
            baseline_ay = float(np.mean(base[:, 2]))
            imu_samples = imu_samples[baseline_count:]
        self._debug_baseline = (baseline_ax, baseline_ay)

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

        odom = np.asarray(odom_samples, dtype=np.float64)
        imu = np.asarray(imu_samples, dtype=np.float64)

        odom_t = odom[:, 0]
        odom_vx = odom[:, 1]
        odom_wz = odom[:, 2]

        order_o = np.argsort(odom_t)
        odom_t = odom_t[order_o]
        odom_vx = odom_vx[order_o]
        odom_wz = odom_wz[order_o]

        order_i = np.argsort(imu[:, 0])
        imu = imu[order_i]

        if len(odom_t) < 3:
            empty["message"] = "Not enough odom samples for central difference."
            return empty
        a_body = np.zeros_like(odom_vx)
        a_body[1:-1] = (odom_vx[2:] - odom_vx[:-2]) / np.maximum(
            odom_t[2:] - odom_t[:-2], 1e-6
        )
        a_body[0] = a_body[1]
        a_body[-1] = a_body[-2]

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

        straight = np.abs(wz_interp) < self.WZ_STRAIGHT_THRESHOLD
        moving = np.abs(a_interp) > self.ACCEL_BODY_THRESHOLD
        mask = straight & moving

        valid_count = int(np.count_nonzero(mask))
        if valid_count < self.MIN_SAMPLES:
            empty["samples_used"] = valid_count
            empty["message"] = (
                f"Only {valid_count} valid samples (need ≥ {self.MIN_SAMPLES}). "
                "Is the robot stuck, colliding, or on uneven ground?"
            )
            return empty

        # Subtract the stationary baseline so we're using motion-induced
        # accel only (removes gravity-tilt leak + accel DC bias).
        ax_valid = imu[mask, 1] - baseline_ax
        ay_valid = imu[mask, 2] - baseline_ay
        a_sign = np.sign(a_interp[mask])

        # Rotate by sign(a_body) so decel samples align with accel samples
        # along a single body +X direction before summing.
        ax_s = ax_valid * a_sign
        ay_s = ay_valid * a_sign

        # VECTOR-SUM approach (not per-sample atan2):
        #   Chip-frame accel for body-frame +X accel has mean vector
        #   (a*cos(imu_yaw), -a*sin(imu_yaw)). Summing sign-corrected
        #   samples gives a vector along that mean direction; zero-mean
        #   noise cancels as 1/sqrt(N). Equivalent to a MAGNITUDE-WEIGHTED
        #   circular average — samples with small |accel| (noise-dominated)
        #   contribute proportionally less than large-|accel| ones. The
        #   old per-sample atan2 + unit-circle average treated all samples
        #   equally and exploded when the WT901's ~0.5 m/s² noise floor
        #   approached the peak body acceleration.
        sum_ax = float(np.sum(ax_s))
        sum_ay = float(np.sum(ay_s))
        imu_yaw_rad = math.atan2(-sum_ay, sum_ax)

        # Confidence: vector-sum magnitude / sum of per-sample magnitudes.
        # 1.0 = samples perfectly aligned, 0.0 = uniform random directions.
        sum_abs = float(np.sum(np.hypot(ax_s, ay_s)))
        r_bar = (
            math.hypot(sum_ax, sum_ay) / sum_abs if sum_abs > 1e-9 else 0.0
        )
        r_bar = min(max(r_bar, 1e-9), 1.0)
        std_rad = math.sqrt(-2.0 * math.log(r_bar))

        # Per-sample diagnostic so a noisy run is visibly noisy even if the
        # vector sum resolves cleanly.
        per_sample = np.arctan2(-ay_s, ax_s)
        per_sample_r = math.hypot(
            float(np.mean(np.cos(per_sample))),
            float(np.mean(np.sin(per_sample))),
        )
        # ------------------------------------------------------------------
        # Pitch / roll from stationary samples
        # ------------------------------------------------------------------
        # Use BOTH the initial baseline AND any IMU samples where wheel_vx
        # and wz are effectively zero. At rest the chip-frame accel reads
        # pure gravity rotated by the URDF (roll, pitch, yaw) mounting, so:
        #   pitch = atan2(-ax_chip, az_chip)
        #   roll  = atan2( ay_chip, az_chip)
        # The URDF applies roll·pitch·yaw in order, meaning pitch/roll live
        # in the chip frame (pre-yaw) — same formula as hardware_bridge's
        # per-dock `Implied mounting tilt` log. The measurement is
        # independent of the yaw solve above; they just share sample stream.
        if len(baseline_samples) > 0:
            base_np = np.asarray(baseline_samples, dtype=np.float64)
            base_ax = base_np[:, 1]
            base_ay = base_np[:, 2]
            base_az = base_np[:, 3]
        else:
            base_ax = np.empty(0)
            base_ay = np.empty(0)
            base_az = np.empty(0)

        # Any post-baseline sample whose wheel vx and wz are both below
        # the stationary thresholds counts too. `wz_interp` already gives
        # us wz; interpolate vx the same way.
        vx_interp = odom_vx[left] + (odom_vx[right] - odom_vx[left]) * frac
        still = (
            (np.abs(vx_interp) < self.STATIONARY_VX_THRESHOLD)
            & (np.abs(wz_interp) < self.STATIONARY_WZ_THRESHOLD)
        )
        still_ax = imu[still, 1]
        still_ay = imu[still, 2]
        still_az = imu[still, 3]

        all_ax = np.concatenate([base_ax, still_ax])
        all_ay = np.concatenate([base_ay, still_ay])
        all_az = np.concatenate([base_az, still_az])
        stationary_count = int(all_az.size)

        pitch_rad = 0.0
        roll_rad = 0.0
        gravity_mag = 0.0
        if stationary_count >= self.MIN_STATIONARY_SAMPLES:
            mean_ax = float(np.mean(all_ax))
            mean_ay = float(np.mean(all_ay))
            mean_az = float(np.mean(all_az))
            pitch_rad = math.atan2(-mean_ax, mean_az)
            roll_rad = math.atan2(mean_ay, mean_az)
            gravity_mag = math.sqrt(
                mean_ax * mean_ax + mean_ay * mean_ay + mean_az * mean_az
            )

        msg = (
            f"imu_yaw={math.degrees(imu_yaw_rad):+.2f}° from {valid_count} "
            f"samples (vector R={r_bar:.2f}, per-sample R={per_sample_r:.2f})."
        )
        if stationary_count >= self.MIN_STATIONARY_SAMPLES:
            msg += (
                f" pitch={math.degrees(pitch_rad):+.2f}° "
                f"roll={math.degrees(roll_rad):+.2f}° "
                f"from {stationary_count} stationary samples "
                f"(|g|={gravity_mag:.3f} m/s²)."
            )

        return {
            "success": True,
            "message": msg,
            "imu_yaw_rad": imu_yaw_rad,
            "imu_yaw_deg": math.degrees(imu_yaw_rad),
            "samples_used": valid_count,
            "std_dev_deg": math.degrees(std_rad),
            "imu_pitch_rad": pitch_rad,
            "imu_pitch_deg": math.degrees(pitch_rad),
            "imu_roll_rad": roll_rad,
            "imu_roll_deg": math.degrees(roll_rad),
            "stationary_samples_used": stationary_count,
            "gravity_mag_mps2": gravity_mag,
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
