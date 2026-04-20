#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
wheel_odom_tf_node.py

Publishes a raw wheel-only TF chain that Kinematic-ICP consumes as its motion
prior. Exists solely to break the feedback loop that would form if Kinematic-
ICP looked up FusionCore's fused `odom -> base_footprint` TF (Kinematic-ICP's
output goes back into FusionCore via `/encoder2/odom`).

    /wheel_odom (nav_msgs/Odometry, twist-only from hardware_bridge)
        -> integrate twist.linear.x / twist.angular.z
        -> TF: wheel_odom_raw -> base_footprint_wheels

Kinematic-ICP is configured with:
    wheel_odom_frame = wheel_odom_raw
    base_frame       = base_footprint_wheels
so its motion-prior TF lookup returns the pure wheel dead-reckoning pose
— independent of FusionCore, GPS, or IMU.

Safety: this node publishes only TF on a private frame pair. It does not
touch `odom`, `base_footprint`, any drive command, or any Nav2 input.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class WheelOdomTfNode(Node):
    """Integrate /wheel_odom twist into a wheel_odom_raw -> base_footprint_wheels TF.

    /wheel_odom typically publishes at 5 Hz (hardware_bridge rate) while
    Kinematic-ICP looks up the wheel TF at scan-end timestamps — usually
    ahead of the latest wheel message, so raw lookups fail with
    "extrapolation into the future". We fix this by re-broadcasting the
    current pose at a higher rate (rebroadcast_hz, default 50 Hz) with
    fresh timestamps, so K-ICP always has a recent TF to interpolate
    against.
    """

    MAX_DT_SEC = 0.5  # reset integration if messages stall (e.g. hardware reconnect)

    def __init__(self) -> None:
        super().__init__("wheel_odom_tf_node")

        self.declare_parameter("input_topic", "/wheel_odom")
        self.declare_parameter("parent_frame", "wheel_odom_raw")
        self.declare_parameter("child_frame", "base_footprint_wheels")
        self.declare_parameter("rebroadcast_hz", 50.0)

        input_topic = self.get_parameter("input_topic").value
        self._parent = self.get_parameter("parent_frame").value
        self._child = self.get_parameter("child_frame").value
        rebroadcast_hz = float(self.get_parameter("rebroadcast_hz").value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._prev_t: Optional[float] = None
        self._have_odom = False

        self._tf = TransformBroadcaster(self)
        self._sub = self.create_subscription(
            Odometry, input_topic, self._on_odom, sensor_qos
        )
        # High-rate rebroadcast with current-time stamps so K-ICP's
        # scan-end TF lookups always have fresh data.
        self._rebroadcast_timer = self.create_timer(
            1.0 / rebroadcast_hz, self._on_rebroadcast_tick
        )

        self.get_logger().info(
            f"wheel_odom_tf_node ready: {input_topic} -> TF {self._parent} -> {self._child} "
            f"(rebroadcast {rebroadcast_hz:.0f} Hz)"
        )

    @staticmethod
    def _stamp_to_float(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_rebroadcast_tick(self) -> None:
        """Re-send the current pose at a fresh timestamp so consumers always
        have a recent TF available for interpolation/extrapolation."""
        if not self._have_odom:
            return
        self._broadcast(self.get_clock().now().to_msg())

    def _on_odom(self, msg: Odometry) -> None:
        t = self._stamp_to_float(msg.header.stamp)
        self._have_odom = True
        if self._prev_t is None:
            self._prev_t = t
            self._broadcast(msg.header.stamp)
            return

        dt = t - self._prev_t
        self._prev_t = t
        if dt <= 0.0 or dt > self.MAX_DT_SEC:
            # Clock jump or stalled stream — don't integrate bogus dt, just
            # republish the current pose so consumers don't starve.
            self._broadcast(msg.header.stamp)
            return

        vx = float(msg.twist.twist.linear.x)
        wz = float(msg.twist.twist.angular.z)

        # Mid-point Euler integration in 2D (y-velocity is zero by differential
        # drive kinematics — any lateral twist component is ignored).
        yaw_mid = self._yaw + 0.5 * wz * dt
        self._x += vx * math.cos(yaw_mid) * dt
        self._y += vx * math.sin(yaw_mid) * dt
        self._yaw = math.atan2(math.sin(self._yaw + wz * dt), math.cos(self._yaw + wz * dt))

        self._broadcast(msg.header.stamp)

    def _broadcast(self, stamp) -> None:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._parent
        tf.child_frame_id = self._child
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        half = 0.5 * self._yaw
        tf.transform.rotation.z = math.sin(half)
        tf.transform.rotation.w = math.cos(half)
        self._tf.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
