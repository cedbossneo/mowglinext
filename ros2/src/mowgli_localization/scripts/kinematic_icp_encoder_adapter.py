#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
kinematic_icp_encoder_adapter.py

Re-publishes Kinematic-ICP's LiDAR-odometry as a "second encoder" twist
source that ekf_odom_node fuses (as odom1) alongside the wheel encoders.

    /kinematic_icp/lidar_odometry (nav_msgs/Odometry, pose + twist)
        -> finite-difference twist (falls back to .twist if populated)
        -> /encoder2/odom              (nav_msgs/Odometry with twist covariance)

Kinematic-ICP's output is already kinematic-constrained: its pose is
seeded by the wheel-odom TF delta and penalised for deviating from the
non-holonomic motion model, so the finite-difference twist cannot
hallucinate lateral motion the way KISS-ICP did on featureless grass.

ekf_odom_node treats this topic as a body-frame twist measurement; the
covariance we advertise gates its influence. Baseline σ values make
healthy ICP twist tighter than wheel-encoder twist (especially on yaw);
degraded samples are inflated so the EKF effectively ignores them.

Safety: this node does not touch drive commands, TF, or any safety
topic. It is purely a measurement re-packager and can be killed/added
at runtime without affecting motor output.
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from nav_msgs.msg import Odometry


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (Z-axis rotation) from a quaternion. Avoids pulling in
    tf_transformations as a hard dep — Kinematic-ICP yields a right-handed
    quaternion with yaw as the dominant rotation for a ground robot."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class KinematicIcpEncoderAdapter(Node):
    """Subscribes to Kinematic-ICP odometry, publishes twist-only encoder odom."""

    # --- Covariance baselines ---
    # Healthy: σ_vx = σ_vy = 0.1 m/s (var 0.01), σ_wz = 0.05 rad/s (var 0.0025).
    # Degraded: σ_vx = 1.0 m/s (var 1.0), σ_wz = 0.5 rad/s (var 0.25).
    VAR_VX_OK, VAR_VY_OK, VAR_WZ_OK = 0.01, 0.01, 0.0025
    VAR_VX_BAD, VAR_VY_BAD, VAR_WZ_BAD = 1.0, 1.0, 0.25

    # --- Plausibility gates ---
    MAX_DT_SEC = 1.0
    MAX_POS_JUMP_M = 2.0

    # Pose-covariance proxy for "match quality" (Kinematic-ICP fills
    # pose.covariance[0] from the position_covariance launch param,
    # default 0.1). If a supervisor cranks this above 1.0 we treat the
    # sample as degraded.
    POS_COV_DEGRADED = 1.0

    def __init__(self) -> None:
        super().__init__("kinematic_icp_encoder_adapter")

        self.declare_parameter("input_topic", "/kinematic_icp/lidar_odometry")
        self.declare_parameter("output_topic", "/encoder2/odom")
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        # Input: /kinematic_icp/lidar_odometry is published with the ROS2
        # default system QoS (RELIABLE, depth=1) by the K-ICP online node.
        in_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Output: /encoder2/odom — ekf_odom_node subscribes RELIABLE, matching
        # hardware_bridge's /wheel_odom publisher QoS. A BEST_EFFORT publisher
        # here would be silently dropped ("incompatible QoS, last policy:
        # RELIABILITY") — we saw this on first deploy.
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._prev_t: Optional[float] = None
        self._prev_xy: Optional[Tuple[float, float]] = None
        self._prev_yaw: Optional[float] = None

        self._pub = self.create_publisher(Odometry, output_topic, out_qos)
        self._sub = self.create_subscription(
            Odometry, input_topic, self._on_odom, in_qos
        )

        self.get_logger().info(
            f"kinematic_icp_encoder_adapter ready: {input_topic} -> {output_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        t = _stamp_to_float(msg.header.stamp)
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        # First sample: prime state and wait for the next one.
        if self._prev_t is None:
            self._prev_t, self._prev_xy, self._prev_yaw = t, (x, y), yaw
            return

        dt = t - self._prev_t
        if dt <= 0.0 or dt > self.MAX_DT_SEC:
            self.get_logger().warn(
                f"Dropping sample: dt={dt:.3f}s outside (0, {self.MAX_DT_SEC}]"
            )
            self._prev_t, self._prev_xy, self._prev_yaw = t, (x, y), yaw
            return

        dx = x - self._prev_xy[0]
        dy = y - self._prev_xy[1]
        if abs(dx) > self.MAX_POS_JUMP_M or abs(dy) > self.MAX_POS_JUMP_M:
            self.get_logger().warn(
                f"Dropping implausible pose jump: dx={dx:.2f}, dy={dy:.2f}"
            )
            self._prev_t, self._prev_xy, self._prev_yaw = t, (x, y), yaw
            return

        dyaw = _wrap_pi(yaw - self._prev_yaw)

        # Prefer Kinematic-ICP's twist when populated (the Odometry message
        # it publishes already fills pose.twist.twist from the kinematic
        # motion model). Fall back to finite-differencing the pose delta
        # in the body frame (rotate dx/dy by -prev_yaw) when the incoming
        # twist is exactly zero — which happens both for legitimate ZUPT
        # and, historically, for the original KISS-ICP messages.
        in_tw = msg.twist.twist
        in_tw_populated = (
            abs(in_tw.linear.x) > 1e-9
            or abs(in_tw.linear.y) > 1e-9
            or abs(in_tw.angular.z) > 1e-9
        )
        if in_tw_populated:
            vx_body = float(in_tw.linear.x)
            vy_body = float(in_tw.linear.y)
            wz = float(in_tw.angular.z)
        else:
            c = math.cos(-self._prev_yaw)
            s = math.sin(-self._prev_yaw)
            vx_body = (c * dx - s * dy) / dt
            vy_body = (s * dx + c * dy) / dt
            wz = dyaw / dt

        healthy = float(msg.pose.covariance[0]) < self.POS_COV_DEGRADED
        if healthy:
            var_vx, var_vy, var_wz = self.VAR_VX_OK, self.VAR_VY_OK, self.VAR_WZ_OK
        else:
            var_vx, var_vy, var_wz = self.VAR_VX_BAD, self.VAR_VY_BAD, self.VAR_WZ_BAD

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose   # pass pose through (EKF ignores it — only twist fields are configured for fusion)
        out.twist.twist.linear.x = vx_body
        out.twist.twist.linear.y = vy_body
        out.twist.twist.angular.z = wz
        cov = [0.0] * 36
        cov[0] = var_vx              # vx
        cov[7] = var_vy              # vy
        cov[14] = 1.0                # vz (unobserved)
        cov[21] = 1.0                # wx (unobserved)
        cov[28] = 1.0                # wy (unobserved)
        cov[35] = var_wz             # wz
        out.twist.covariance = cov
        self._pub.publish(out)

        self._prev_t, self._prev_xy, self._prev_yaw = t, (x, y), yaw


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicIcpEncoderAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
