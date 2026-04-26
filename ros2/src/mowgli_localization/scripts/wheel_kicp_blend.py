#!/usr/bin/env python3
"""Blend /wheel_odom with K-ICP /encoder2/odom into a single twist source.

Why this exists
---------------
robot_localization's ekf_odom_node caps its publish rate at ~5 Hz whenever
its odom1 input (the K-ICP body-frame twist on /encoder2/odom) is alive,
regardless of the configured frequency, queue depths, or measurement
stamps. Without odom1 the same EKF runs at its 25 Hz target. The exact
mechanism is opaque (not history rewind, not CPU contention, not stamp
lag) so we work around it by fusing K-ICP into the wheel twist *before*
the EKF instead of giving the EKF a second odom source.

Output
------
/wheel_odom_fused : nav_msgs/Odometry, body-frame twist with vx, vy, wz
populated and inverse-variance combined per axis. Covariance reflects
the combined uncertainty. Stamp = wallclock now() so the EKF treats it
as a fresh measurement (no rewind).

Fallback
--------
If K-ICP goes silent (>kicp_max_age_sec) we publish the wheel twist
unchanged. That preserves dead-reckoning when LiDAR drops out without
needing a separate guard at the EKF layer.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


def stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class WheelKicpBlend(Node):
    def __init__(self) -> None:
        super().__init__("wheel_kicp_blend")

        # Drop K-ICP samples older than this; fall back to wheel-only.
        self._kicp_max_age = float(
            self.declare_parameter("kicp_max_age_sec", 0.25).value
        )
        # Optional override for output topic — default keeps the legacy
        # /wheel_odom_fused name we wire into ekf_odom.yaml.
        out_topic = str(
            self.declare_parameter("output_topic", "/wheel_odom_fused").value
        )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._latest_kicp: Odometry | None = None
        self._latest_kicp_arrival_t: float = 0.0
        self.create_subscription(
            Odometry, "/encoder2/odom", self._on_kicp, qos_reliable
        )
        self.create_subscription(
            Odometry, "/wheel_odom", self._on_wheel, qos_reliable
        )
        self._pub = self.create_publisher(Odometry, out_topic, qos_reliable)

        self._wheel_count = 0
        self._blended_count = 0
        self._fallback_count = 0
        self.create_timer(30.0, self._log_stats)

        self.get_logger().info(
            f"wheel_kicp_blend ready: /wheel_odom + /encoder2/odom -> {out_topic} "
            f"(kicp_max_age={self._kicp_max_age:.3f}s)"
        )

    def _on_kicp(self, msg: Odometry) -> None:
        self._latest_kicp = msg
        self._latest_kicp_arrival_t = self.get_clock().now().nanoseconds * 1e-9

    def _on_wheel(self, msg: Odometry) -> None:
        self._wheel_count += 1

        # Wheel covariance / twist values
        wt = msg.twist.twist
        wcov = msg.twist.covariance
        wvx, wvy, wwz = float(wt.linear.x), float(wt.linear.y), float(wt.angular.z)
        wvar_vx = max(float(wcov[0]),  1e-9)   # vx
        wvar_vy = max(float(wcov[7]),  1e-9)   # vy (tight non-holo lock)
        wvar_wz = max(float(wcov[35]), 1e-9)   # wz

        out = Odometry()
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose

        now_t = self.get_clock().now().nanoseconds * 1e-9
        kicp = self._latest_kicp
        kicp_age = now_t - self._latest_kicp_arrival_t
        kicp_fresh = kicp is not None and kicp_age < self._kicp_max_age

        if not kicp_fresh:
            # Pass wheel through unchanged (with a fresh stamp).
            out.header.stamp = self.get_clock().now().to_msg()
            out.twist = msg.twist
            self._fallback_count += 1
            self._pub.publish(out)
            return

        # Inverse-variance weighted blend per axis.
        kt = kicp.twist.twist
        kcov = kicp.twist.covariance
        kvx, kvy, kwz = float(kt.linear.x), float(kt.linear.y), float(kt.angular.z)
        kvar_vx = max(float(kcov[0]),  1e-9)
        kvar_vy = max(float(kcov[7]),  1e-9)
        kvar_wz = max(float(kcov[35]), 1e-9)

        def blend(va, var_a, vb, var_b):
            inv_a = 1.0 / var_a
            inv_b = 1.0 / var_b
            mu = (va * inv_a + vb * inv_b) / (inv_a + inv_b)
            var = 1.0 / (inv_a + inv_b)
            return mu, var

        vx, var_vx = blend(wvx, wvar_vx, kvx, kvar_vx)
        # vy: wheel insists on 0 with a tight covariance; if K-ICP also
        # confidently reports 0 (it does on featureless straight runs),
        # the blend stays at 0. If K-ICP detects a real lateral motion
        # (skid), wheel's 0+tight-σ wins anyway unless K-ICP has an even
        # tighter σ — which the K-ICP encoder adapter sets only when its
        # pose covariance is healthy. So the blend behaves correctly:
        # wheel's non-holo lock dominates in normal driving and relaxes
        # when K-ICP has high-confidence cross-track information.
        vy, var_vy = blend(wvy, wvar_vy, kvy, kvar_vy)
        wz, var_wz = blend(wwz, wvar_wz, kwz, kvar_wz)

        out.header.stamp = self.get_clock().now().to_msg()
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.angular.z = wz
        cov = [0.0] * 36
        cov[0]  = var_vx
        cov[7]  = var_vy
        cov[14] = 1.0   # vz unobserved
        cov[21] = 1.0   # wx unobserved
        cov[28] = 1.0   # wy unobserved
        cov[35] = var_wz
        out.twist.covariance = cov
        self._blended_count += 1
        self._pub.publish(out)

    def _log_stats(self) -> None:
        total = self._wheel_count or 1
        self.get_logger().info(
            f"wheel_kicp_blend: {self._wheel_count} wheel msgs, "
            f"{self._blended_count} blended ({100*self._blended_count/total:.0f}%), "
            f"{self._fallback_count} fallback ({100*self._fallback_count/total:.0f}%)"
        )
        self._wheel_count = self._blended_count = self._fallback_count = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelKicpBlend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
