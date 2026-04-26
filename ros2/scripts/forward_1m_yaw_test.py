#!/usr/bin/env python3
"""Drive forward ~1 m at 0.15 m/s, logging yaw from every source.

Captures: fusion yaw (map→base_footprint), gyro-integrated yaw, wheel-
integrated yaw, GPS COG (course over ground), GPS x/y excursion.
Prints per-second snapshot + final summary so we can spot which source
diverges when going forward (per user observation: forward = orientation
goes wild, backward = fine)."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from mowgli_interfaces.srv import HighLevelControl


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def llh_to_ecef(lat, lon, h):
    lat = math.radians(lat); lon = math.radians(lon)
    s = math.sin(lat)
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * s * s)
    return ((N + h) * math.cos(lat) * math.cos(lon),
            (N + h) * math.cos(lat) * math.sin(lon),
            (N * (1 - WGS84_E2) + h) * s)


def ecef_to_enu(x, y, z, lat0, lon0, h0):
    x0, y0, z0 = llh_to_ecef(lat0, lon0, h0)
    dx, dy, dz = x - x0, y - y0, z - z0
    lat0r = math.radians(lat0); lon0r = math.radians(lon0)
    sl, cl = math.sin(lat0r), math.cos(lat0r)
    so, co = math.sin(lon0r), math.cos(lon0r)
    e = -so * dx + co * dy
    n = -sl * co * dx - sl * so * dy + cl * dz
    return e, n


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def unwrap(prev, cur):
    d = cur - prev
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return prev + d


class ForwardYawTest(Node):
    def __init__(self, distance, vx):
        super().__init__("forward_1m_yaw_test")
        self.distance = distance
        self.vx = vx

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel_teleop", 10)
        self.create_subscription(Odometry, "/odometry/filtered_map", self.on_fusion, 10)
        self.create_subscription(Odometry, "/wheel_odom", self.on_wheel, 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, sensor_qos)
        self.create_subscription(NavSatFix, "/gps/fix", self.on_gps, sensor_qos)

        self.cli = self.create_client(HighLevelControl,
                                      "/behavior_tree_node/high_level_control")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for /high_level_control")

        self.fusion_yaw = None
        self.fusion_x0 = None; self.fusion_y0 = None
        self.fusion_x = 0.0; self.fusion_y = 0.0

        self.wheel_yaw = 0.0
        self.wheel_dist = 0.0
        self.last_wheel_t = None

        self.gyro_yaw = 0.0
        self.last_gyro_t = None

        self.gps_lat0 = None; self.gps_lon0 = None; self.gps_alt0 = None
        self.gps_e = 0.0; self.gps_n = 0.0
        self.gps_e_prev = None; self.gps_n_prev = None
        self.gps_cog = None  # course over ground (rad), only valid when moving

        self.samples = []
        self.t0 = None

    def call_high_level(self, cmd):
        req = HighLevelControl.Request()
        req.command = cmd
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        return fut.result()

    def on_fusion(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.fusion_x0 is None:
            self.fusion_x0 = x; self.fusion_y0 = y
        self.fusion_x = x - self.fusion_x0
        self.fusion_y = y - self.fusion_y0
        q = msg.pose.pose.orientation
        self.fusion_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    def on_wheel(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_wheel_t is None:
            self.last_wheel_t = now; return
        dt = now - self.last_wheel_t
        self.last_wheel_t = now
        self.wheel_yaw += msg.twist.twist.angular.z * dt
        self.wheel_dist += msg.twist.twist.linear.x * dt

    def on_imu(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_gyro_t is None:
            self.last_gyro_t = now; return
        dt = now - self.last_gyro_t
        self.last_gyro_t = now
        self.gyro_yaw += msg.angular_velocity.z * dt

    def on_gps(self, msg):
        if self.gps_lat0 is None:
            self.gps_lat0 = msg.latitude
            self.gps_lon0 = msg.longitude
            self.gps_alt0 = msg.altitude
            return
        e, n = ecef_to_enu(*llh_to_ecef(msg.latitude, msg.longitude, msg.altitude),
                           self.gps_lat0, self.gps_lon0, self.gps_alt0)
        if self.gps_e_prev is not None:
            de = e - self.gps_e_prev
            dn = n - self.gps_n_prev
            if math.hypot(de, dn) > 0.05:  # only when actually moving
                self.gps_cog = math.atan2(dn, de)
        self.gps_e_prev = e; self.gps_n_prev = n
        self.gps_e = e; self.gps_n = n

    def publish_twist(self, vx):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x = vx
        self.cmd_pub.publish(m)

    def run(self):
        self.get_logger().info("Sending COMMAND_RECORD_AREA (3) to enable teleop")
        for _ in range(20):
            r = self.call_high_level(3)
            if r is None: continue
            self.get_logger().info(f"RECORD ack: {r.success}")
            if r.success: break
            time.sleep(0.05)

        # warm up
        t_start_warm = time.time()
        while time.time() - t_start_warm < 1.0:
            self.publish_twist(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

        # snapshot start
        start_fusion_yaw = self.fusion_yaw or 0.0
        start_wheel = self.wheel_yaw
        start_gyro = self.gyro_yaw
        start_dist = self.wheel_dist

        self.t0 = time.time()
        target_dur = abs(self.distance / self.vx) + 1.5  # +ramp tail
        ramp = 0.8
        last_log = self.t0

        while True:
            t = time.time() - self.t0
            traveled = self.wheel_dist - start_dist
            if abs(traveled) >= abs(self.distance) or t > target_dur + 5.0:
                break
            if t < ramp:
                v = self.vx * (t / ramp)
            else:
                v = self.vx
            self.publish_twist(v)
            rclpy.spin_once(self, timeout_sec=0.02)

            self.samples.append({
                "t": t,
                "fusion_yaw": (self.fusion_yaw or 0.0) - start_fusion_yaw,
                "wheel_yaw": self.wheel_yaw - start_wheel,
                "gyro_yaw": self.gyro_yaw - start_gyro,
                "wheel_dist": traveled,
                "fusion_x": self.fusion_x,
                "fusion_y": self.fusion_y,
                "gps_e": self.gps_e,
                "gps_n": self.gps_n,
                "gps_cog": self.gps_cog,
            })

            if t - (last_log - self.t0) > 0.5:
                last_log = time.time()
                cog_str = f"{math.degrees(self.gps_cog):+7.2f}" if self.gps_cog is not None else "  n/a "
                self.get_logger().info(
                    f"t={t:5.2f}s d={traveled:.3f}m | fusY={math.degrees(self.fusion_yaw or 0):+.2f}° "
                    f"wheelΔY={math.degrees(self.wheel_yaw - start_wheel):+.2f}° "
                    f"gyroΔY={math.degrees(self.gyro_yaw - start_gyro):+.2f}° "
                    f"GPS_COG={cog_str}° fusXY=({self.fusion_x:+.2f},{self.fusion_y:+.2f})"
                )

        # ramp down + stop
        for i in range(20):
            self.publish_twist(self.vx * (1.0 - i / 20.0))
            rclpy.spin_once(self, timeout_sec=0.02)
        for _ in range(20):
            self.publish_twist(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

        # cancel record
        self.call_high_level(6)

        self.print_summary(start_fusion_yaw, start_wheel, start_gyro)

    def print_summary(self, start_fy, start_wy, start_gy):
        if not self.samples:
            print("no samples"); return
        last = self.samples[-1]
        print()
        print("=" * 60)
        print(f"FORWARD {self.distance} m @ {self.vx} m/s — YAW DIVERGENCE")
        print("=" * 60)
        print(f"samples: {len(self.samples)}, duration: {last['t']:.1f} s")
        print(f"wheel-integrated distance: {last['wheel_dist']:.3f} m")
        print(f"fusion XY excursion: ({last['fusion_x']:+.3f}, {last['fusion_y']:+.3f}) m  "
              f"|d|={math.hypot(last['fusion_x'], last['fusion_y']):.3f} m")
        print()
        print("Δ YAW (since start, should all be ~0 for straight forward):")
        print(f"  fusion :   {math.degrees(last['fusion_yaw']):+8.2f}°")
        print(f"  gyro   :   {math.degrees(last['gyro_yaw']):+8.2f}°")
        print(f"  wheel  :   {math.degrees(last['wheel_yaw']):+8.2f}°")
        if last["gps_cog"] is not None:
            print(f"  GPS COG:   {math.degrees(last['gps_cog']):+8.2f}° (absolute)")
        print()
        print("time series (every ~0.5s):")
        last_t = -1.0
        for s in self.samples:
            if s["t"] - last_t < 0.5: continue
            last_t = s["t"]
            cog = f"{math.degrees(s['gps_cog']):+7.2f}" if s['gps_cog'] is not None else "  n/a "
            print(f"  t={s['t']:5.2f}s d={s['wheel_dist']:5.3f}m  "
                  f"fusΔY={math.degrees(s['fusion_yaw']):+7.2f}° "
                  f"gyroΔY={math.degrees(s['gyro_yaw']):+7.2f}° "
                  f"wheelΔY={math.degrees(s['wheel_yaw']):+7.2f}° "
                  f"COG={cog}° "
                  f"fusXY=({s['fusion_x']:+.2f},{s['fusion_y']:+.2f})")


def main():
    rclpy.init()
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    vx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
    node = ForwardYawTest(distance, vx)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
