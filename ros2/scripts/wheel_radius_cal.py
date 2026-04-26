#!/usr/bin/env python3
"""Drive forward 1 m then backward 1 m. Compare wheel-integrated distance vs
RAW GPS displacement (NavSatFix → local ENU). Bypasses the EKF, since fused
pose is sensor-fused at sub-meter granularity but not a clean ground truth.

Subscribes:
  /wheel_odom (nav_msgs/Odometry) — twist.linear.x integrated
  /gps/fix    (sensor_msgs/NavSatFix) — raw RTK position

Publishes:
  /cmd_vel_teleop (geometry_msgs/TwistStamped)

Reports forward and reverse: wheel distance, GPS straight-line, ratio,
asymmetry. Requires RTK-Fixed (status==2) for ~3 mm sigma.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def llh_to_ecef(lat_deg, lon_deg, h):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat)
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * s * s)
    x = (N + h) * math.cos(lat) * math.cos(lon)
    y = (N + h) * math.cos(lat) * math.sin(lon)
    z = (N * (1 - WGS84_E2) + h) * s
    return x, y, z


def ecef_to_enu(x, y, z, lat0_deg, lon0_deg, h0):
    x0, y0, z0 = llh_to_ecef(lat0_deg, lon0_deg, h0)
    dx, dy, dz = x - x0, y - y0, z - z0
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    sl, cl = math.sin(lat0), math.cos(lat0)
    so, co = math.sin(lon0), math.cos(lon0)
    e = -so * dx + co * dy
    n = -sl * co * dx - sl * so * dy + cl * dz
    u = cl * co * dx + cl * so * dy + sl * dz
    return e, n, u


class RadiusCal(Node):
    def __init__(self, target_m=1.0, vx_cmd=0.20):
        super().__init__('wheel_radius_cal')
        self.target = target_m
        self.vx_cmd = vx_cmd
        self.phase = 'wait_subs'
        self.phase_t = None

        self.wheel_dist = 0.0
        self.last_wheel_t = None

        self.gps = None  # (lat, lon, alt, status)
        self.datum = None  # (lat0, lon0, h0)

        self.gps_start = None
        self.gps_end_fwd = None
        self.gps_start_rev = None
        self.gps_end_rev = None
        self.wheel_start = None
        self.wheel_end_fwd = None
        self.wheel_start_rev = None
        self.wheel_end_rev = None

        self.create_subscription(Odometry, '/wheel_odom', self.cb_wheel, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.cb_gps, 10)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
        self.timer = self.create_timer(0.05, self.tick)

    def cb_wheel(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vx = msg.twist.twist.linear.x
        if self.last_wheel_t is not None:
            dt = t - self.last_wheel_t
            if 0 < dt < 0.5:
                self.wheel_dist += vx * dt
        self.last_wheel_t = t

    def cb_gps(self, msg: NavSatFix):
        self.gps = (msg.latitude, msg.longitude, msg.altitude, msg.status.status)

    def gps_enu(self):
        if self.gps is None or self.datum is None:
            return None
        x, y, z = llh_to_ecef(self.gps[0], self.gps[1], self.gps[2])
        e, n, _ = ecef_to_enu(x, y, z, *self.datum)
        return (e, n)

    def publish_cmd(self, vx):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = float(vx)
        self.pub.publish(m)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.phase == 'wait_subs':
            if self.last_wheel_t is not None and self.gps is not None:
                if self.gps[3] != 2:
                    self.get_logger().warn(f'GPS status={self.gps[3]} (need 2 = RTK-Fixed). Waiting...')
                    self.publish_cmd(0.0)
                    return
                self.datum = (self.gps[0], self.gps[1], self.gps[2])
                self.gps_start = self.gps_enu()
                self.wheel_start = self.wheel_dist
                self.phase = 'forward'
                self.phase_t = now
                self.get_logger().info(f'RTK-Fixed locked. Forward 1.0 m at {self.vx_cmd:.2f} m/s')
            self.publish_cmd(0.0)
            return

        if self.phase == 'forward':
            traveled = self.wheel_dist - self.wheel_start
            if traveled >= self.target or (now - self.phase_t) > 15.0:
                self.gps_end_fwd = self.gps_enu()
                self.wheel_end_fwd = self.wheel_dist
                self.phase = 'settle1'
                self.phase_t = now
                self.get_logger().info('Forward done, settling 1.5 s')
                self.publish_cmd(0.0)
            else:
                self.publish_cmd(self.vx_cmd)
            return

        if self.phase == 'settle1':
            self.publish_cmd(0.0)
            if now - self.phase_t > 1.5:
                self.gps_start_rev = self.gps_enu()
                self.wheel_start_rev = self.wheel_dist
                self.phase = 'reverse'
                self.phase_t = now
                self.get_logger().info(f'Reverse 1.0 m at {-self.vx_cmd:.2f} m/s')
            return

        if self.phase == 'reverse':
            traveled = self.wheel_start_rev - self.wheel_dist
            if traveled >= self.target or (now - self.phase_t) > 15.0:
                self.gps_end_rev = self.gps_enu()
                self.wheel_end_rev = self.wheel_dist
                self.phase = 'settle2'
                self.phase_t = now
                self.get_logger().info('Reverse done, settling 1.5 s')
                self.publish_cmd(0.0)
            else:
                self.publish_cmd(-self.vx_cmd)
            return

        if self.phase == 'settle2':
            self.publish_cmd(0.0)
            if now - self.phase_t > 1.5:
                self.gps_end_rev_settled = self.gps_enu()
                self.phase = 'done'
                self.report()
                for _ in range(10):
                    self.publish_cmd(0.0)
                    time.sleep(0.05)
                rclpy.shutdown()
            return

    def report(self):
        def dist(a, b):
            return math.hypot(b[0]-a[0], b[1]-a[1])

        fwd_wheel = self.wheel_end_fwd - self.wheel_start
        fwd_gps = dist(self.gps_start, self.gps_end_fwd)
        rev_wheel = self.wheel_start_rev - self.wheel_end_rev
        rev_gps = dist(self.gps_start_rev, self.gps_end_rev)

        fwd_ratio = fwd_wheel / fwd_gps if fwd_gps > 1e-3 else float('nan')
        rev_ratio = rev_wheel / rev_gps if rev_gps > 1e-3 else float('nan')

        net_gps = dist(self.gps_start, self.gps_end_rev_settled)

        print('\n=== WHEEL RADIUS CALIBRATION (raw GPS) ===')
        print(f'  forward:  wheel={fwd_wheel:.3f} m   gps={fwd_gps:.3f} m   ratio(wheel/gps)={fwd_ratio:.4f}')
        print(f'  reverse:  wheel={rev_wheel:.3f} m   gps={rev_gps:.3f} m   ratio(wheel/gps)={rev_ratio:.4f}')
        if not math.isnan(fwd_ratio) and not math.isnan(rev_ratio) and rev_ratio > 1e-6:
            asym = fwd_ratio / rev_ratio
            print(f'  asymmetry (fwd/rev) = {asym:.4f}   (1.000 = no direction bias)')
        print(f'\n  net displacement after fwd+rev: {net_gps:.3f} m (should be ~0)')
        if not math.isnan(fwd_ratio) and not math.isnan(rev_ratio):
            mean = 0.5 * (fwd_ratio + rev_ratio)
            print(f'\n  mean wheel/gps ratio: {mean:.4f}')
            if abs(mean - 1.0) < 0.02:
                print(f'  → wheel radius is correctly calibrated (within 2%)')
            else:
                print(f'  → multiply current wheel_radius by {1.0/mean:.4f} to correct')


def main():
    rclpy.init()
    node = RadiusCal(target_m=1.0, vx_cmd=0.20)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
