#!/usr/bin/env python3
"""Log every /gps/fix during a fwd 1m + rev 1m. Print all positions in ENU
relative to first fix, plus message timestamps. Lets us see whether GPS is
actually following the motion."""
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


def llh_to_ecef(lat, lon, h):
    lat = math.radians(lat); lon = math.radians(lon)
    s = math.sin(lat)
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * s * s)
    return ((N+h)*math.cos(lat)*math.cos(lon),
            (N+h)*math.cos(lat)*math.sin(lon),
            (N*(1-WGS84_E2)+h)*s)


def ecef_to_enu(x,y,z, lat0, lon0, h0):
    x0,y0,z0 = llh_to_ecef(lat0, lon0, h0)
    dx,dy,dz = x-x0, y-y0, z-z0
    sl = math.sin(math.radians(lat0)); cl = math.cos(math.radians(lat0))
    so = math.sin(math.radians(lon0)); co = math.cos(math.radians(lon0))
    return (-so*dx + co*dy, -sl*co*dx - sl*so*dy + cl*dz)


class Logger(Node):
    def __init__(self):
        super().__init__('gps_log')
        self.records = []  # (t_msg, t_now, e, n, status)
        self.wheel_records = []  # (t_now, wheel_dist)
        self.wheel_dist = 0.0
        self.last_wheel_t = None
        self.datum = None
        self.phase = 'wait'
        self.phase_t = None
        self.create_subscription(NavSatFix, '/gps/fix', self.cb_gps, 10)
        self.create_subscription(Odometry, '/wheel_odom', self.cb_wheel, 10)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.start_t = None

    def cb_gps(self, msg: NavSatFix):
        if self.datum is None:
            self.datum = (msg.latitude, msg.longitude, msg.altitude)
        x,y,z = llh_to_ecef(msg.latitude, msg.longitude, msg.altitude)
        e,n = ecef_to_enu(x,y,z, *self.datum)
        t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        t_now = time.time()
        self.records.append((t_msg, t_now, e, n, msg.status.status))

    def cb_wheel(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.last_wheel_t is not None:
            dt = t - self.last_wheel_t
            if 0 < dt < 0.5:
                self.wheel_dist += msg.twist.twist.linear.x * dt
        self.last_wheel_t = t
        self.wheel_records.append((time.time(), self.wheel_dist))

    def cmd(self, vx):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = float(vx)
        self.pub.publish(m)

    def tick(self):
        now = time.time()
        if self.phase == 'wait':
            if self.last_wheel_t and self.records:
                self.start_t = now
                self.wheel_start = self.wheel_dist
                self.phase = 'fwd'
                self.phase_t = now
                self.get_logger().info('forward 1m')
            self.cmd(0.0); return
        if self.phase == 'fwd':
            if self.wheel_dist - self.wheel_start >= 1.0 or now-self.phase_t > 12:
                self.phase='settle1'; self.phase_t=now; self.cmd(0.0); self.t_fwd_end=now; return
            self.cmd(0.20); return
        if self.phase == 'settle1':
            self.cmd(0.0)
            if now - self.phase_t > 1.5:
                self.phase='rev'; self.phase_t=now
                self.wheel_start_rev=self.wheel_dist
                self.get_logger().info('reverse 1m')
            return
        if self.phase == 'rev':
            if self.wheel_start_rev - self.wheel_dist >= 1.0 or now-self.phase_t > 12:
                self.phase='settle2'; self.phase_t=now; self.cmd(0.0); self.t_rev_end=now; return
            self.cmd(-0.20); return
        if self.phase == 'settle2':
            self.cmd(0.0)
            if now - self.phase_t > 5.0:
                self.report()
                rclpy.shutdown()

    def report(self):
        print(f'\n=== GPS LOG ({len(self.records)} fixes) ===')
        print(f'{"t_rel":>7} {"t_msg-t_now":>11} {"east":>8} {"north":>8} {"dist0":>8} {"status":>6}')
        e0, n0 = self.records[0][2], self.records[0][3]
        for tm, tn, e, n, st in self.records:
            d = math.hypot(e-e0, n-n0)
            lag = tm - tn
            print(f'{tn-self.start_t:7.2f} {lag:+11.3f} {e:+8.3f} {n:+8.3f} {d:8.3f} {st:6}')
        peak = max(math.hypot(e-e0,n-n0) for _,_,e,n,_ in self.records)
        last = math.hypot(self.records[-1][2]-e0, self.records[-1][3]-n0)
        print(f'\npeak distance from start: {peak:.3f} m')
        print(f'final distance from start: {last:.3f} m')
        print(f'wheel total: {self.wheel_dist:.3f} m (signed)')


def main():
    rclpy.init()
    n = Logger()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass


if __name__ == '__main__':
    main()
