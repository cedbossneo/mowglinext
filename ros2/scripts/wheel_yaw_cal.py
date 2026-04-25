#!/usr/bin/env python3
"""Spin in place ~360° and compare wheel-integrated yaw vs gyro-integrated yaw.

Subscribes:
  /wheel_odom (nav_msgs/Odometry) — uses twist.angular.z
  /imu/data   (sensor_msgs/Imu)   — uses angular_velocity.z

Publishes:
  /cmd_vel_teleop (geometry_msgs/TwistStamped) — wz command, vx=0

Stops when |gyro_yaw_integrated| >= target_deg, then prints both integrals
and the ratio (wheel/gyro). 1.000 = perfect; >1 = wheel over-reports rotation
(track width too large or one wheel slipping); <1 = under-reports.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class YawCal(Node):
    def __init__(self, target_deg=360.0, wz_cmd=0.3):
        super().__init__('wheel_yaw_cal')
        self.target = math.radians(target_deg)
        self.wz_cmd = wz_cmd
        self.wheel_yaw = 0.0
        self.gyro_yaw = 0.0
        self.last_wheel_t = None
        self.last_gyro_t = None
        self.start_t = None
        self.done = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Odometry, '/wheel_odom', self.cb_wheel, 10)
        self.create_subscription(Imu, '/imu/data', self.cb_imu, sensor_qos)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz cmd

    def cb_wheel(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wz = msg.twist.twist.angular.z
        if self.last_wheel_t is not None:
            dt = t - self.last_wheel_t
            if 0 < dt < 0.5:
                self.wheel_yaw += wz * dt
        self.last_wheel_t = t

    def cb_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wz = msg.angular_velocity.z
        if self.last_gyro_t is not None:
            dt = t - self.last_gyro_t
            if 0 < dt < 0.5:
                self.gyro_yaw += wz * dt
        self.last_gyro_t = t

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_t is None:
            # Wait for both subs to deliver one msg before starting the clock
            if self.last_wheel_t is None or self.last_gyro_t is None:
                return
            self.start_t = now
            self.get_logger().info('Starting 360° in-place spin')

        elapsed = now - self.start_t

        if not self.done and abs(self.gyro_yaw) >= self.target:
            self.done = True
            self.get_logger().info('Target reached, stopping')

        # Hard timeout safety: 30 s
        if elapsed > 30.0 and not self.done:
            self.done = True
            self.get_logger().warn('Timeout 30s, stopping')

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0 if self.done else self.wz_cmd
        self.pub.publish(msg)

        if self.done and elapsed > 1.0:
            # Send a few zero commands then exit
            for _ in range(10):
                self.pub.publish(msg)
                time.sleep(0.05)
            self.report()
            rclpy.shutdown()

    def report(self):
        w = math.degrees(self.wheel_yaw)
        g = math.degrees(self.gyro_yaw)
        ratio = w / g if abs(g) > 1e-6 else float('nan')
        print('\n=== WHEEL YAW CALIBRATION RESULT ===')
        print(f'  wheel-integrated yaw: {w:+8.2f} deg')
        print(f'  gyro-integrated yaw : {g:+8.2f} deg')
        print(f'  ratio (wheel/gyro)  : {ratio:.4f}')
        print(f'  excess              : {(ratio-1)*100:+.2f} %')
        if abs(g) > 1e-6:
            scale_track = ratio
            print(f'\n  Interpretation: wheel kinematic model is reporting {ratio:.4f}× the true rotation.')
            print(f'  If track-width is the cause, multiply current track_width by {1.0/scale_track:.4f}.')


def main():
    rclpy.init()
    node = YawCal(target_deg=360.0, wz_cmd=0.3)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
