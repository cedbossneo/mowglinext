#!/usr/bin/env python3
"""Undock (back up 1.5m at 0.15 m/s), then rotate 90° at wz=0.30 rad/s for 5.236s.
Reports wheel and gyro integrated rotation delta over the rotation segment only."""
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from mowgli_interfaces.msg import HighLevelStatus, Status as HwStatus, Emergency
from mowgli_interfaces.srv import HighLevelControl

HL_CMD_RECORD_AREA = 3
HL_CMD_RECORD_CANCEL = 6
HL_STATE_RECORDING = 3

RATE = 20.0
PERIOD = 1.0 / RATE


class UndockAndRotate(Node):
    def __init__(self):
        super().__init__("undock_and_rotate")
        self._cb = ReentrantCallbackGroup()
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_imu = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)

        self._hlc = self.create_client(HighLevelControl,
            "/behavior_tree_node/high_level_control", callback_group=self._cb)
        self._pub = self.create_publisher(TwistStamped, "/cmd_vel_teleop", qos)

        self._bt_state = 0
        self._is_charging = False
        self._emergency = False
        self._wheel_yaw = 0.0
        self._gyro_yaw = 0.0
        self._last_wheel_t = None
        self._last_gyro_t = None

        self.create_subscription(HighLevelStatus, "/behavior_tree_node/high_level_status",
            lambda m: setattr(self, "_bt_state", int(m.state)), qos, callback_group=self._cb)
        self.create_subscription(HwStatus, "/hardware_bridge/status",
            lambda m: setattr(self, "_is_charging", bool(m.is_charging)), qos, callback_group=self._cb)
        self.create_subscription(Emergency, "/hardware_bridge/emergency",
            lambda m: setattr(self, "_emergency", bool(m.active_emergency or m.latched_emergency)),
            qos, callback_group=self._cb)
        self.create_subscription(Odometry, "/wheel_odom", self._wheel_cb, qos, callback_group=self._cb)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, qos_imu, callback_group=self._cb)

    def _wheel_cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self._last_wheel_t is not None:
            dt = t - self._last_wheel_t
            if 0 < dt < 1:
                self._wheel_yaw += m.twist.twist.angular.z * dt
        self._last_wheel_t = t

    def _imu_cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self._last_gyro_t is not None:
            dt = t - self._last_gyro_t
            if 0 < dt < 1:
                self._gyro_yaw += m.angular_velocity.z * dt
        self._last_gyro_t = t

    def call_hlc(self, cmd, label):
        if not self._hlc.wait_for_service(timeout_sec=3.0):
            print(f"HLC not ready — {label}"); return False
        req = HighLevelControl.Request(); req.command = cmd
        fut = self._hlc.call_async(req)
        deadline = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return fut.done() and fut.result() and fut.result().success

    def publish(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_footprint"
        m.twist.linear.x = float(vx); m.twist.angular.z = float(wz)
        self._pub.publish(m)

    def drive(self, vx, wz, duration, label=""):
        if label: print(f"→ {label}: vx={vx:+.2f} wz={wz:+.2f} for {duration:.2f}s")
        n = max(1, int(duration * RATE))
        for _ in range(n):
            if self._emergency:
                print("EMERGENCY"); return False
            self.publish(vx, wz); time.sleep(PERIOD)
        for _ in range(5):
            self.publish(0, 0); time.sleep(PERIOD)
        return True

    def pause(self, sec):
        n = max(1, int(sec * RATE))
        for _ in range(n):
            self.publish(0, 0); time.sleep(PERIOD)


def main():
    BACKUP_SPEED = 0.15          # m/s
    BACKUP_DURATION = 10.0       # 10s × 0.15 = 1.5m
    ROTATE_WZ = 0.30             # rad/s
    ROTATE_DURATION = 5.236      # rad: 0.30 × 5.236 = 1.571 = 90°

    rclpy.init()
    node = UndockAndRotate()
    ex = MultiThreadedExecutor(); ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.5)

    if node._emergency:
        print("ABORT: emergency active"); return 1

    # Enter RECORDING (required for teleop to reach firmware)
    if node._bt_state != HL_STATE_RECORDING:
        if not node.call_hlc(HL_CMD_RECORD_AREA, "enter RECORDING"):
            print("ABORT: could not enter RECORDING"); return 1
        for _ in range(50):
            if node._bt_state == HL_STATE_RECORDING: break
            time.sleep(0.1)
    print("In RECORDING")

    # Undock backup
    print(f"\nBacking up {BACKUP_SPEED * BACKUP_DURATION:.2f}m...")
    if not node.drive(-BACKUP_SPEED, 0.0, BACKUP_DURATION, "undock back"):
        print("Undock failed"); node.call_hlc(HL_CMD_RECORD_CANCEL, "cancel"); return 2

    # Settle
    node.pause(3.0)

    if node._is_charging:
        print("WARN: still charging after backup — robot may not be off dock")

    # Capture baselines before rotation
    wheel_start = node._wheel_yaw
    gyro_start = node._gyro_yaw
    print(f"\nStarting rotation test (wz={ROTATE_WZ} rad/s for {ROTATE_DURATION}s, expect +90°)")
    print(f"baseline wheel: {math.degrees(wheel_start):+.2f}°  gyro: {math.degrees(gyro_start):+.2f}°")

    node.drive(0.0, ROTATE_WZ, ROTATE_DURATION, "rotation 90")
    time.sleep(0.5)   # let final encoder packets arrive

    wheel_delta = math.degrees(node._wheel_yaw - wheel_start)
    gyro_delta = math.degrees(node._gyro_yaw - gyro_start)
    expected = math.degrees(ROTATE_WZ * ROTATE_DURATION)
    print(f"\n====== RESULT ======")
    print(f"Expected rotation:    {expected:+.2f}°")
    print(f"Wheel integrated:     {wheel_delta:+.2f}° → ratio = {wheel_delta/expected:.2f}x commanded")
    print(f"Gyro  integrated:     {gyro_delta:+.2f}° → ratio = {gyro_delta/expected:.2f}x commanded")
    print(f"====================")
    print("Compare with physical rotation observed.\n")

    # Stop and cancel recording
    for _ in range(10):
        node.publish(0, 0); time.sleep(PERIOD)
    node.call_hlc(HL_CMD_RECORD_CANCEL, "cancel recording")
    ex.shutdown(); node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    import sys; sys.exit(main())
