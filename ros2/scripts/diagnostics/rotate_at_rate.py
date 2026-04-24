#!/usr/bin/env python3
"""Rotation test with adjustable wz and duration. Reports wheel+gyro delta.
Usage: diag_rotate_param.py WZ DURATION_SEC
Expected rotation = WZ * DURATION_SEC (in radians, converted to degrees)."""
import math
import sys
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


class RotNode(Node):
    def __init__(self):
        super().__init__("diag_rotate_param")
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
        self._bt_state = 0; self._is_charging = False; self._emergency = False
        self._wheel_yaw = 0.0; self._gyro_yaw = 0.0
        self._last_wheel_t = None; self._last_gyro_t = None
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
            if 0 < dt < 1: self._wheel_yaw += m.twist.twist.angular.z * dt
        self._last_wheel_t = t

    def _imu_cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self._last_gyro_t is not None:
            dt = t - self._last_gyro_t
            if 0 < dt < 1: self._gyro_yaw += m.angular_velocity.z * dt
        self._last_gyro_t = t

    def call_hlc(self, cmd, label):
        if not self._hlc.wait_for_service(timeout_sec=3.0): return False
        req = HighLevelControl.Request(); req.command = cmd
        fut = self._hlc.call_async(req)
        deadline = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < deadline: time.sleep(0.05)
        return fut.done() and fut.result() and fut.result().success

    def publish(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_footprint"
        m.twist.linear.x = float(vx); m.twist.angular.z = float(wz)
        self._pub.publish(m)


def main():
    if len(sys.argv) != 3:
        print("usage: diag_rotate_param.py WZ DURATION_SEC"); return 2
    WZ = float(sys.argv[1])
    DURATION = float(sys.argv[2])

    rclpy.init()
    node = RotNode()
    ex = MultiThreadedExecutor(); ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.5)

    if node._emergency: print("ABORT: emergency"); return 1
    if node._is_charging: print("ABORT: charging, undock first"); return 1

    if node._bt_state != HL_STATE_RECORDING:
        if not node.call_hlc(HL_CMD_RECORD_AREA, "enter RECORDING"): return 1
        for _ in range(50):
            if node._bt_state == HL_STATE_RECORDING: break
            time.sleep(0.1)
    print("In RECORDING. Waiting 2s baseline...")
    for _ in range(int(2 * RATE)):
        node.publish(0, 0); time.sleep(PERIOD)

    w_start = node._wheel_yaw; g_start = node._gyro_yaw
    expected = math.degrees(WZ * DURATION)
    print(f"\nRotation: wz={WZ:+.3f} rad/s for {DURATION:.3f}s → expect {expected:+.2f}°\n")
    n = int(DURATION * RATE)
    for _ in range(n):
        if node._emergency: break
        node.publish(0, WZ); time.sleep(PERIOD)
    for _ in range(10):
        node.publish(0, 0); time.sleep(PERIOD)
    time.sleep(0.5)

    wd = math.degrees(node._wheel_yaw - w_start)
    gd = math.degrees(node._gyro_yaw - g_start)
    print(f"====== RESULT (wz={WZ}) ======")
    print(f"Expected:  {expected:+.2f}°")
    print(f"Wheel:     {wd:+.2f}°  → ratio = {wd/expected:.2f}x")
    print(f"Gyro:      {gd:+.2f}°  → ratio = {gd/expected:.2f}x")
    print(f"==========================\n")

    node.call_hlc(HL_CMD_RECORD_CANCEL, "cancel")
    ex.shutdown(); node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
