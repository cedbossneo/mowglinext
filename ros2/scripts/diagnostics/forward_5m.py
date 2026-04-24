#!/usr/bin/env python3
"""Simple forward 5m drive at 0.2 m/s. Reports fusion trajectory +
wheel/gyro integrated yaw so we can tell whether the robot went straight."""
import math, sys, threading, time
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


class Fwd5m(Node):
    def __init__(self):
        super().__init__("fwd_5m")
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
        self._fx = None; self._fy = None; self._fyaw = None
        self.create_subscription(HighLevelStatus, "/behavior_tree_node/high_level_status",
            lambda m: setattr(self, "_bt_state", int(m.state)), qos, callback_group=self._cb)
        self.create_subscription(HwStatus, "/hardware_bridge/status",
            lambda m: setattr(self, "_is_charging", bool(m.is_charging)), qos, callback_group=self._cb)
        self.create_subscription(Emergency, "/hardware_bridge/emergency",
            lambda m: setattr(self, "_emergency", bool(m.active_emergency or m.latched_emergency)),
            qos, callback_group=self._cb)
        self.create_subscription(Odometry, "/wheel_odom", self._wheel_cb, qos, callback_group=self._cb)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, qos_imu, callback_group=self._cb)
        self.create_subscription(Odometry, "/fusion/odom", self._fusion_cb, qos, callback_group=self._cb)

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

    def _fusion_cb(self, m):
        self._fx = m.pose.pose.position.x
        self._fy = m.pose.pose.position.y
        qz = m.pose.pose.orientation.z
        qw = m.pose.pose.orientation.w
        self._fyaw = 2.0 * math.atan2(qz, qw)

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
    DIST = 5.0           # m
    SPEED = 0.2          # m/s
    RAMP_SEC = 1.5       # s
    CRUISE_SEC = DIST / SPEED - RAMP_SEC  # account for ramp up; ramp down separate
    # So total distance ~ cruise + 2×ramp*0.5 = cruise + ramp
    # With cruise = 23.5s, ramp 1.5s, distance = 23.5*0.2 + 1.5*0.1 + 1.5*0.1 = 4.7 + 0.15 + 0.15 = 5.0m ✓

    rclpy.init()
    node = Fwd5m()
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
    print("In RECORDING. 3s baseline...")
    for _ in range(int(3 * RATE)):
        node.publish(0, 0); time.sleep(PERIOD)

    # Capture start pose
    fx0, fy0, fyaw0 = node._fx, node._fy, node._fyaw
    w_start, g_start = node._wheel_yaw, node._gyro_yaw
    print(f"\nstart pose: fusion=({fx0:+.3f}, {fy0:+.3f}) yaw={math.degrees(fyaw0):+.1f}°")
    print(f"Drive 5m forward at 0.2 m/s ({CRUISE_SEC+2*RAMP_SEC:.1f}s)\n")

    # ramp up
    ramp_n = max(1, int(RAMP_SEC * RATE))
    for i in range(ramp_n):
        if node._emergency: break
        node.publish(SPEED * (i+1)/ramp_n, 0); time.sleep(PERIOD)
    # cruise
    cruise_n = max(1, int(CRUISE_SEC * RATE))
    for _ in range(cruise_n):
        if node._emergency: break
        node.publish(SPEED, 0); time.sleep(PERIOD)
    # ramp down
    for i in range(ramp_n):
        if node._emergency: break
        node.publish(SPEED * (ramp_n-i-1)/ramp_n, 0); time.sleep(PERIOD)
    # settle
    for _ in range(int(2 * RATE)):
        node.publish(0, 0); time.sleep(PERIOD)

    # Capture end pose
    fx1, fy1, fyaw1 = node._fx, node._fy, node._fyaw
    w_delta_deg = math.degrees(node._wheel_yaw - w_start)
    g_delta_deg = math.degrees(node._gyro_yaw - g_start)
    disp_x = fx1 - fx0; disp_y = fy1 - fy0
    displacement = math.hypot(disp_x, disp_y)
    fyaw_delta = math.degrees(fyaw1 - fyaw0)

    # Direction of displacement vs initial yaw
    if displacement > 0.1:
        disp_angle = math.atan2(disp_y, disp_x)
        # Error = angle between robot's initial heading and actual travel direction
        err = math.degrees(disp_angle - fyaw0)
        # Normalize to [-180, 180]
        while err > 180: err -= 360
        while err < -180: err += 360
    else:
        err = 0.0

    print(f"\n====== RESULT ======")
    print(f"end pose:   fusion=({fx1:+.3f}, {fy1:+.3f}) yaw={math.degrees(fyaw1):+.1f}°")
    print(f"displacement    : ({disp_x:+.3f}, {disp_y:+.3f}), |Δ|={displacement:.3f}m (expected ~5.0m)")
    print(f"yaw drift (fusion)    : {fyaw_delta:+.1f}° (expected ~0)")
    print(f"yaw drift (wheel)     : {w_delta_deg:+.2f}°")
    print(f"yaw drift (gyro)      : {g_delta_deg:+.2f}°")
    print(f"direction error vs start yaw: {err:+.1f}° (0 = went perfectly straight)")
    print(f"====================\n")

    node.call_hlc(HL_CMD_RECORD_CANCEL, "cancel")
    ex.shutdown(); node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
