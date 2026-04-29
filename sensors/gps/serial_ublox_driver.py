#!/usr/bin/env python3
"""Minimal ROS2 driver for a u-blox F9P over serial (kernel CDC ACM).

Replaces aussierobots/ublox_dgnss for the topics our stack actually
consumes. Bypasses libusb and its 3.5 s pipeline lag — header.stamp is
set at the wall-clock instant the serial byte is read.

Subscribes:
  /ntrip_client/rtcm      rtcm_msgs/Message     RTCM3 corrections from NTRIP

Publishes:
  /gps/fix                sensor_msgs/NavSatFix      built from NAV-HPPOSLLH + NAV-COV + NAV-STATUS
  /gps/fix_status_text    std_msgs/String            human-readable RTK status (debug)

Parameters (--ros-args):
  device          (string)  default '/dev/ttyACM1'
  frame_id        (string)  default 'gps'
  publish_pvt     (bool)    default false   — also republish UBXNavPVT (requires ublox_ubx_msgs)
"""
import struct
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

import serial

try:
    from rtcm_msgs.msg import Message as RtcmMessage
    HAVE_RTCM = True
except ImportError:
    HAVE_RTCM = False


UBX_NAV_HPPOSLLH = (0x01, 0x14)
UBX_NAV_COV = (0x01, 0x36)
UBX_NAV_STATUS = (0x01, 0x03)
UBX_NAV_PVT = (0x01, 0x07)


def ubx_checksum(cls, mid, payload):
    a = b = 0
    data = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def ned_to_enu_cov(c_ned):
    # c_ned = [Pnn, Pne, Pnd, Pen, Pee, Ped, Pdn, Pde, Pdd] (NED row-major 3x3)
    Pnn, Pne, Pnd, _, Pee, Ped, _, _, Pdd = c_ned
    # ENU = [E, N, U] = M * NED with M = [[0,1,0],[1,0,0],[0,0,-1]]
    Pee_e = Pee
    Pen_e = Pne
    Peu_e = -Ped
    Pnn_e = Pnn
    Pnu_e = -Pnd
    Puu_e = Pdd
    return [
        Pee_e, Pen_e, Peu_e,
        Pen_e, Pnn_e, Pnu_e,
        Peu_e, Pnu_e, Puu_e,
    ]


class SerialUbloxDriver(Node):
    def __init__(self):
        super().__init__("serial_ublox_driver")
        self.declare_parameter("device", "/dev/ttyACM1")
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("publish_pvt", False)

        self.device_path = self.get_parameter("device").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_pvt = self.get_parameter("publish_pvt").value

        # Reliable QoS to match what navsat_to_absolute_pose/ekf_map_node/rtabmap expect.
        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.fix_pub = self.create_publisher(NavSatFix, "/gps/fix", gps_qos)
        self.status_pub = self.create_publisher(String, "/gps/fix_status_text", 10)

        if HAVE_RTCM:
            self.create_subscription(
                RtcmMessage, "/ntrip_client/rtcm", self.cb_rtcm, 50)
        else:
            self.get_logger().warn("rtcm_msgs not available — NTRIP corrections disabled")

        self.ser = None
        self.ser_lock = threading.Lock()
        self.open_serial()

        # Latest aggregated state
        self.last_status_itow = None
        self.last_status_carr_soln = 0  # 0=none, 1=Float, 2=Fixed
        self.last_status_diff_soln = False

        self.last_cov_itow = None
        self.last_cov_pos_enu = None  # 9 elements

        self.bytes_in_total = 0
        self.frames_total = 0
        self.fix_pub_count = 0
        self.last_log_t = time.time()

        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

    def open_serial(self):
        for attempt in range(20):
            try:
                self.ser = serial.Serial(self.device_path, 9600, timeout=0.1)
                time.sleep(0.2)
                self.ser.reset_input_buffer()
                self.get_logger().info(f"opened {self.device_path}")
                return
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f"open {self.device_path} attempt {attempt+1}: {e}")
                time.sleep(1.0)
        self.get_logger().error("could not open serial after 20 attempts")
        raise RuntimeError("serial open failed")

    def cb_rtcm(self, msg):
        if not self.ser:
            return
        try:
            with self.ser_lock:
                self.ser.write(bytes(msg.message))
        except Exception as e:
            self.get_logger().warn(f"rtcm write: {e}")

    def reader_loop(self):
        buf = bytearray()
        while rclpy.ok():
            try:
                chunk = self.ser.read(2048)
            except Exception as e:
                self.get_logger().error(f"serial read: {e}")
                time.sleep(1.0)
                self.open_serial()
                continue
            recv_t_ns = time.time_ns()
            if not chunk:
                continue
            self.bytes_in_total += len(chunk)
            buf.extend(chunk)
            self.parse_buffer(buf, recv_t_ns)

            now = time.time()
            if now - self.last_log_t > 5.0:
                self.get_logger().info(
                    f"in={self.bytes_in_total}B frames={self.frames_total} fix_pubs={self.fix_pub_count}")
                self.last_log_t = now

    def parse_buffer(self, buf, recv_t_ns):
        while True:
            i = buf.find(b"\xb5\x62")
            if i < 0:
                if len(buf) > 8192:
                    del buf[:-8192]
                return
            if len(buf) < i + 8:
                if i > 0:
                    del buf[:i]
                return
            cls = buf[i + 2]
            mid = buf[i + 3]
            plen = struct.unpack_from("<H", buf, i + 4)[0]
            if len(buf) < i + 6 + plen + 2:
                if i > 0:
                    del buf[:i]
                return
            payload = bytes(buf[i + 6:i + 6 + plen])
            a_exp, b_exp = ubx_checksum(cls, mid, payload)
            a_got, b_got = buf[i + 6 + plen], buf[i + 6 + plen + 1]
            if a_exp == a_got and b_exp == b_got:
                self.frames_total += 1
                self.dispatch(cls, mid, payload, recv_t_ns)
                del buf[:i + 6 + plen + 2]
            else:
                del buf[:i + 2]

    def dispatch(self, cls, mid, payload, recv_t_ns):
        if (cls, mid) == UBX_NAV_STATUS:
            self.handle_nav_status(payload)
        elif (cls, mid) == UBX_NAV_COV:
            self.handle_nav_cov(payload)
        elif (cls, mid) == UBX_NAV_HPPOSLLH:
            self.handle_nav_hpposllh(payload, recv_t_ns)

    def handle_nav_status(self, payload):
        if len(payload) < 16:
            return
        itow, gps_fix, flags, fix_stat, flags2 = struct.unpack_from("<IBBBB", payload, 0)
        diff_soln = bool(flags & 0x02)
        carr_soln = (flags2 >> 6) & 0x03
        self.last_status_itow = itow
        self.last_status_diff_soln = diff_soln
        self.last_status_carr_soln = carr_soln

    def handle_nav_cov(self, payload):
        if len(payload) < 64:
            return
        itow, version, posCovValid, velCovValid = struct.unpack_from("<IBBB", payload, 0)
        # Skip 9 reserved bytes → pos cov starts at offset 16
        # 6 floats: posCovNN, posCovNE, posCovND, posCovEE, posCovED, posCovDD
        pn, pne, pnd, pe, ped, pd = struct.unpack_from("<6f", payload, 16)
        c_ned = [pn, pne, pnd, pne, pe, ped, pnd, ped, pd]
        self.last_cov_itow = itow
        self.last_cov_pos_enu = ned_to_enu_cov(c_ned)

    def handle_nav_hpposllh(self, payload, recv_t_ns):
        if len(payload) < 36:
            return
        # version(1) reserved(2) flags(1) iTOW(4) lon(4) lat(4) height(4) hMSL(4)
        # lonHp(1) latHp(1) heightHp(1) hMSLHp(1) hAcc(4) vAcc(4)
        version = payload[0]
        flags = payload[3]
        itow = struct.unpack_from("<I", payload, 4)[0]
        lon = struct.unpack_from("<i", payload, 8)[0]
        lat = struct.unpack_from("<i", payload, 12)[0]
        height = struct.unpack_from("<i", payload, 16)[0]
        # hmsl = struct.unpack_from("<i", payload, 20)[0]
        lonHp = struct.unpack_from("<b", payload, 24)[0]
        latHp = struct.unpack_from("<b", payload, 25)[0]
        heightHp = struct.unpack_from("<b", payload, 26)[0]
        # hMSLHp = payload[27]
        hAcc = struct.unpack_from("<I", payload, 28)[0]
        vAcc = struct.unpack_from("<I", payload, 32)[0]

        latitude = lat * 1e-7 + latHp * 1e-9
        longitude = lon * 1e-7 + lonHp * 1e-9
        altitude = height * 1e-3 + heightHp * 1e-4

        msg = NavSatFix()
        msg.header.frame_id = self.frame_id
        msg.header.stamp.sec = recv_t_ns // 1_000_000_000
        msg.header.stamp.nanosec = recv_t_ns % 1_000_000_000

        # NavSatStatus: 0 = STATUS_FIX, 1 = STATUS_SBAS_FIX, 2 = STATUS_GBAS_FIX
        status = NavSatStatus()
        if self.last_status_carr_soln == 2:
            status.status = NavSatStatus.STATUS_GBAS_FIX  # RTK Fixed
        elif self.last_status_diff_soln or self.last_status_carr_soln == 1:
            status.status = NavSatStatus.STATUS_SBAS_FIX
        else:
            status.status = NavSatStatus.STATUS_FIX
        status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS \
            | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        msg.status = status

        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude

        if self.last_cov_pos_enu is not None and self.last_cov_itow == itow:
            msg.position_covariance = self.last_cov_pos_enu
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        else:
            # Fall back to hAcc/vAcc when NAV-COV hasn't matched this epoch yet.
            sigma_h = hAcc * 1e-4  # hAcc is in 0.1 mm
            sigma_v = vAcc * 1e-4
            sxy = sigma_h * sigma_h
            sz = sigma_v * sigma_v
            msg.position_covariance = [sxy, 0.0, 0.0,
                                       0.0, sxy, 0.0,
                                       0.0, 0.0, sz]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.fix_pub.publish(msg)
        self.fix_pub_count += 1

        # Status text
        carr_name = {0: "no-carrier", 1: "Float", 2: "Fixed"}.get(self.last_status_carr_soln, "?")
        s = String()
        s.data = (f"{carr_name} hAcc={hAcc * 0.1:.1f}mm vAcc={vAcc * 0.1:.1f}mm "
                  f"itow={itow}")
        self.status_pub.publish(s)


def main():
    rclpy.init()
    node = SerialUbloxDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
