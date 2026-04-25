#!/usr/bin/env python3
# =============================================================================
# NMEA serial bridge — two jobs:
#
# 1. Read NMEA from /dev/gps, republish raw sentences on /nmea_sentence so
#    nmea_navsat_driver can convert them to NavSatFix.
#
# 2. Republish nmea_navsat_driver's NavSatFix on /gps/fix with the
#    status.status field corrected to reflect the GGA quality field.
#    nmea_navsat_driver's upstream behavior maps both Q=4 (RTK Fixed) and
#    Q=5 (RTK Float) to STATUS_GBAS_FIX (=2), losing the distinction
#    downstream consumers (FusionCore, navsat_to_absolute_pose, GUI) need
#    to gate Fixed-only logic. We track the last seen GGA quality here
#    and overwrite status.status before republishing.
#
# Why a separate bridge instead of nmea_navsat_driver's built-in serial
# reader?
# - We want the raw sentences on a topic (debugging, logging, future consumers).
# - We control QoS explicitly (RELIABLE) — Cyclone DDS rejects RELIABLE
#   subscribers paired with BEST_EFFORT publishers, and FusionCore /
#   navsat_to_absolute_pose subscribe RELIABLE. See sensors/gps/ublox_dgnss.yaml
#   for the same fix on the ublox side.
# - We add automatic reconnect on serial errors (USB hot-unplug, kernel
#   re-enumeration after str2str write bursts).
# - We need the GGA quality field (see point 2) which the upstream driver
#   silently collapses.
# =============================================================================

import threading
import time

import rclpy
import serial
from nmea_msgs.msg import Sentence
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus

RECONNECT_DELAY_S = 1.0

# NMEA-0183 GGA quality field → sensor_msgs/NavSatStatus.status mapping.
# This matches what navsat_to_absolute_pose_node.cpp expects:
#   STATUS_GBAS_FIX (2) → FLAG_GPS_RTK_FIXED
#   STATUS_SBAS_FIX (1) → FLAG_GPS_RTK_FLOAT
#   STATUS_FIX      (0) → FLAG_GPS_RTK   (generic)
#   STATUS_NO_FIX  (-1) → no flags
# nmea_navsat_driver upstream collapses both Q=4 and Q=5 to STATUS_GBAS_FIX,
# losing Float-vs-Fixed signal — we restore it.
GGA_QUALITY_TO_STATUS = {
    0: NavSatStatus.STATUS_NO_FIX,    # invalid / no fix
    1: NavSatStatus.STATUS_FIX,        # autonomous (SPS)
    2: NavSatStatus.STATUS_FIX,        # DGPS — has corrections but not RTK
    3: NavSatStatus.STATUS_FIX,        # PPS (rare)
    4: NavSatStatus.STATUS_GBAS_FIX,   # RTK Fixed → ~3 mm
    5: NavSatStatus.STATUS_SBAS_FIX,   # RTK Float → 0.5–2 m
    6: NavSatStatus.STATUS_FIX,        # estimated / dead reckoning
}


class NmeaSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("nmea_serial_bridge")
        self.declare_parameter("port", "/dev/gps")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "gps_link")

        self._port = self.get_parameter("port").value
        self._baud = int(self.get_parameter("baud").value)
        self._frame_id = self.get_parameter("frame_id").value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Sentence, "nmea_sentence", qos)

        # GGA quality tracking — updated from the serial reader, read by the
        # /gps/fix_raw subscriber. Default to 1 (autonomous) so a missing
        # GGA stream doesn't accidentally elevate any pre-existing fix to RTK.
        self._last_quality = 1
        self._quality_lock = threading.Lock()

        # NavSatFix passthrough: subscribe to nmea_navsat_driver's output
        # (remapped to /gps/fix_raw in start_gps_nmea.sh) and republish on
        # /gps/fix with status.status overwritten from the live GGA quality.
        self._fix_pub = self.create_publisher(NavSatFix, "/gps/fix", qos)
        self._fix_sub = self.create_subscription(
            NavSatFix, "/gps/fix_raw", self._on_fix_raw, qos
        )

        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    @staticmethod
    def _gga_quality(line: str) -> int | None:
        # GGA layout: $G[NP]GGA,utc,lat,N/S,lon,E/W,quality,...
        # Field index 6 is the quality digit (0..6/7/8 depending on receiver).
        if len(line) < 7 or line[3:6] != "GGA":
            return None
        parts = line.split(",")
        if len(parts) < 7:
            return None
        try:
            return int(parts[6])
        except ValueError:
            return None

    def _on_fix_raw(self, msg: NavSatFix) -> None:
        with self._quality_lock:
            q = self._last_quality
        msg.status.status = GGA_QUALITY_TO_STATUS.get(q, NavSatStatus.STATUS_FIX)
        self._fix_pub.publish(msg)

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                with serial.Serial(self._port, self._baud, timeout=1.0) as ser:
                    self.get_logger().info(
                        f"opened {self._port} @ {self._baud} baud, publishing /nmea_sentence"
                    )
                    while not self._stop.is_set():
                        raw = ser.readline()
                        if not raw:
                            continue
                        line = raw.decode("ascii", errors="replace").strip()
                        if not line.startswith("$"):
                            continue
                        # Track GGA quality for the NavSatFix republisher.
                        q = self._gga_quality(line)
                        if q is not None:
                            with self._quality_lock:
                                self._last_quality = q
                        msg = Sentence()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = self._frame_id
                        msg.sentence = line
                        self._pub.publish(msg)
            except serial.SerialException as e:
                self.get_logger().warning(
                    f"serial error on {self._port}: {e}; reconnecting in {RECONNECT_DELAY_S}s"
                )
                time.sleep(RECONNECT_DELAY_S)
            except OSError as e:
                self.get_logger().warning(
                    f"OS error opening {self._port}: {e}; reconnecting in {RECONNECT_DELAY_S}s"
                )
                time.sleep(RECONNECT_DELAY_S)

    def destroy_node(self) -> bool:
        self._stop.set()
        self._reader.join(timeout=2.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = NmeaSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
