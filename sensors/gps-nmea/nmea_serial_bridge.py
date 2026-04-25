#!/usr/bin/env python3
# =============================================================================
# NMEA serial bridge: reads NMEA sentences from /dev/gps and republishes them
# as nmea_msgs/Sentence on /nmea_sentence for downstream consumers
# (nmea_navsat_driver's nmea_topic_driver).
#
# Why a separate bridge instead of nmea_navsat_driver's built-in nmea_serial_driver?
# - We want the raw sentences on a topic (debugging, logging, future consumers).
# - We control QoS explicitly (RELIABLE) — Cyclone DDS rejects RELIABLE
#   subscribers paired with BEST_EFFORT publishers, and FusionCore /
#   navsat_to_absolute_pose subscribe RELIABLE. See sensors/gps/ublox_dgnss.yaml
#   for the same fix on the ublox side.
# - We add automatic reconnect on serial errors (USB hot-unplug, kernel
#   re-enumeration after str2str write bursts).
# =============================================================================

import threading
import time

import rclpy
import serial
from nmea_msgs.msg import Sentence
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

RECONNECT_DELAY_S = 1.0


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

        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

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
