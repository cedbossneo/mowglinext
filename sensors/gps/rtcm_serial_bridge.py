#!/usr/bin/env python3
"""Bridge that subscribes to /rtcm and writes raw RTCM bytes to a serial port.

Uses raw file I/O (no pyserial open/close) to avoid disrupting the ublox
driver's ASIO connection on the same USB-CDC device.
"""
import os
import sys
import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message


class RtcmSerialBridge(Node):
    def __init__(self):
        super().__init__('rtcm_serial_bridge')
        self.declare_parameter('device', '/dev/gps')

        device = self.get_parameter('device').value

        try:
            self._fd = os.open(device, os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK)
            self.get_logger().info(
                f'Opened {device} (write-only) for RTCM forwarding')
        except Exception as e:
            self.get_logger().fatal(f'Cannot open {device}: {e}')
            sys.exit(1)

        self._sub = self.create_subscription(
            Message, '/rtcm', self._on_rtcm, 10)
        self._count = 0

    def _on_rtcm(self, msg: Message):
        try:
            os.write(self._fd, bytes(msg.message))
            self._count += 1
            if self._count % 100 == 1:
                self.get_logger().info(
                    f'Forwarded {self._count} RTCM messages to serial')
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')


def main():
    rclpy.init()
    node = RtcmSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
