#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Lightweight gate script that exits 0 as soon as a TF transform becomes
available, or exits 1 on timeout. Used in launch files to replace fixed
TimerAction delays with event-driven startup.

Usage:
    wait_for_tf.py --parent map --child odom --timeout 120
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TFWaiter(Node):
    def __init__(self, parent: str, child: str, timeout: float):
        super().__init__("tf_waiter")
        self._parent = parent
        self._child = child
        self._timeout = timeout
        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self._start = self.get_clock().now()
        self._timer = self.create_timer(0.5, self._check)
        self.get_logger().info(
            f"Waiting for TF {parent} -> {child} (timeout {timeout}s)..."
        )

    def _check(self):
        if self._buf.can_transform(self._parent, self._child, rclpy.time.Time()):
            elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
            self.get_logger().info(
                f"TF {self._parent} -> {self._child} available after {elapsed:.1f}s"
            )
            raise SystemExit(0)

        elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed > self._timeout:
            self.get_logger().warn(
                f"Timeout waiting for TF {self._parent} -> {self._child}"
            )
            raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--parent", default="map")
    parser.add_argument("--child", default="odom")
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = TFWaiter(args.parent, args.child, args.timeout)
    try:
        rclpy.spin(node)
    except SystemExit as e:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(e.code)


if __name__ == "__main__":
    main()
