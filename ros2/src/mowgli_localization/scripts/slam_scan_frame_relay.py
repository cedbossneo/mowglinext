#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
slam_scan_frame_relay.py

Bridges the LiDAR scan onto the isolated wheel-odom TF tree that
slam_toolbox consumes. (Originally written for Kinematic-ICP; the same
parallel-tree decoupling now serves slam_toolbox after K-ICP was
removed.)

  Real URDF tree:
      map (ekf_map_node) -> odom (ekf_odom_node) -> base_footprint -> base_link -> lidar_link
  Parallel isolated tree (slam_toolbox + slam_pose_anchor_node):
      map -> map_slam (slam_pose_anchor_node EWMA-aligns to GPS)
                  │
                  └─> wheel_odom_raw (slam_toolbox publishes map_slam->wheel_odom_raw,
                                       wheel_odom_tf_node publishes wheel_odom_raw->base_footprint_wheels)
                          │
                          └─> base_footprint_wheels
                                       │
                                       └─> lidar_link_wheels (published HERE, static)

slam_toolbox needs BOTH a motion-prior TF (wheel_odom_raw -> base_footprint_wheels)
AND the sensor extrinsic (base_footprint_wheels -> lidar_link_wheels). TF
permits only one parent per frame, so we can't graft the real lidar_link
onto the parallel tree — we must publish a SECOND lidar frame
(`lidar_link_wheels`) rooted in the parallel tree.

This node does two things:

  1. On startup, waits for the real URDF transform `base_footprint -> lidar_link`
     to become available, then rebroadcasts it as the static
     `base_footprint_wheels -> lidar_link_wheels` so the parallel tree has
     the correct sensor extrinsic baked in.

  2. Subscribes to /scan (the real LaserScan, header.frame_id=lidar_link)
     and republishes each message on /scan_slam with header.frame_id
     rewritten to "lidar_link_wheels". slam_toolbox looks up
     `base_footprint_wheels -> lidar_link_wheels` for each scan and finds
     it entirely within the parallel tree.

Safety: read-only relay. Does not touch drive commands, /tf (except
the one static TF on the parallel tree), or any safety topic.
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


class SlamScanFrameRelay(Node):
    def __init__(self) -> None:
        super().__init__("slam_scan_frame_relay")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_slam")
        self.declare_parameter("input_sensor_frame", "lidar_link")
        self.declare_parameter("output_sensor_frame", "lidar_link_wheels")
        self.declare_parameter("real_base_frame", "base_footprint")
        self.declare_parameter("wheels_base_frame", "base_footprint_wheels")
        self.declare_parameter("extrinsic_timeout_sec", 30.0)

        self._in_topic = self.get_parameter("input_topic").value
        self._out_topic = self.get_parameter("output_topic").value
        self._in_frame = self.get_parameter("input_sensor_frame").value
        self._out_frame = self.get_parameter("output_sensor_frame").value
        self._real_base = self.get_parameter("real_base_frame").value
        self._wheels_base = self.get_parameter("wheels_base_frame").value
        self._extrinsic_timeout = float(
            self.get_parameter("extrinsic_timeout_sec").value
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_static = StaticTransformBroadcaster(self)

        # Defer scan pass-through until the static extrinsic has been
        # mirrored onto the parallel tree.
        self._extrinsic_ready = False
        self._pub: Optional[rclpy.publisher.Publisher] = None

        self._pub = self.create_publisher(LaserScan, self._out_topic, sensor_qos)
        self._sub = self.create_subscription(
            LaserScan, self._in_topic, self._on_scan, sensor_qos
        )

        # Poll for the real extrinsic every 0.5 s until it's there.
        self._extrinsic_timer = self.create_timer(0.5, self._try_publish_extrinsic)
        self._time_started = self.get_clock().now()

        self.get_logger().info(
            f"slam_scan_frame_relay ready:\n"
            f"  scan:   {self._in_topic} ({self._in_frame}) -> {self._out_topic} ({self._out_frame})\n"
            f"  static: {self._real_base} -> {self._in_frame}  =>  "
            f"{self._wheels_base} -> {self._out_frame}"
        )

    def _try_publish_extrinsic(self) -> None:
        if self._extrinsic_ready:
            return
        if self._tf_buffer.can_transform(
            self._real_base, self._in_frame, rclpy.time.Time()
        ):
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._real_base, self._in_frame, rclpy.time.Time()
                )
            except Exception as e:  # noqa: BLE001 — tf2 raises a handful of subclasses
                self.get_logger().warn(f"TF lookup failed: {e}")
                return

            out = TransformStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self._wheels_base
            out.child_frame_id = self._out_frame
            out.transform = tf.transform
            self._tf_static.sendTransform(out)

            self._extrinsic_ready = True
            self.get_logger().info(
                f"Mirrored sensor extrinsic: "
                f"{self._wheels_base} -> {self._out_frame}  "
                f"xyz=({out.transform.translation.x:.3f},"
                f"{out.transform.translation.y:.3f},"
                f"{out.transform.translation.z:.3f})"
            )
            self._extrinsic_timer.cancel()
            return

        elapsed = (self.get_clock().now() - self._time_started).nanoseconds * 1e-9
        if elapsed > self._extrinsic_timeout:
            self.get_logger().error(
                f"Timed out waiting for real extrinsic {self._real_base} -> "
                f"{self._in_frame} after {elapsed:.1f}s — scan relay is "
                f"disabled until TF arrives."
            )

    def _on_scan(self, msg: LaserScan) -> None:
        if not self._extrinsic_ready:
            return
        # LaserScan is an immutable-ish message; we just swap the frame_id.
        out = LaserScan(
            header=msg.header,
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=msg.range_min,
            range_max=msg.range_max,
            ranges=msg.ranges,
            intensities=msg.intensities,
        )
        out.header.frame_id = self._out_frame
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamScanFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
