#!/usr/bin/env python3
# Publishes an empty OccupancyGrid covering the global costmap area on
# /no_lidar_static_map (transient_local, latched). Used by the no-lidar
# nav2 config so the global_costmap has a static_layer driving its
# `current_` flag — without it the costmap never reports current and
# the planner aborts every request with "Costmap timed out waiting for
# update" (Nav2 Kilted KeepoutFilter doesn't reliably set the flag).
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid


class EmptyStaticMapPub(Node):
    def __init__(self) -> None:
        super().__init__("empty_static_map_pub")
        size_m = float(self.declare_parameter("size_m", 200.0).value)
        resolution = float(self.declare_parameter("resolution", 0.5).value)
        topic = str(self.declare_parameter("topic", "/no_lidar_static_map").value)
        frame_id = str(self.declare_parameter("frame_id", "map").value)

        cells = max(1, int(math.ceil(size_m / resolution)))
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = resolution
        msg.info.width = cells
        msg.info.height = cells
        msg.info.origin.position.x = -size_m / 2.0
        msg.info.origin.position.y = -size_m / 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (cells * cells)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(OccupancyGrid, topic, qos)
        self._pub.publish(msg)
        self.get_logger().info(
            f"Published empty static map on {topic} ({cells}x{cells} @ {resolution} m)"
        )


def main() -> None:
    rclpy.init()
    node = EmptyStaticMapPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
