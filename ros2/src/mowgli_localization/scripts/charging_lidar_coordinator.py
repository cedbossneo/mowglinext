#!/usr/bin/env python3
"""charging_lidar_coordinator.py

Issue #29 Layer 1: software-side CPU savings while charging on the dock.

Subscribes /hardware_bridge/status (mowgli_interfaces/msg/Status). On stable
is_charging=true (>= charging_debounce_sec), calls lifecycle DEACTIVATE on
ldlidar_node and on kinematic_icp_online_node so the LD19 serial driver and
the ICP main loop stop consuming CPU. On is_charging=false the activation is
immediate (no debounce — wake before undock).

Layer 2 (LD19 motor PWM control via Pi5 GPIO) is intentionally NOT here —
see the #29 issue body and the open follow-up. This node is software-only
and works on any install regardless of GPIO wiring.

Failsafe: if a lifecycle service is unavailable (use_lidar=false, the LiDAR
stack is not running, or a node hasn't reached active state yet) the call
is logged as a warning and skipped — the mower keeps working as today.
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from mowgli_interfaces.msg import Status


class ChargingLidarCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("charging_lidar_coordinator")

        self.declare_parameter("enabled", True)
        self.declare_parameter("charging_debounce_sec", 5.0)
        self.declare_parameter("status_topic", "/hardware_bridge/status")
        self.declare_parameter(
            "lidar_lifecycle_service", "/ldlidar_node/change_state")
        self.declare_parameter(
            "kicp_lifecycle_service",
            "/kinematic_icp/kinematic_icp_online_node/change_state")
        self.declare_parameter("service_call_timeout_sec", 2.0)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._debounce_sec = float(
            self.get_parameter("charging_debounce_sec").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._lidar_svc_name = str(
            self.get_parameter("lidar_lifecycle_service").value)
        self._kicp_svc_name = str(
            self.get_parameter("kicp_lifecycle_service").value)
        self._svc_timeout = float(
            self.get_parameter("service_call_timeout_sec").value)

        self._last_charging = False
        self._charging_since = None
        self._currently_paused = False

        self._lidar_client = self.create_client(
            ChangeState, self._lidar_svc_name)
        self._kicp_client = self.create_client(
            ChangeState, self._kicp_svc_name)

        self._status_sub = self.create_subscription(
            Status, self._status_topic, self._on_status, 10)

        if not self._enabled:
            self.get_logger().info(
                "ChargingLidarCoordinator disabled via parameter")
        else:
            self.get_logger().info(
                "ChargingLidarCoordinator active: debounce=%.1fs, "
                "lidar=%s, kicp=%s",
                self._debounce_sec,
                self._lidar_svc_name,
                self._kicp_svc_name,
            )

    def _on_status(self, msg: Status) -> None:
        if not self._enabled:
            return

        now = self.get_clock().now()
        is_charging = bool(msg.is_charging)

        if is_charging and not self._last_charging:
            self._charging_since = now
            self.get_logger().info(
                "Charging started — will pause LiDAR+K-ICP after %.1fs "
                "of stable charging",
                self._debounce_sec,
            )

        elif not is_charging and self._last_charging:
            self._charging_since = None
            if self._currently_paused:
                self._set_paused(False)

        elif (is_charging
              and self._charging_since is not None
              and not self._currently_paused):
            elapsed = (now - self._charging_since).nanoseconds / 1e9
            if elapsed >= self._debounce_sec:
                self._set_paused(True)
                self._charging_since = None

        self._last_charging = is_charging

    def _set_paused(self, pause: bool) -> None:
        transition_id = (Transition.TRANSITION_DEACTIVATE if pause
                         else Transition.TRANSITION_ACTIVATE)
        verb = "DEACTIVATE" if pause else "ACTIVATE"

        for client, label in (
            (self._lidar_client, "ldlidar_node"),
            (self._kicp_client, "kinematic_icp_online_node"),
        ):
            if not client.wait_for_service(timeout_sec=self._svc_timeout):
                self.get_logger().warning(
                    "%s lifecycle service unavailable — skipping %s",
                    label, verb,
                )
                continue
            req = ChangeState.Request()
            req.transition.id = transition_id
            future = client.call_async(req)
            future.add_done_callback(
                lambda fut, lbl=label, vrb=verb:
                self._on_change_done(fut, lbl, vrb))

        self._currently_paused = pause
        self.get_logger().info(
            "LiDAR+K-ICP %s requested (%s)",
            verb,
            "charging stable" if pause else "charging cleared",
        )

    def _on_change_done(
            self, future, node_label: str, verb: str) -> None:
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(
                "%s %s failed: %s", node_label, verb, exc)
            return
        if resp.success:
            self.get_logger().info("%s %s OK", node_label, verb)
        else:
            self.get_logger().warning(
                "%s %s returned success=false (likely already in target state)",
                node_label, verb,
            )


def main() -> None:
    rclpy.init()
    node = ChargingLidarCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
