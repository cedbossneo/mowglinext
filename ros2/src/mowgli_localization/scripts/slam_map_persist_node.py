#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
slam_map_persist_node.py

Persists the current slam_toolbox graph + occupancy map to disk on every
transition OUT of HIGH_LEVEL_STATE_RECORDING. The serialised map is
auto-loaded by slam_toolbox at the next session start (lifelong mode),
enabling cross-session map refinement: the operator records each new
area while slam_toolbox builds a fresh map, the polygon is saved via
/map_server_node/add_area, and at the same time we checkpoint the slam
posegraph alongside it. Subsequent mowing sessions launch slam_toolbox
in lifelong mode against that checkpoint and refine it.

Trigger
-------
Subscribes to /behavior_tree_node/high_level_status (mowgli_interfaces/
HighLevelStatus). Detects the falling edge of `state == 3`
(HIGH_LEVEL_STATE_RECORDING). After a short settle delay (so the BT has
time to finish saving the polygon and any /slam_toolbox/serialize_map
call sees a stable graph), invokes the service.

Note on cancel
--------------
This node fires on both COMMAND_RECORD_FINISH (5) and
COMMAND_RECORD_CANCEL (6) — both transition the state out of RECORDING.
slam_toolbox accumulated lidar scans regardless of whether the polygon
was kept, and overwriting the previous checkpoint with more scan data
is generally harmless (lifelong refinement). If a future revision wants
cancel-aware behaviour (don't overwrite on cancel), gate this on a
separate event such as a successful /map_server_node/add_area service
return.

Failure modes
-------------
slam_toolbox's serialize_map service is asynchronous and can take a few
hundred ms on a large graph. We use a future-callback so this node does
not block its own callback group. If the service is unavailable
(slam_toolbox didn't start, or use_lidar is false) we log a one-shot
warning and skip — non-fatal.

Safety: pure observer node. Calls a single read-only-from-the-robot
service (slam_toolbox file IO). Does not touch drive commands, /tf, or
any safety topic.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from mowgli_interfaces.msg import HighLevelStatus

# slam_toolbox is an exec_depend; the service definition is generated at
# build time. Import is wrapped so this node is importable even when
# slam_toolbox is missing (e.g., use_lidar=false test runs) — it just
# can't actually serialise.
try:
    from slam_toolbox.srv import SerializePoseGraph
except ImportError:
    SerializePoseGraph = None  # type: ignore[assignment]


# CLAUDE.md HighLevelStatus.msg states — kept in a constant to make the
# trigger condition obvious in code review.
HIGH_LEVEL_STATE_RECORDING = 3


class SlamMapPersistNode(Node):
    """Auto-checkpoint slam_toolbox on RECORDING-exit."""

    def __init__(self) -> None:
        super().__init__("slam_map_persist_node")

        # Path passed to /slam_toolbox/serialize_map. slam_toolbox auto-
        # appends .posegraph and .data, so DO NOT include an extension.
        self._map_path = str(
            self.declare_parameter(
                "map_path", "/ros2_ws/maps/slam"
            ).value
        )
        # Settle delay after the RECORDING-exit edge before we call
        # serialize_map. Gives the BT time to finish add_area and any
        # other end-of-recording bookkeeping.
        self._settle_delay_sec = float(
            self.declare_parameter("settle_delay_sec", 1.0).value
        )
        # Topic the BT publishes its high-level status on. Default
        # matches the behavior_tree_node's relative `~/high_level_status`
        # publisher when launched with the standard node name.
        status_topic = str(
            self.declare_parameter(
                "status_topic",
                "/behavior_tree_node/high_level_status",
            ).value
        )
        # Service the slam_toolbox/async_slam_toolbox_node node exposes
        # when started with the node name "slam_toolbox".
        self._service_name = str(
            self.declare_parameter(
                "service_name",
                "/slam_toolbox/serialize_map",
            ).value
        )

        if SerializePoseGraph is None:
            self.get_logger().warn(
                "slam_toolbox.srv.SerializePoseGraph not importable — is "
                "slam_toolbox installed? slam_map_persist_node will "
                "still observe state transitions but cannot serialise."
            )

        self._prev_state: Optional[int] = None
        self._serialise_pending = False
        self._settle_timer = None

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            HighLevelStatus, status_topic, self._on_status, status_qos
        )

        self._client = (
            self.create_client(SerializePoseGraph, self._service_name)
            if SerializePoseGraph is not None
            else None
        )

        self.get_logger().info(
            "slam_map_persist_node ready: serialise %s -> %s on transition "
            "out of RECORDING (settle %.1fs)"
            % (self._service_name, self._map_path, self._settle_delay_sec)
        )

    # ------------------------------------------------------------------
    # State transition handling
    # ------------------------------------------------------------------

    def _on_status(self, msg: HighLevelStatus) -> None:
        state = msg.state
        prev = self._prev_state
        self._prev_state = state
        if prev is None:
            return
        if (
            prev == HIGH_LEVEL_STATE_RECORDING
            and state != HIGH_LEVEL_STATE_RECORDING
            and not self._serialise_pending
        ):
            self.get_logger().info(
                "RECORDING-exit detected (state %d -> %d, name=%s) — "
                "scheduling slam serialise in %.1fs"
                % (prev, state, msg.state_name, self._settle_delay_sec)
            )
            self._serialise_pending = True
            self._settle_timer = self.create_timer(
                self._settle_delay_sec, self._on_settle_elapsed
            )

    def _on_settle_elapsed(self) -> None:
        # One-shot semantics: cancel the timer immediately so it doesn't
        # re-fire while serialize_map is in flight.
        if self._settle_timer is not None:
            self._settle_timer.cancel()
            self._settle_timer = None
        self._call_serialise()

    # ------------------------------------------------------------------
    # Service call
    # ------------------------------------------------------------------

    def _call_serialise(self) -> None:
        if self._client is None or SerializePoseGraph is None:
            self.get_logger().warn(
                "Skipping serialise — slam_toolbox client unavailable."
            )
            self._serialise_pending = False
            return
        if not self._client.service_is_ready():
            ready = self._client.wait_for_service(timeout_sec=2.0)
            if not ready:
                self.get_logger().warn(
                    "Service %s not ready after 2s — skipping serialise."
                    % self._service_name
                )
                self._serialise_pending = False
                return

        request = SerializePoseGraph.Request()
        request.filename = self._map_path
        future = self._client.call_async(request)
        future.add_done_callback(self._on_serialise_done)
        self.get_logger().info(
            "Sent serialize_map request: filename=%s" % self._map_path
        )

    def _on_serialise_done(self, future) -> None:
        # Always clear the pending flag so the next RECORDING-exit can
        # re-trigger.
        self._serialise_pending = False
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 — service exceptions vary
            self.get_logger().error(
                "serialize_map call failed: %s" % exc
            )
            return
        if result is None:
            self.get_logger().error("serialize_map returned no result.")
            return
        # SerializePoseGraph.Response has a single int8 `result` field;
        # 0 = success in slam_toolbox's convention.
        if result.result == 0:
            self.get_logger().info(
                "Slam map persisted to %s.{posegraph,data}"
                % self._map_path
            )
        else:
            self.get_logger().error(
                "serialize_map returned non-zero (%d) — map not persisted."
                % result.result
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamMapPersistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
