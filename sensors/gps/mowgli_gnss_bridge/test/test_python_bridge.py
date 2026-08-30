# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""Contract-parity tests for the production Python fallback bridge."""

from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from universal_gnss_topic_bridge import (  # noqa: E402
    UniversalGnssStatus,
    UniversalGnssTopicBridge,
)


class _CapturePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _make_bridge() -> UniversalGnssTopicBridge:
    bridge = object.__new__(UniversalGnssTopicBridge)
    bridge._frame_id = "gps_link"
    bridge._backend = "universal"
    bridge._receiver_vendor = "u-blox"
    bridge._diagnostic_entries = {}
    bridge._status_pub = _CapturePublisher()
    return bridge


class PythonBridgeContractTest(unittest.TestCase):
    def test_receipt_stamp_and_sequence_match_cpp_contract_vector(self) -> None:
        bridge = _make_bridge()
        incoming = UniversalGnssStatus()
        incoming.fix_type = UniversalGnssStatus.FIX_TYPE_RTK_FIXED
        incoming.fix_valid = True
        incoming.stamp.sec = 123
        incoming.stamp.nanosec = 456
        incoming.position_observation_sequence = 41

        bridge._on_status(incoming)

        public = bridge._status_pub.messages[-1]
        self.assertEqual(public.header.stamp.sec, 123)
        self.assertEqual(public.header.stamp.nanosec, 456)
        self.assertEqual(public.position_observation_sequence, 41)
        self.assertEqual(public.header.frame_id, "gps_link")

    def test_cached_and_genuine_identical_observation_sequences_are_preserved(self) -> None:
        bridge = _make_bridge()
        incoming = UniversalGnssStatus()
        incoming.fix_type = UniversalGnssStatus.FIX_TYPE_RTK_FIXED
        incoming.fix_valid = True
        incoming.stamp.sec = 200
        incoming.stamp.nanosec = 300
        incoming.position_observation_sequence = 700

        bridge._on_status(incoming)
        bridge._on_status(incoming)
        incoming.position_observation_sequence = 701
        bridge._on_status(incoming)

        self.assertEqual(
            [msg.position_observation_sequence for msg in bridge._status_pub.messages],
            [700, 700, 701],
        )


if __name__ == "__main__":
    unittest.main()
