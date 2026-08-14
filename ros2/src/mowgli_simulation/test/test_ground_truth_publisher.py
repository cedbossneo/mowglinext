# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

from pathlib import Path


def test_kinematic_drive_publishes_simulator_pose_truth() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / "mowgli_simulation"
        / "kinematic_drive.py"
    ).read_text()

    assert "PoseStamped, '/sim/ground_truth_pose'" in source
    assert "self.__gt_pose_pub.publish(gt)" in source
    assert "gt.header.frame_id = 'map'" in source
