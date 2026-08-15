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


def test_chassis_and_wheel_odom_share_one_actuation_model() -> None:
    package = Path(__file__).resolve().parents[1]
    source = (package / 'mowgli_simulation' / 'kinematic_drive.py').read_text()
    urdf = (package / 'urdf_webots' / 'mowgli_webots.urdf').read_text()

    assert '<cmdVelTopic>/cmd_vel_wheels</cmdVelTopic>' in urdf
    assert '<applyFirmwareModel>false</applyFirmwareModel>' in urdf
    assert "properties.get('applyFirmwareModel', 'true')" in source
    assert 'if self.__apply_firmware_model:' in source
    assert 'vx, wz = cmd_vx, cmd_wz' in source
def test_full_sim_gyro_uses_authoritative_achievable_twist() -> None:
    source_root = Path(__file__).resolve().parents[2]
    launch = (
        source_root
        / 'mowgli_bringup'
        / 'launch'
        / 'sim_full_system.launch.py'
    ).read_text()

    assert '"synthesize_from_cmd_vel": True' in launch
    assert '"cmd_vel_topic": "/cmd_vel_wheels"' in launch
