import os
import pty
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


REQUIRED_PUBLIC_TOPICS = {
    "/imu/data",
    "/wheel_odom",
    "/hardware_bridge/status",
    "/hardware_bridge/power",
    "/hardware_bridge/emergency",
    "/cmd_vel",
}


def generate_backend_test_description(backend: str):
    bringup_dir = get_package_share_directory("mowgli_bringup")
    launch_path = os.path.join(bringup_dir, "launch", "mowgli.launch.py")

    state = {"master_fd": None, "slave_fd": None, "serial_port": "/dev/null"}

    def launch_setup(_context, *args, **kwargs):
        if backend == "mowgli":
            master_fd, slave_fd = pty.openpty()
            state["master_fd"] = master_fd
            state["slave_fd"] = slave_fd
            state["serial_port"] = os.ttyname(slave_fd)

        include = launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={
                "use_sim_time": "false",
                "hardware_backend": backend,
                "serial_port": state["serial_port"],
            }.items(),
        )
        return [include, launch_testing.actions.ReadyToTest()]

    return (
        launch.LaunchDescription([OpaqueFunction(function=launch_setup)]),
        {"backend": backend, "pty_state": state},
    )


class BackendTopicsTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("backend_topics_test_helper")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_expected_public_topics(self, backend) -> None:
        deadline = time.monotonic() + 10.0
        found: set[str] = set()

        while time.monotonic() < deadline and found != REQUIRED_PUBLIC_TOPICS:
            topic_names_and_types = self.node.get_topic_names_and_types()
            found = {name for name, _ in topic_names_and_types} & REQUIRED_PUBLIC_TOPICS
            if found != REQUIRED_PUBLIC_TOPICS:
                time.sleep(0.25)

        missing = REQUIRED_PUBLIC_TOPICS - found
        self.assertFalse(
            missing,
            msg=f"Backend '{backend}' did not expose the expected public topics within 10 s: {missing}",
        )
