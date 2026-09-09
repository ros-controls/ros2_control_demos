# Copyright (c) 2026 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import unittest

import pytest
import rclpy
import launch_testing.markers
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running,
)


# Test: controllers come up (cartesian_motion reaching active exercises the kinematics-plugin
# load + configure on the r6bot chain) and joint_states are published.
@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros2_control_demo_example_20"),
                "bringup/launch/cartesian_demo.launch.py",
            )
        ),
        launch_arguments={"gui": "False", "run_policy": "False"}.items(),
    )
    return LaunchDescription([launch_include, ReadyToTest()])


class TestCartesianDemo(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_node_start(self):
        check_node_running(self.node, "robot_state_publisher")

    def test_controllers_active(self, proc_info):
        cnames = ["joint_state_broadcaster", "cartesian_motion"]
        check_controllers_running(self.node, cnames, state="active")

        for cname in cnames:
            proc_info.assertWaitForShutdown(process="spawner", cmd_args=cname, timeout=30)
            launch_testing.asserts.assertExitCodes(proc_info, process="spawner", cmd_args=cname)

        check_controllers_running(self.node, cnames, state="active")

    def test_joint_states_published(self):
        check_if_js_published(
            "/joint_states",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check that all processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
