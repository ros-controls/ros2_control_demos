# Copyright 2026 ros2_control Development Team
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

import unittest

import pytest

from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.markers
import rclpy


@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("ros2_control_demo_example_1"),
                "launch",
                "rrbot.launch.py",
            ]
        ),
        launch_arguments={"gui": "False"}.items(),
    )

    return LaunchDescription([launch_include, ReadyToTest()])


class TestMovement(unittest.TestCase):

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

    def test_node_start(self, proc_output):
        check_node_running(self.node, "robot_state_publisher")

    def test_controller_running(self, proc_output):
        cnames = ["forward_position_controller", "joint_state_broadcaster"]
        check_controllers_running(self.node, cnames)

    def test_check_if_msgs_published(self):
        check_if_js_published("/joint_states", ["joint1", "joint2"])

    def test_movement(self, launch_service, proc_info, proc_output):
        proc_action = Node(
            package="ros2_control_demo_test_utils",
            executable="test_forward_command",
            output="screen",
        )

        with launch_testing.tools.launch_process(
            launch_service, proc_action, proc_info, proc_output
        ):
            proc_info.assertWaitForShutdown(process=proc_action, timeout=60)
            launch_testing.asserts.assertExitCodes(
                proc_info, process=proc_action, allowable_exit_codes=[0]
            )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
