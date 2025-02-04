# Copyright 2025 ros2_control Development Team
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
import pytest
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

# import launch_testing.markers
import rclpy
from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running,
)


# Executes the given launch file and checks if all nodes can be started
@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros2_control_demo_example_16"),
                "launch/diffbot.launch.py",
            )
        ),
        launch_arguments={"gui": "False"}.items(),
    )

    return LaunchDescription([launch_include, ReadyToTest()])


# This is our test fixture. Each method is a test case.
# These run alongside the processes specified in generate_test_description()
class TestFixture(unittest.TestCase):
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
        # disable this test for now, as the activation of the diffbot_base_controller fails
        cnames = [
            "pid_controller_left_wheel_joint",
            "pid_controller_right_wheel_joint",
            "diffbot_base_controller",
            "joint_state_broadcaster",
        ]

        check_controllers_running(self.node, cnames)

    def test_check_if_msgs_published(self):
        check_if_js_published("/joint_states", ["left_wheel_joint", "right_wheel_joint"])


# TODO(anyone): enable this if shutdown of ros2_control_node does not fail anymore
# @launch_testing.post_shutdown_test()
# # These tests are run after the processes in generate_test_description() have shutdown.
# class TestDescriptionCraneShutdown(unittest.TestCase):

#     def test_exit_codes(self, proc_info):
#         """Check if the processes exited normally."""
#         launch_testing.asserts.assertExitCodes(proc_info)
