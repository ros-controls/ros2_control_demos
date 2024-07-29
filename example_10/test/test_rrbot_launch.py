# Copyright (c) 2024 AIT - Austrian Institute of Technology GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Christoph Froehlich

import os
import pytest
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

import launch_testing
import rclpy
from rclpy.node import Node
from ros2_control_demo_testing.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running,
)


# This function specifies the processes to be run for our test
# The ReadyToTest action waits for 15 second by default for the processes to
# start, if processes take more time an error is thrown. We use decorator here
# to provide timeout duration of 20 second so that processes that take longer than
# 15 seconds can start up.
@pytest.mark.launch_test
@launch_testing.ready_to_test_action_timeout(20)
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros2_control_demo_example_10"),
                "launch/rrbot.launch.py",
            )
        ),
        launch_arguments={"gui": "false"}.items(),
    )

    return LaunchDescription([launch_include, ReadyToTest()])


# This is our test fixture. Each method is a test case.
# These run alongside the processes specified in generate_test_description()
class TestFixture(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = Node("test_node")

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_start(self, proc_output):
        check_node_running(self.node, "robot_state_publisher")

    def test_controller_running(self, proc_output):

        cnames = ["forward_position_controller", "gpio_controller", "joint_state_broadcaster"]

        check_controllers_running(self.node, cnames)

    def test_check_if_msgs_published(self):
        check_if_js_published("/joint_states", ["joint1", "joint2"])


# TODO(anyone): enable this if shutdown of ros2_control_node does not fail anymore
# @launch_testing.post_shutdown_test()
# # These tests are run after the processes in generate_test_description() have shutdown.
# class TestDescriptionCraneShutdown(unittest.TestCase):

#     def test_exit_codes(self, proc_info):
#         """Check if the processes exited normally."""
#         launch_testing.asserts.assertExitCodes(proc_info)
