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
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

# import launch_testing.markers
from launch_testing_ros import WaitForTopics
from controller_manager.controller_manager_services import list_controllers
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Executes the given launch file and checks if all nodes can be started
@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros2_control_demo_example_2"),
                "launch/diffbot.launch.py",
            )
        ),
        launch_arguments={"gui": "true"}.items(),
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
        start = time.time()
        found = False
        while time.time() - start < 2.0 and not found:
            found = "robot_state_publisher" in self.node.get_node_names()
            time.sleep(0.1)
        assert found, "robot_state_publisher not found!"

    def test_controller_running(self, proc_output):

        cname = "diffbot_base_controller"

        start = time.time()
        found = False
        while time.time() - start < 10.0 and not found:
            controllers = list_controllers(self.node, "controller_manager", 5.0).controller
            assert controllers, "No controllers found!"
            for c in controllers:
                if c.name == cname and c.state == "active":
                    found = True
                    break
        assert found, f"{cname} not found!"

    def test_check_if_msgs_published(self):
        wait_for_topics = WaitForTopics([("/joint_states", JointState)], timeout=15.0)
        assert wait_for_topics.wait(), "Topic '/joint_states' not found!"
        msgs = wait_for_topics.received_messages("/joint_states")
        msg = msgs[0]
        assert len(msg.name) == 2, "Wrong number of joints in message"
        assert msg.name == ["left_wheel_joint", "right_wheel_joint"], "Wrong joint names"
        wait_for_topics.shutdown()


# TODO(anyone): enable this if shutdown of ros2_control_node does not fail anymore
# @launch_testing.post_shutdown_test()
# # These tests are run after the processes in generate_test_description() have shutdown.
# class TestDescriptionCraneShutdown(unittest.TestCase):

#     def test_exit_codes(self, proc_info):
#         """Check if the processes exited normally."""
#         launch_testing.asserts.assertExitCodes(proc_info)
