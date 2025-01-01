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
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

# import launch_testing.markers
import rclpy
from controller_manager.test_utils import check_controllers_running


# Executes the given launch file and checks if all nodes can be started
@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros2_control_demo_example_1"),
                "launch/rrbot.launch.py",
            )
        ),
        launch_arguments={"gui": "False"}.items(),
    )

    return LaunchDescription([launch_include, ReadyToTest()])


class TestFixtureCliDirect(unittest.TestCase):
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

    def test_main(self, proc_output):

        # Command to run the CLI
        cname = "joint_trajectory_position_controller"
        command = [
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            cname,
            os.path.join(
                get_package_share_directory("ros2_control_demo_example_1"),
                "config/rrbot_jtc.yaml",
            ),
        ]
        subprocess.run(command, check=True)
        check_controllers_running(self.node, [cname], state="inactive")
        check_controllers_running(self.node, ["forward_position_controller"], state="active")

        command = [
            "ros2",
            "control",
            "switch_controllers",
            "--activate",
            cname,
            "--deactivate",
            "forward_position_controller",
        ]
        subprocess.run(command, check=True)
        check_controllers_running(self.node, ["forward_position_controller"], state="inactive")
        check_controllers_running(self.node, [cname], state="active")


# TODO(anyone): enable this if shutdown of ros2_control_node does not fail anymore
# @launch_testing.post_shutdown_test()
# # These tests are run after the processes in generate_test_description() have shutdown.
# class TestDescriptionCraneShutdown(unittest.TestCase):

#     def test_exit_codes(self, proc_info):
#         """Check if the processes exited normally."""
#         launch_testing.asserts.assertExitCodes(proc_info)
