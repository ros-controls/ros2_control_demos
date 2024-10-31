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
from rclpy.node import Node
from ros2_control_demo_testing.test_utils import (
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
                get_package_share_directory("ros2_control_demo_example_13"),
                "launch/three_robots.launch.py",
            )
        ),
        launch_arguments={"gui": "false"}.items(),
    )

    return LaunchDescription([launch_include, ReadyToTest()])


# This is our first test fixture. Each method is a test case.
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

    def test_behavior(self, proc_output):

        # --- initial configuration ---
        cnames = [
            "joint_state_broadcaster",
            "rrbot_external_fts_broadcaster",
            "rrbot_position_controller",
            "rrbot_with_sensor_joint_state_broadcaster",
            "rrbot_with_sensor_fts_broadcaster",
        ]

        check_controllers_running(self.node, cnames)
        check_if_js_published(
            "/joint_states",
            [
                "rrbot_with_sensor_joint1",
                "rrbot_joint1",
                "rrbot_with_sensor_joint2",
                "rrbot_joint2",
            ],
        )

        # --- activate second robot with its controllers ---
        # Command to activate the second robot with its controllers
        subprocess.run(
            [
                "ros2",
                "control",
                "set_hardware_component_state",
                "RRBotSystemWithSensor",
                "active",
            ]
        )
        subprocess.run(
            [
                "ros2",
                "control",
                "switch_controllers",
                "--activate",
                "rrbot_with_sensor_position_controller",
            ]
        )
        cnames = [
            "joint_state_broadcaster",
            "rrbot_external_fts_broadcaster",
            "rrbot_joint_state_broadcaster",
            "rrbot_position_controller",
            "rrbot_with_sensor_joint_state_broadcaster",
            "rrbot_with_sensor_fts_broadcaster",
            "rrbot_with_sensor_position_controller",
        ]
        check_controllers_running(self.node, cnames)
        # still the same joint_states
        check_if_js_published(
            "/joint_states",
            [
                "rrbot_with_sensor_joint1",
                "rrbot_joint1",
                "rrbot_with_sensor_joint2",
                "rrbot_joint2",
            ],
        )

        # --- configure FakeThreeDofBot its controllers ---
        subprocess.run(
            [
                "ros2",
                "control",
                "set_hardware_component_state",
                "FakeThreeDofBot",
                "inactive",
            ]
        )
        subprocess.run(
            [
                "ros2",
                "control",
                "switch_controllers",
                "--activate",
                "threedofbot_joint_state_broadcaster",
                "threedofbot_pid_gain_controller",
            ]
        )
        cnames = [
            "joint_state_broadcaster",
            "rrbot_external_fts_broadcaster",
            "rrbot_joint_state_broadcaster",
            "rrbot_position_controller",
            "rrbot_with_sensor_joint_state_broadcaster",
            "rrbot_with_sensor_fts_broadcaster",
            "rrbot_with_sensor_position_controller",
            "threedofbot_joint_state_broadcaster",
            "threedofbot_pid_gain_controller",
        ]
        check_controllers_running(self.node, cnames)
        # still the same joint_states
        check_if_js_published(
            "/joint_states",
            [
                "rrbot_with_sensor_joint1",
                "rrbot_joint1",
                "rrbot_with_sensor_joint2",
                "rrbot_joint2",
            ],
        )

        # --- restart global joint state broadcaster ---
        subprocess.run(
            [
                "ros2",
                "control",
                "switch_controllers",
                "--deactivate",
                "joint_state_broadcaster",
            ]
        )
        subprocess.run(
            [
                "ros2",
                "control",
                "switch_controllers",
                "--activate",
                "joint_state_broadcaster",
            ]
        )
        # now the joint_states of threedofbot and rrbot_with_sensor are broadcasted, too.
        check_if_js_published(
            "/joint_states",
            [
                "rrbot_with_sensor_joint1",
                "rrbot_joint1",
                "rrbot_with_sensor_joint2",
                "rrbot_joint2",
                "threedofbot_joint1",
                "threedofbot_joint2",
                "threedofbot_joint3",
            ],
        )

        # --- activate FakeThreeDofBot and its controllers ---
        subprocess.run(
            [
                "ros2",
                "control",
                "set_hardware_component_state",
                "FakeThreeDofBot",
                "active",
            ]
        )
        subprocess.run(
            [
                "ros2",
                "control",
                "switch_controllers",
                "--activate",
                "threedofbot_position_controller",
            ]
        )
        cnames = [
            "joint_state_broadcaster",
            "rrbot_external_fts_broadcaster",
            "rrbot_joint_state_broadcaster",
            "rrbot_position_controller",
            "rrbot_with_sensor_joint_state_broadcaster",
            "rrbot_with_sensor_fts_broadcaster",
            "rrbot_with_sensor_position_controller",
            "threedofbot_joint_state_broadcaster",
            "threedofbot_pid_gain_controller",
            "threedofbot_position_controller",
        ]
        check_controllers_running(self.node, cnames)

        # --- deactivate RRBotSystemPositionOnly and its controllers ---
        subprocess.run(
            [
                "ros2",
                "control",
                "switch_controllers",
                "--deactivate",
                "rrbot_position_controller",
            ]
        )
        subprocess.run(
            [
                "ros2",
                "control",
                "set_hardware_component_state",
                "RRBotSystemPositionOnly",
                "inactive",
            ]
        )
        cnames = [
            "joint_state_broadcaster",
            "rrbot_external_fts_broadcaster",
            "rrbot_joint_state_broadcaster",
            "rrbot_with_sensor_joint_state_broadcaster",
            "rrbot_with_sensor_fts_broadcaster",
            "rrbot_with_sensor_position_controller",
            "threedofbot_joint_state_broadcaster",
            "threedofbot_pid_gain_controller",
            "threedofbot_position_controller",
        ]
        check_controllers_running(self.node, cnames)

        # --- Set RRBotSystemPositionOnly in unconfigured state, and deactivate its joint state broadcaster. Also restart global joint state broadcaster.  ---
        subprocess.run(
            [
                "ros2",
                "control",
                "switch_controllers",
                "--deactivate",
                "rrbot_joint_state_broadcaster",
                "joint_state_broadcaster",
            ]
        )
        subprocess.run(
            [
                "ros2",
                "control",
                "set_hardware_component_state",
                "RRBotSystemPositionOnly",
                "unconfigured",
            ]
        )
        subprocess.run(
            [
                "ros2",
                "control",
                "switch_controllers",
                "--activate",
                "joint_state_broadcaster",
            ]
        )
        cnames = [
            "joint_state_broadcaster",
            "rrbot_external_fts_broadcaster",
            "rrbot_with_sensor_joint_state_broadcaster",
            "rrbot_with_sensor_fts_broadcaster",
            "rrbot_with_sensor_position_controller",
            "threedofbot_joint_state_broadcaster",
            "threedofbot_pid_gain_controller",
            "threedofbot_position_controller",
        ]
        check_controllers_running(self.node, cnames)
        # still all joint_states are published, even if the hardware is in unconfigured state
        check_if_js_published(
            "/joint_states",
            [
                "rrbot_with_sensor_joint1",
                "rrbot_joint1",
                "rrbot_with_sensor_joint2",
                "rrbot_joint2",
                "threedofbot_joint1",
                "threedofbot_joint2",
                "threedofbot_joint3",
            ],
        )


# TODO(anyone): enable this if shutdown of ros2_control_node does not fail anymore
# @launch_testing.post_shutdown_test()
# # These tests are run after the processes in generate_test_description() have shutdown.
# class TestDescriptionCraneShutdown(unittest.TestCase):

#     def test_exit_codes(self, proc_info):
#         """Check if the processes exited normally."""
#         launch_testing.asserts.assertExitCodes(proc_info)
