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

import rclpy

from ros2_control_demo_test_utils.test_forward_command import ForwardCommand


class TestForwardCommandModule(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_parameter_defaults(self):
        node = ForwardCommand()
        try:
            self.assertEqual(node.controller_name, "forward_position_controller")
            self.assertEqual(node.joints, ["joint1", "joint2"])
            self.assertEqual(node.publish_topic, "/forward_position_controller/commands")
            self.assertEqual(node.command_value, 0.5)
            self.assertEqual(node.position_tolerance, 0.01)
        finally:
            node.destroy_node()
