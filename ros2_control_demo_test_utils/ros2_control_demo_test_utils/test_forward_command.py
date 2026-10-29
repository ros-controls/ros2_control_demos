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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

TIMEOUT = 15.0
WAIT_FOR_JOINT_STATES_TIMEOUT = 10.0


class ForwardCommand(Node):

    def __init__(self):
        super().__init__("test_forward_command")
        self.declare_parameter("controller_name", "forward_position_controller")
        self.declare_parameter("joints", ["joint1", "joint2"])
        self.declare_parameter("initial_position", 0.0)
        self.declare_parameter("command_value", 0.5)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("min_movement", 0.1)

        self.controller_name = self.get_parameter("controller_name").value
        self.joints = self.get_parameter("joints").value
        self.initial_position = self.get_parameter("initial_position").value
        self.command_value = self.get_parameter("command_value").value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.min_movement = self.get_parameter("min_movement").value
        self.publish_topic = f"/{self.controller_name}/commands"

        self.publisher_ = self.create_publisher(Float64MultiArray, self.publish_topic, 10)
        self.joint_state_msg = None
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

    def joint_state_callback(self, msg):
        self.joint_state_msg = msg

    def run(self):
        # 1. Wait for first joint state message
        start_time = self.get_clock().now()
        while self.joint_state_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > WAIT_FOR_JOINT_STATES_TIMEOUT:
                self.get_logger().error("Timeout waiting for /joint_states")
                return 1

        # 2. Record and verify initial positions
        msg = self.joint_state_msg
        if not all(joint in msg.name for joint in self.joints):
            self.get_logger().error(f"/joint_states does not contain joints {self.joints}")
            return 1

        indices = [msg.name.index(joint) for joint in self.joints]
        init_positions = [msg.position[i] for i in indices]

        self.get_logger().info(f"Initial positions: {dict(zip(self.joints, init_positions))}")

        if any(
            abs(pos - self.initial_position) > self.position_tolerance for pos in init_positions
        ):
            self.get_logger().error(f"Joints not at initial position {self.initial_position}")
            return 1

        # 3. Publish position command
        command = Float64MultiArray()
        command.data = [self.command_value] * len(self.joints)
        self.publisher_.publish(command)
        self.get_logger().info(f"Published command: {command.data}")

        # 4. Wait for hardware simulation to respond
        end_time = self.get_clock().now().nanoseconds + int(TIMEOUT * 1e9)
        while self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 5. Check final positions
        final_positions = [self.joint_state_msg.position[i] for i in indices]

        self.get_logger().info(f"Final positions: {dict(zip(self.joints, final_positions))}")

        if any(pos < self.min_movement for pos in final_positions):
            self.get_logger().error(
                f"Joints did not move enough. "
                f"Expected >{self.min_movement}, got "
                f"{dict(zip(self.joints, final_positions))}"
            )
            return 1

        self.get_logger().info("Movement validated successfully")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = ForwardCommand()
    result = node.run()
    node.destroy_node()
    rclpy.shutdown()
    exit(result)


if __name__ == "__main__":
    main()
