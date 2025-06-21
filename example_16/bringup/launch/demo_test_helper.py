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

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool


class DiffbotChainedControllersTest(Node):
    def __init__(self):
        super().__init__("diffbot_chained_controllers_demo_helper_node")
        self.publisher_ = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        request_left = SetBool.Request()
        request_left.data = True
        future_left = self.client_left_.call_async(request_left)

        request_right = SetBool.Request()
        request_right.data = True
        future_right = self.client_right_.call_async(request_right)

        rclpy.spin_until_future_complete(self, future_left)
        rclpy.spin_until_future_complete(self, future_right)

        self.get_logger().info("Enabled feedforward control for both wheels.")

    def publish_cmd_vel(self, delay=0.1):
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = 0.7
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 1.0

        while rclpy.ok():
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(f"Publishing twist message to cmd_vel: {twist_msg}")
            self.publisher_.publish(twist_msg)
            time.sleep(delay)


if __name__ == "__main__":
    rclpy.init()
    test_node = DiffbotChainedControllersTest()
    test_node.publish_cmd_vel(delay=0.1)
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()
