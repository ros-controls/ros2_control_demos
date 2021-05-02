# Copyright 2021 ros2_control Development Team
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
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy


class PublisherJTC(Node):
    def __init__(self):
        super().__init__("publisher_joint_trajectory_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 5)
        self.declare_parameter("joint_names", ["joint1", "joint2"])
        self.declare_parameter("joint_values", [0.3, 0.3])

        # Read parameters
        self.msg = JointTrajectory()
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        self.msg.joint_names = self.get_parameter("joint_names").value
        joint_values = self.get_parameter("joint_values").value

        # Create message
        assert len(self.msg.joint_names) == len(joint_values)
        tp = JointTrajectoryPoint()
        tp.positions = joint_values
        tp.time_from_start.sec = 1
        self.msg.points = [tp]

        self.zero_msg = deepcopy(self.msg)
        self.zero_msg.points[0].positions = [0.0] * len(joint_values)

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            f'Publishing {len(self.msg.joint_names)} goals on topic "{publish_topic}"\
              every {wait_sec_between_publish} s'
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.i % 2 == 0:
            self.publisher_.publish(self.msg)
            self.get_logger().info(f'Publishing: "{self.msg}"')
        else:
            self.publisher_.publish(self.zero_msg)
            self.get_logger().info(f'Publishing: "{self.zero_msg}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher_forward_position = PublisherJTC()

    rclpy.spin(publisher_forward_position)
    publisher_forward_position.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
