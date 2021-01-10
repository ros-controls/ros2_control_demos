# Copyright 2021 Stogl Robotics Consulting
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

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PublisherJointTrajectory(Node):

    def __init__(self):
        super().__init__('publisher_joint_trajectory_controller')
        # Declare all parameters
        self.declare_parameter('controller_name', "forward_command_controller_position")
        self.declare_parameter('wait_sec_between_publish', 5)
        self.declare_parameter('goal_names', ['pos1', 'pos2'])
        self.declare_parameter('joint_names')

        # Read parameters
        controller_name = self.get_parameter('controller_name').value
        wait_sec_between_publish = self.get_parameter('wait_sec_between_publish').value
        goal_names = self.get_parameter('goal_names').value
        self.joint_names = self.get_parameter('joint_names').value

        if self.joint_names is None or len(self.joint_names) == 0:
            raise Exception('"joint_names" parameter is not set!')

        # Read all positions from parameters
        self.goals = []
        for name in goal_names:
            self.declare_parameter(name)
            goal = self.get_parameter(name).value
            if goal is None or len(goal) == 0:
                raise Exception('Values for goal "{}" not set!'.format(name))

            float_goal = []
            for value in goal:
                float_goal.append(float(value))
            self.goals.append(float_goal)

        publish_topic = "/" + controller_name + "/" + "commands"

        self.get_logger().info(
            'Publishing {} goals on topic "{}" every {} s'.format(
                len(goal_names), publish_topic, wait_sec_between_publish))

        self.publisher_ = self.create_publisher(
            JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.goals[self.i]
        point.time_from_start = rclpy.Duration(4)

        traj.points.append(point)
        self.publisher_.publish(traj)

        self.i += 1
        self.i %= len(self.goals)


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
