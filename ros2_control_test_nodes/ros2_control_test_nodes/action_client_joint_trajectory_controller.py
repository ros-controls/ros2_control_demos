# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from control_msgs.action import FollowJointTrajectory


class ActionClientJointTrajectory(Node):
    def __init__(self):
        super().__init__("action_client_joint_trajectory_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "position_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 6)
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("joints")
        self.declare_parameter("check_starting_point", False)
        self.declare_parameter(
            "starting_point_limits", None, ParameterDescriptor(dynamic_typing=True)
        )

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )
        # initialize starting point status
        if not self.check_starting_point:
            self.starting_point_ok = True
        else:
            self.starting_point_ok = False

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = []
        for name in goal_names:
            self.declare_parameter(name)
            goal = self.get_parameter(name).value
            if goal is None or len(goal) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_goal = []
            for value in goal:
                float_goal.append(float(value))
            self.goals.append(float_goal)

        action_name = "/" + controller_name + "/" + "follow_joint_trajectory"

        self.get_logger().info(
            'Sending {} goals on action "{}" every {} s '.format(
                len(goal_names), action_name, wait_sec_between_publish
            )
        )

        self._action_client = ActionClient(self, FollowJointTrajectory, action_name)
        self._action_client.wait_for_server()
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        if self.starting_point_ok:

            traj = JointTrajectory()
            traj.joint_names = self.joints
            point = JointTrajectoryPoint()
            point.positions = self.goals[self.i]
            point.time_from_start = Duration(sec=4)

            traj.points.append(point)
            goal_msg = FollowJointTrajectory.Goal(trajectory=traj)

            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

            self.i += 1
            self.i %= len(self.goals)

        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warn("Start configuration is not within configured limits!")

    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:

            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")


def main(args=None):
    rclpy.init(args=args)

    ac_joint_trajectory = ActionClientJointTrajectory()

    rclpy.spin(ac_joint_trajectory)
    ac_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
