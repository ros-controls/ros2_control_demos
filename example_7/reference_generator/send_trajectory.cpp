// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_trajectory");
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/r6bot_controller/joint_trajectory", 10);

  // get robot description
  auto robot_param = rclcpp::Parameter();
  node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  node->get_parameter("robot_description", robot_param);
  auto robot_description = robot_param.as_string();

  // create kinematic chain
  KDL::Tree robot_tree;
  KDL::Chain chain;
  kdl_parser::treeFromString(robot_description, robot_tree);
  robot_tree.getChain("base_link", "tool0", chain);

  auto joint_positions = KDL::JntArray(chain.getNrOfJoints());
  auto joint_velocities = KDL::JntArray(chain.getNrOfJoints());
  auto twist = KDL::Twist();
  // create KDL solvers
  auto ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->now();
  for (size_t i = 0; i < chain.getNrOfSegments(); i++)
  {
    auto joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
    {
      trajectory_msg.joint_names.push_back(joint.getName());
    }
  }

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.resize(chain.getNrOfJoints());
  trajectory_point_msg.velocities.resize(chain.getNrOfJoints());

  double total_time = 3.0;
  int trajectory_len = 200;
  int loop_rate = trajectory_len / total_time;
  double dt = 1.0 / loop_rate;

  for (int i = 0; i < trajectory_len; i++)
  {
    // set endpoint twist
    double t = i;
    twist.vel.x(2 * 0.3 * cos(2 * M_PI * t / trajectory_len));
    twist.vel.y(-0.3 * sin(2 * M_PI * t / trajectory_len));

    // convert cart to joint velocities
    ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);

    // copy to trajectory_point_msg
    std::memcpy(
      trajectory_point_msg.positions.data(), joint_positions.data.data(),
      trajectory_point_msg.positions.size() * sizeof(double));
    std::memcpy(
      trajectory_point_msg.velocities.data(), joint_velocities.data.data(),
      trajectory_point_msg.velocities.size() * sizeof(double));

    // integrate joint velocities
    joint_positions.data += joint_velocities.data * dt;

    // set timing information
    trajectory_point_msg.time_from_start.sec = i / loop_rate;
    trajectory_point_msg.time_from_start.nanosec = static_cast<int>(
      1E9 / loop_rate *
      static_cast<double>(t - loop_rate * (i / loop_rate)));  // implicit integer division

    trajectory_msg.points.push_back(trajectory_point_msg);
  }

  pub->publish(trajectory_msg);
  while (rclcpp::ok())
  {
  }

  return 0;
}
