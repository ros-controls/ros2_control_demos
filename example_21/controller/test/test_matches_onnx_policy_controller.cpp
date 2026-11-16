// Copyright (C) 2026 ros2_control Development Team
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
//
// Authors: Anthony Shoumikhin

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_msgs/msg/float64_values.hpp"
#include "control_msgs/msg/keys.hpp"
#include "executorch_policy_controller/executorch_policy_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "onnx_policy_controller/onnx_policy_controller.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"

using controller_interface::return_type;
using hardware_interface::CommandInterface;
using hardware_interface::LoanedCommandInterface;

namespace
{
const std::vector<std::string> JOINTS = {
  "left_hip_yaw",   "left_hip_roll",   "left_hip_pitch", "left_knee",  "left_ankle",
  "neck_pitch",     "head_pitch",      "head_yaw",       "head_roll",  "right_hip_yaw",
  "right_hip_roll", "right_hip_pitch", "right_knee",     "right_ankle"};
const std::vector<double> HOME = {0.002, 0.053, -0.63,  1.368,  -0.784, 0.0,   0.0,
                                  0.0,   0.0,   -0.003, -0.065, 0.635,  1.379, -0.796};
// The shipped limits, except a tight upper limit on left_knee so that the clamp is exercised.
const std::vector<double> LIMITS_MIN = {-0.523599, -0.436332, -1.221730, -1.570796, -1.570796,
                                        -0.349066, -0.785398, -2.792527, -0.523599, -0.523599,
                                        -0.436332, -0.523599, -1.570796, -1.570796};
const std::vector<double> LIMITS_MAX = {0.523599, 0.436332, 0.523599, 1.388,    1.570796,
                                        1.134464, 0.785398, 2.792527, 0.523599, 0.523599,
                                        0.436332, 1.221730, 1.570796, 1.570796};

struct ControllerUnderTest
{
  std::unique_ptr<controller_interface::ControllerInterface> controller;
  std::vector<CommandInterface::SharedPtr> commands;
};

template <typename ControllerT>
ControllerUnderTest make_controller(
  const std::string & name, const std::string & model_path, bool with_default_pose)
{
  ControllerUnderTest result{std::make_unique<ControllerT>(), {}};
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = name;
  params.node_options = result.controller->define_custom_node_options();
  std::vector<rclcpp::Parameter> overrides = {
    rclcpp::Parameter("joints", JOINTS), rclcpp::Parameter("model_path", model_path),
    rclcpp::Parameter("joint_position_limits_min", LIMITS_MIN),
    rclcpp::Parameter("joint_position_limits_max", LIMITS_MAX)};
  if (with_default_pose)
  {
    overrides.emplace_back("default_joint_positions", HOME);
  }
  params.node_options.parameter_overrides(overrides);
  EXPECT_EQ(result.controller->init(params), return_type::OK);
  std::vector<LoanedCommandInterface> loaned;
  for (const auto & joint : JOINTS)
  {
    result.commands.push_back(std::make_shared<CommandInterface>(joint, "position"));
    loaned.emplace_back(result.commands.back(), nullptr);
  }
  result.controller->assign_interfaces(std::move(loaned), {});
  EXPECT_EQ(
    result.controller->get_node()->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(
    result.controller->get_node()->activate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  return result;
}

// One sample of state_interfaces_broadcaster: IMU, joint positions and velocities, foot contacts.
std::pair<std::vector<std::string>, std::vector<double>> sensor_sample()
{
  std::vector<std::string> names = {"imu/orientation.x",         "imu/orientation.y",
                                    "imu/orientation.z",         "imu/orientation.w",
                                    "imu/angular_velocity.x",    "imu/angular_velocity.y",
                                    "imu/angular_velocity.z",    "imu/linear_acceleration.x",
                                    "imu/linear_acceleration.y", "imu/linear_acceleration.z"};
  std::vector<double> values = {0.0, 0.0, 0.0, 1.0, 0.3, -0.2, 0.25, 0.1, -0.1, 9.8};
  for (size_t i = 0; i < JOINTS.size(); ++i)
  {
    names.push_back(JOINTS[i] + "/position");
    values.push_back(HOME[i] + 0.01 * static_cast<double>(i % 3));
  }
  for (size_t i = 0; i < JOINTS.size(); ++i)
  {
    names.push_back(JOINTS[i] + "/velocity");
    values.push_back(0.05 * static_cast<double>(i % 4) - 0.1);
  }
  names.insert(names.end(), {"left_foot_contact/contact_raw", "right_foot_contact/contact_raw"});
  values.insert(values.end(), {1.0, 0.0});
  return {names, values};
}

// Both controllers see the same inputs for 300 updates, which covers the stabilization delay, the
// blend-in and the steady state with action history, so their joint commands must stay the same.
void expect_same_commands(bool with_default_pose)
{
  auto onnx = make_controller<onnx_policy_controller::OnnxPolicyController>(
    "onnx_policy_controller", ONNX_MODEL_PATH, with_default_pose);
  auto executorch = make_controller<executorch_policy_controller::ExecuTorchPolicyController>(
    "executorch_policy_controller", PTE_MODEL_PATH, with_default_pose);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(onnx.controller->get_node()->get_node_base_interface());
  executor.add_node(executorch.controller->get_node()->get_node_base_interface());
  auto node = std::make_shared<rclcpp::Node>("test_publisher");
  auto names_publisher = node->create_publisher<control_msgs::msg::Keys>(
    "state_interfaces_broadcaster/names", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  auto values_publisher = node->create_publisher<control_msgs::msg::Float64Values>(
    "state_interfaces_broadcaster/values", rclcpp::SystemDefaultsQoS());
  using ros2_control_demo_example_18::msg::VelocityCommandWithHead;
  std::vector<rclcpp::Publisher<VelocityCommandWithHead>::SharedPtr> command_publishers = {
    node->create_publisher<VelocityCommandWithHead>(
      "onnx_policy_controller/cmd_velocity_with_head", rclcpp::SystemDefaultsQoS()),
    node->create_publisher<VelocityCommandWithHead>(
      "executorch_policy_controller/cmd_velocity_with_head", rclcpp::SystemDefaultsQoS())};

  // Volatile publishers drop messages sent before discovery matched both subscriptions.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline &&
         (values_publisher->get_subscription_count() < 2 ||
          command_publishers[0]->get_subscription_count() == 0 ||
          command_publishers[1]->get_subscription_count() == 0))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(values_publisher->get_subscription_count(), 2u);

  const auto [names, values] = sensor_sample();
  control_msgs::msg::Keys names_msg;
  names_msg.keys = names;
  control_msgs::msg::Float64Values values_msg;
  values_msg.values = values;
  VelocityCommandWithHead command;
  // Distinct values for every field, so a field mapped to the wrong policy input is caught.
  command.base_velocity.linear.x = 0.15;
  command.base_velocity.linear.y = 0.05;
  command.base_velocity.angular.z = 0.2;
  command.neck_pitch = 0.1;
  command.head_pitch = -0.1;
  command.head_yaw = 0.3;
  command.head_roll = -0.05;
  names_publisher->publish(names_msg);
  values_publisher->publish(values_msg);
  for (const auto & publisher : command_publishers)
  {
    publisher->publish(command);
  }
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (std::chrono::steady_clock::now() < end)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto period = rclcpp::Duration::from_seconds(0.02);
  double largest_motion = 0.0;
  bool knee_clamped = false;
  for (int step = 0; step < 300; ++step)
  {
    ASSERT_EQ(onnx.controller->update(rclcpp::Time(0, 0), period), return_type::OK);
    ASSERT_EQ(executorch.controller->update(rclcpp::Time(0, 0), period), return_type::OK);
    for (size_t i = 0; i < JOINTS.size(); ++i)
    {
      const double expected = onnx.commands[i]->get_optional().value();
      ASSERT_NEAR(executorch.commands[i]->get_optional().value(), expected, 1e-5)
        << JOINTS[i] << " at update " << step;
      largest_motion = std::max(largest_motion, std::abs(expected - HOME[i]));
    }
    knee_clamped |= onnx.commands[3]->get_optional().value() == LIMITS_MAX[3];
  }
  // Guards against both controllers trivially agreeing on the standing pose.
  EXPECT_GT(largest_motion, 0.01);
  EXPECT_TRUE(knee_clamped);

  executor.remove_node(onnx.controller->get_node()->get_node_base_interface());
  executor.remove_node(executorch.controller->get_node()->get_node_base_interface());
  onnx.controller.reset();
  executorch.controller.reset();
}
}  // namespace

TEST(ExecuTorchPolicyControllerParity, MatchesOnnxPolicyController) { expect_same_commands(true); }

// Without default_joint_positions, both controllers take the standing pose from the sensors.
TEST(ExecuTorchPolicyControllerParity, MatchesOnnxPolicyControllerWithPoseFromSensors)
{
  expect_same_commands(false);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
