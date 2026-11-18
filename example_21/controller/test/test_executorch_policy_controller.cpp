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
// Authors: Julia Jia, Anthony Shoumikhin

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/msg/float64_values.hpp"
#include "executorch_policy_controller/executorch_policy_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"

using controller_interface::return_type;
using hardware_interface::CommandInterface;
using hardware_interface::LoanedCommandInterface;
using lifecycle_msgs::msg::State;

namespace
{
const std::vector<std::string> JOINTS = {
  "left_hip_yaw",   "left_hip_roll",   "left_hip_pitch", "left_knee",  "left_ankle",
  "neck_pitch",     "head_pitch",      "head_yaw",       "head_roll",  "right_hip_yaw",
  "right_hip_roll", "right_hip_pitch", "right_knee",     "right_ankle"};
const std::vector<double> HOME = {0.002, 0.053, -0.63,  1.368,  -0.784, 0.0,   0.0,
                                  0.0,   0.0,   -0.003, -0.065, 0.635,  1.379, -0.796};
constexpr double kPeriod = 0.02;
constexpr double kMaxMotorVelocity = 5.24;
}  // namespace

// Exercises the real controller update() with the bundled ExecuTorch policy, without simulation.
class ExecuTorchPolicyControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  State::_id_type SetUpController(
    const std::vector<std::string> & joints, const std::vector<rclcpp::Parameter> & extra = {})
  {
    controller_ = std::make_unique<executorch_policy_controller::ExecuTorchPolicyController>();
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("joints", joints), rclcpp::Parameter("model_path", PTE_MODEL_PATH),
      rclcpp::Parameter("default_joint_positions", HOME),
      rclcpp::Parameter("max_motor_velocity", kMaxMotorVelocity)};
    parameters.insert(parameters.end(), extra.begin(), extra.end());

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "executorch_policy_controller";
    params.node_options = controller_->define_custom_node_options();
    params.node_options.parameter_overrides(parameters);
    EXPECT_EQ(controller_->init(params), return_type::OK);

    std::vector<LoanedCommandInterface> command_ifs;
    for (const auto & joint : joints)
    {
      // Command interfaces start as NaN, so any write by the controller is observable.
      command_interfaces_.push_back(std::make_shared<CommandInterface>(joint, "position"));
      command_ifs.emplace_back(command_interfaces_.back(), nullptr);
    }
    controller_->assign_interfaces(std::move(command_ifs), {});
    executor_.add_node(controller_->get_node()->get_node_base_interface());
    return controller_->get_node()->configure().id();
  }

  void Activate()
  {
    ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  }

  void TearDown() override
  {
    if (controller_)
    {
      executor_.remove_node(controller_->get_node()->get_node_base_interface());
    }
    controller_.reset();
  }

  template <typename MessageT>
  void publish_and_spin(const std::string & topic, const MessageT & msg)
  {
    auto publisher = test_node_->create_publisher<MessageT>(topic, rclcpp::SystemDefaultsQoS());
    // The publisher is volatile: publishing before discovery matched the subscription drops it.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(publisher->get_subscription_count(), 0u) << topic << " never matched";
    publisher->publish(msg);
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() < end)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  return_type update()
  {
    return controller_->update(rclcpp::Time(0, 0), rclcpp::Duration::from_seconds(kPeriod));
  }

  std::unique_ptr<executorch_policy_controller::ExecuTorchPolicyController> controller_;
  std::vector<CommandInterface::SharedPtr> command_interfaces_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr test_node_ = std::make_shared<rclcpp::Node>("test_publisher");
};

// The observation size depends on the joint count, so the policy only fits 14 joints.
TEST_F(ExecuTorchPolicyControllerTest, JointCountMustMatchPolicy)
{
  const std::vector<std::string> too_few_joints(JOINTS.begin(), JOINTS.end() - 1);
  EXPECT_NE(SetUpController(too_few_joints), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(ExecuTorchPolicyControllerTest, NoSensorDataDoesNotWriteCommands)
{
  ASSERT_EQ(SetUpController(JOINTS), State::PRIMARY_STATE_INACTIVE);
  Activate();

  EXPECT_EQ(update(), return_type::OK);
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    EXPECT_TRUE(std::isnan(command_interfaces_[i]->get_optional().value())) << JOINTS[i];
  }
}

TEST_F(ExecuTorchPolicyControllerTest, NoVelocityCommandHoldsDefaultPose)
{
  ASSERT_EQ(
    SetUpController(JOINTS, {rclcpp::Parameter("stabilization_delay", 0)}),
    State::PRIMARY_STATE_INACTIVE);
  Activate();
  publish_and_spin("state_interfaces_broadcaster/values", control_msgs::msg::Float64Values());

  ASSERT_EQ(update(), return_type::OK);
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(command_interfaces_[i]->get_optional().value(), HOME[i]) << JOINTS[i];
  }
}

TEST_F(ExecuTorchPolicyControllerTest, HeldPoseIsClampedToJointLimits)
{
  std::vector<double> limits_max(JOINTS.size(), 3.0);
  limits_max[3] = 1.3;  // left_knee, below its standing position 1.368
  ASSERT_EQ(
    SetUpController(
      JOINTS,
      {rclcpp::Parameter("stabilization_delay", 0),
       rclcpp::Parameter("joint_position_limits_min", std::vector<double>(JOINTS.size(), -3.0)),
       rclcpp::Parameter("joint_position_limits_max", limits_max)}),
    State::PRIMARY_STATE_INACTIVE);
  Activate();
  publish_and_spin("state_interfaces_broadcaster/values", control_msgs::msg::Float64Values());

  ASSERT_EQ(update(), return_type::OK);
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(
      command_interfaces_[i]->get_optional().value(), std::min(HOME[i], limits_max[i]))
      << JOINTS[i];
  }
}

TEST_F(ExecuTorchPolicyControllerTest, VelocityCommandRunsPolicy)
{
  // No stabilization delay or blend-in, so the first update writes the policy's action.
  ASSERT_EQ(
    SetUpController(
      JOINTS,
      {rclcpp::Parameter("stabilization_delay", 0), rclcpp::Parameter("blend_in_steps", 1)}),
    State::PRIMARY_STATE_INACTIVE);
  Activate();
  publish_and_spin("state_interfaces_broadcaster/values", control_msgs::msg::Float64Values());
  ros2_control_demo_example_18::msg::VelocityCommandWithHead command;
  command.base_velocity.linear.x = 0.1;
  publish_and_spin("executorch_policy_controller/cmd_velocity_with_head", command);

  ASSERT_EQ(update(), return_type::OK);
  double largest_change = 0.0;
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    const double change = std::abs(command_interfaces_[i]->get_optional().value() - HOME[i]);
    EXPECT_LE(change, kMaxMotorVelocity * kPeriod + 1e-9) << JOINTS[i] << " ignored rate limit";
    largest_change = std::max(largest_change, change);
  }
  EXPECT_GT(largest_change, 1e-3) << "the policy did not move any joint";
}

// After cleanup and configure, the controller must start over: stabilization delay again and no
// action history from the earlier run, so its first policy command equals the first one of the run.
TEST_F(ExecuTorchPolicyControllerTest, ReconfiguredControllerStartsOver)
{
  constexpr int kDelay = 3;
  ASSERT_EQ(
    SetUpController(
      JOINTS,
      {rclcpp::Parameter("stabilization_delay", kDelay), rclcpp::Parameter("blend_in_steps", 1)}),
    State::PRIMARY_STATE_INACTIVE);
  ros2_control_demo_example_18::msg::VelocityCommandWithHead command;
  command.base_velocity.linear.x = 0.1;
  const auto run_until_first_policy_command = [&]()
  {
    Activate();
    publish_and_spin("state_interfaces_broadcaster/values", control_msgs::msg::Float64Values());
    publish_and_spin("executorch_policy_controller/cmd_velocity_with_head", command);
    std::vector<double> first_policy_command(JOINTS.size());
    // The policy first runs on update number kDelay.
    for (int step = 1; step <= kDelay; ++step)
    {
      EXPECT_EQ(update(), return_type::OK);
      for (size_t i = 0; i < JOINTS.size(); ++i)
      {
        const double value = command_interfaces_[i]->get_optional().value();
        if (step < kDelay)
        {
          EXPECT_DOUBLE_EQ(value, HOME[i]) << JOINTS[i] << " moved during stabilization";
        }
        first_policy_command[i] = value;
      }
    }
    return first_policy_command;
  };

  const auto first_run = run_until_first_policy_command();
  for (int step = 0; step < 5; ++step)
  {
    ASSERT_EQ(update(), return_type::OK);
  }
  ASSERT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(controller_->get_node()->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(controller_->get_node()->configure().id(), State::PRIMARY_STATE_INACTIVE);
  const auto second_run = run_until_first_policy_command();

  double largest_change = 0.0;
  for (size_t i = 0; i < JOINTS.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(second_run[i], first_run[i]) << JOINTS[i];
    largest_change = std::max(largest_change, std::abs(first_run[i] - HOME[i]));
  }
  EXPECT_GT(largest_change, 1e-3) << "the policy did not move any joint";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
