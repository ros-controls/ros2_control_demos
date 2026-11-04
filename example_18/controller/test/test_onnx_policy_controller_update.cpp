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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/msg/float64_values.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "onnx_policy_controller/onnx_policy_controller.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"

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
}  // namespace

// Exercises the real controller update() with the shipped ONNX policy, without simulation.
class OnnxPolicyControllerUpdateTest : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUpController(const std::vector<rclcpp::Parameter> & extra_params = {})
  {
    controller_ = std::make_unique<onnx_policy_controller::OnnxPolicyController>();
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("joints", JOINTS), rclcpp::Parameter("model_path", ONNX_MODEL_PATH)};
    parameters.insert(parameters.end(), extra_params.begin(), extra_params.end());

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "onnx_policy_controller";
    params.node_options = controller_->define_custom_node_options();
    params.node_options.parameter_overrides(parameters);
    ASSERT_EQ(controller_->init(params), return_type::OK);

    std::vector<LoanedCommandInterface> command_ifs;
    for (const auto & joint : JOINTS)
    {
      // Command interfaces start as NaN, so any write by the controller is observable.
      command_interfaces_.push_back(std::make_shared<CommandInterface>(joint, "position"));
      command_ifs.emplace_back(command_interfaces_.back(), nullptr);
    }
    controller_->assign_interfaces(std::move(command_ifs), {});
    executor_.add_node(controller_->get_node()->get_node_base_interface());

    ASSERT_EQ(
      controller_->get_node()->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(
      controller_->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  void TearDown() override
  {
    if (controller_)
    {
      executor_.remove_node(controller_->get_node()->get_node_base_interface());
    }
    controller_.reset();
  }

  return_type update()
  {
    return controller_->update(rclcpp::Time(0, 0), rclcpp::Duration::from_seconds(0.02));
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::unique_ptr<onnx_policy_controller::OnnxPolicyController> controller_;
  std::vector<CommandInterface::SharedPtr> command_interfaces_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(OnnxPolicyControllerUpdateTest, NoSensorDataDoesNotRunOrWriteCommands)
{
  SetUpController();

  EXPECT_EQ(update(), return_type::OK);
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    EXPECT_TRUE(std::isnan(command_interfaces_[i]->get_optional().value()))
      << "joint " << JOINTS[i] << " was written before any sensor data arrived";
  }
}

TEST_F(OnnxPolicyControllerUpdateTest, NoVelocityCommandHoldsDefaultPose)
{
  // No stabilization delay, so a wrongly triggered policy would act on the very first update.
  SetUpController(
    {rclcpp::Parameter("default_joint_positions", HOME),
     rclcpp::Parameter("stabilization_delay", 0)});

  auto test_node = std::make_shared<rclcpp::Node>("test_publisher");
  auto publisher = test_node->create_publisher<control_msgs::msg::Float64Values>(
    "state_interfaces_broadcaster/values", rclcpp::SystemDefaultsQoS());
  publisher->publish(control_msgs::msg::Float64Values());
  spin_for(std::chrono::milliseconds(100));

  // Without a velocity command the policy must not run; the controller holds the standing pose.
  ASSERT_EQ(update(), return_type::OK);
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(command_interfaces_[i]->get_optional().value(), HOME[i]) << JOINTS[i];
  }
}

TEST(OnnxPolicyControllerCommandMapping, VelocityCommandToPolicyInput)
{
  ros2_control_demo_example_18::msg::VelocityCommandWithHead msg;
  msg.base_velocity.linear.x = 0.1;
  msg.base_velocity.linear.y = 0.2;
  msg.base_velocity.angular.z = 0.3;
  msg.neck_pitch = 0.4;
  msg.head_pitch = 0.5;
  msg.head_yaw = 0.6;
  msg.head_roll = 0.7;

  EXPECT_EQ(
    onnx_policy_controller::OnnxPolicyController::velocity_command_to_policy_input(msg),
    std::vector<double>({0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7}));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
