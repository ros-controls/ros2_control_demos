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

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_10__GPIO_CONTROLLER_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_10__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/interface_value.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "controller_interface/controller_interface.hpp"

namespace ros2_control_demo_example_10
{
using CmdType = std_msgs::msg::Float64MultiArray;

class GPIOController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_init() override;

private:
  // we hardcode the names of the interfaces here
  std::vector<std::string> state_interfaces_names = {
    "flange_analog_IOs/analog_output1", "flange_analog_IOs/analog_input1",
    "flange_analog_IOs/analog_input2", "flange_vacuum/vacuum"};
  std::vector<std::string> command_interfaces_names = {
    "flange_analog_IOs/analog_output1", "flange_vacuum/vacuum"};

protected:
  void initMsgs();

  // internal commands
  std::shared_ptr<CmdType> output_cmd_ptr_;

  // publisher
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::InterfaceValue>> gpio_publisher_;
  control_msgs::msg::InterfaceValue gpio_msg_;

  // subscriber
  rclcpp::Subscription<CmdType>::SharedPtr subscription_command_;
};
}  // namespace ros2_control_demo_example_10

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_10__GPIO_CONTROLLER_HPP_
