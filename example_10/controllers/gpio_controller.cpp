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

#include "ros2_control_demo_example_10/gpio_controller.hpp"

#include <string>

namespace ros2_control_demo_example_10
{
controller_interface::CallbackReturn GPIOController::on_init()
{
  initMsgs();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto command_interfaces_name : command_interfaces_names)
  {
    config.names.emplace_back(command_interfaces_name);
  }

  return config;
}

controller_interface::InterfaceConfiguration
ros2_control_demo_example_10::GPIOController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto state_interface_name : state_interfaces_names)
  {
    config.names.emplace_back(state_interface_name);
  }

  return config;
}

controller_interface::return_type ros2_control_demo_example_10::GPIOController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // send inputs
  for (size_t i = 0; i < state_interfaces_.size(); i++)
  {
    // RCLCPP_INFO(
    //   get_node()->get_logger(), "%s: (%f)", state_interfaces_[i].get_name().c_str(),
    //   state_interfaces_[i].get_value());
    gpio_msg_.values.at(i) = static_cast<float>(state_interfaces_.at(i).get_value());
  }
  gpio_publisher_->publish(gpio_msg_);

  // set outputs
  if (!output_cmd_ptr_)
  {
    // no command received yet
    return controller_interface::return_type::OK;
  }
  if (output_cmd_ptr_->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)", output_cmd_ptr_->data.size(),
      command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < command_interfaces_.size(); i++)
  {
    command_interfaces_[i].set_value(output_cmd_ptr_->data[i]);
    // RCLCPP_INFO(
    //   get_node()->get_logger(), "%s: (%f)", command_interfaces_[i].get_name().c_str(),
    //   command_interfaces_[i].get_value());
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ros2_control_demo_example_10::GPIOController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    // register publisher
    gpio_publisher_ = get_node()->create_publisher<control_msgs::msg::InterfaceValue>(
      "~/inputs", rclcpp::SystemDefaultsQoS());

    // register subscriber
    subscription_command_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { output_cmd_ptr_ = msg; });
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GPIOController::initMsgs()
{
  for (auto state_interfaces_name : state_interfaces_names)
  {
    gpio_msg_.interface_names.emplace_back(state_interfaces_name);
  }
  gpio_msg_.values.resize(state_interfaces_names.size());
}

controller_interface::CallbackReturn ros2_control_demo_example_10::GPIOController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ros2_control_demo_example_10::GPIOController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    // reset publisher
    gpio_publisher_.reset();
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace ros2_control_demo_example_10

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_10::GPIOController, controller_interface::ControllerInterface)
