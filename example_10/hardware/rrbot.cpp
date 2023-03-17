// Copyright 2020 ros2_control Development Team
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

#include "ros2_control_demo_example_10/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_10
{
hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Configuring ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }
  for (uint i = 0; i < hw_gpio_in_.size(); i++)
  {
    hw_gpio_in_[i] = 0;
  }
  for (uint i = 0; i < hw_gpio_out_.size(); i++)
  {
    hw_gpio_out_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemWithGPIOHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "State interfaces:");
  for (size_t i = 0; i < info_.gpios.size(); i++)
  {
    for (auto state_if : info_.gpios.at(i).state_interfaces)
    {
      // there might be multiple state interfaces per GPIO group
      hw_gpio_in_.emplace_back(std::numeric_limits<double>::quiet_NaN());
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.gpios.at(i).name, state_if.name, &hw_gpio_in_[i]));
      RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Added %s/%s",
        info_.gpios.at(i).name.c_str(), state_if.name.c_str());
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemWithGPIOHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Command interfaces:");
  for (size_t i = 0; i < info_.gpios.size(); i++)
  {
    for (auto command_if : info_.gpios.at(i).command_interfaces)
    {
      // there might be multiple command interfaces per GPIO group
      hw_gpio_out_.emplace_back(std::numeric_limits<double>::quiet_NaN());
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios.at(i).name, command_if.name, &hw_gpio_out_[i]));
      RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Added %s/%s",
        info_.gpios.at(i).name.c_str(), command_if.name.c_str());
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Activating ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemWithGPIOHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]);
    // RCLCPP_INFO(
    //   rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Got state %.5f for joint %d!",
    //   hw_states_[i], i);
  }
  // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Joints successfully read!");

  for (uint i = 0; i < hw_gpio_in_.size(); i++)
  {
    hw_gpio_in_[i] = i;
    // mirror GPIOs back
    hw_gpio_in_[i] = hw_gpio_out_[i];

    // RCLCPP_INFO(
    //   rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Read %.1f from GP input %d!",
    //   hw_gpio_in_[i], i);
  }
  // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "GPIOs successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemWithGPIOHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Writing...");

  // for (uint i = 0; i < hw_commands_.size(); i++)
  // {
  //   // Simulate sending commands to the hardware
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Got command %.5f for joint %d!",
  //     hw_commands_[i], i);
  // }
  // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Joints successfully written!");

  // for (uint i = 0; i < hw_gpio_out_.size(); i++)
  // {
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "Got command %.1f for GP output %d!",
  //     hw_gpio_out_[i], i);
  // }
  // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemWithGPIOHardware"), "GPIOs successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_10

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_10::RRBotSystemWithGPIOHardware, hardware_interface::SystemInterface)
