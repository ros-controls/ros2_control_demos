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

#include "ros2_control_demo_example_10/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
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

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // RRBotSystemWithGPIOHardware has exactly two GPIO components
  if (info_.gpios.size() != 2)
  {
    RCLCPP_FATAL(
      get_logger(), "RRBotSystemWithGPIOHardware has '%ld' GPIO components, '%d' expected.",
      info_.gpios.size(), 2);
    return hardware_interface::CallbackReturn::ERROR;
  }
  // with exactly 1 command interface
  for (int i = 0; i < 2; i++)
  {
    if (info_.gpios[i].command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "GPIO component %s has '%ld' command interfaces, '%d' expected.",
        info_.gpios[i].name.c_str(), info_.gpios[i].command_interfaces.size(), 1);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  // and 3/1 state interfaces, respectively
  if (info_.gpios[0].state_interfaces.size() != 3)
  {
    RCLCPP_FATAL(
      get_logger(), "GPIO component %s has '%ld' state interfaces, '%d' expected.",
      info_.gpios[0].name.c_str(), info_.gpios[0].state_interfaces.size(), 3);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.gpios[1].state_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      get_logger(), "GPIO component %s has '%ld' state interfaces, '%d' expected.",
      info_.gpios[1].name.c_str(), info_.gpios[1].state_interfaces.size(), 1);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  for (const auto & [name, descr] : gpio_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : gpio_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_command(name, get_state(name));
  }
  for (const auto & [name, descr] : gpio_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemWithGPIOHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemWithGPIOHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";

  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    // Simulate RRBot's movement
    auto new_value = get_state(name) + (get_command(name) - get_state(name)) / hw_slowdown_;
    set_state(name, new_value);
  }

  for (const auto & [name, descr] : gpio_command_interfaces_)
  {
    // mirror GPIOs back
    set_state(name, get_command(name));
  }

  // random inputs analog_input1 and analog_input2
  unsigned int seed = static_cast<unsigned int>(time(NULL)) + 1;
  set_state(
    info_.gpios[0].name + "/" + info_.gpios[0].state_interfaces[1].name,
    static_cast<float>(rand_r(&seed)));
  seed = time(NULL) + 2;
  set_state(
    info_.gpios[0].name + "/" + info_.gpios[0].state_interfaces[2].name,
    static_cast<float>(rand_r(&seed)));

  for (const auto & [name, descr] : gpio_state_interfaces_)
  {
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << get_state(name) << " from GPIO input '" << name << "'";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemWithGPIOHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";

  for (const auto & [name, descr] : gpio_command_interfaces_)
  {
    // Simulate sending commands to the hardware
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << get_command(name) << " for GPIO output '" << name << "'";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_10

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_10::RRBotSystemWithGPIOHardware, hardware_interface::SystemInterface)
