// Copyright 2021 Department of Engineering Cybernetics, NTNU
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

#include "ros2_control_demo_example_3/rrbot_system_multi_interface.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_3
{
hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  control_level_.resize(info_.joints.size(), integration_level_t::POSITION);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemMultiInterface has exactly 3 state interfaces
    // and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces. 3 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %s command interface. Expected %s, %s, or %s.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_ACCELERATION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s'has %zu state interfaces. 3 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %s state interface. Expected %s, %s, or %s.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_ACCELERATION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
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
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemMultiInterfaceHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Prepare for new command modes
  std::vector<integration_level_t> new_modes = {};
  for (std::string key : start_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        new_modes.push_back(integration_level_t::POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        new_modes.push_back(integration_level_t::VELOCITY);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION)
      {
        new_modes.push_back(integration_level_t::ACCELERATION);
      }
    }
  }
  // Example criteria: All joints must be given new command mode at the same time
  if (new_modes.size() != info_.joints.size())
  {
    return hardware_interface::return_type::ERROR;
  }
  // Example criteria: All joints must have the same command mode
  if (!std::all_of(
        new_modes.begin() + 1, new_modes.end(),
        [&](integration_level_t mode) { return mode == new_modes[0]; }))
  {
    return hardware_interface::return_type::ERROR;
  }

  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        set_command(
          info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION,
          get_state(info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION));
        set_command(info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
        set_command(info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION, 0.0);
        control_level_[i] = integration_level_t::UNDEFINED;  // Revert to undefined
      }
    }
  }
  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (control_level_[i] != integration_level_t::UNDEFINED)
    {
      // Something else is using the joint! Abort!
      return hardware_interface::return_type::ERROR;
    }
    control_level_[i] = new_modes[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating... please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // Set some default values
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    control_level_[i] = integration_level_t::UNDEFINED;
  }

  RCLCPP_INFO(get_logger(), "System successfully activated! %u", control_level_[0]);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating... please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemMultiInterfaceHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    const auto name_acc = info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION;
    const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    switch (control_level_[i])
    {
      case integration_level_t::UNDEFINED:
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000, "Nothing is using the hardware interface!");
        return hardware_interface::return_type::OK;
        break;
      case integration_level_t::POSITION:
        set_state(name_acc, 0.);
        set_state(name_vel, 0.);
        set_state(
          name_pos,
          get_state(name_pos) + (get_command(name_pos) - get_state(name_pos)) / hw_slowdown_);
        break;
      case integration_level_t::VELOCITY:
        set_state(name_acc, 0.);
        set_state(name_vel, get_command(name_vel));
        set_state(
          name_pos, get_state(name_pos) + get_state(name_vel) * period.seconds() / hw_slowdown_);
        break;
      case integration_level_t::ACCELERATION:
        set_state(name_acc, get_command(name_acc));
        set_state(
          name_vel, get_state(name_vel) + get_state(name_acc) * period.seconds() / hw_slowdown_);
        set_state(
          name_pos, get_state(name_pos) + get_state(name_vel) * period.seconds() / hw_slowdown_);
        break;
    }
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t"
       << "pos: " << get_state(name_pos) << ", vel: " << get_state(name_vel)
       << ", acc: " << get_state(name_acc) << " for joint " << i;
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemMultiInterfaceHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    // Simulate sending commands to the hardware
    const auto name_acc = info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION;
    const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t"
       << "command pos: " << get_command(name_pos) << ", vel: " << get_command(name_vel)
       << ", acc: " << get_command(name_acc) << " for joint " << i
       << ", control lvl: " << static_cast<int>(control_level_[i]);
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_3

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_3::RRBotSystemMultiInterfaceHardware,
  hardware_interface::SystemInterface)
