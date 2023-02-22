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
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_3
{
hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_level_.resize(info_.joints.size(), integration_level_t::POSITION);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemMultiInterface has exactly 3 state interfaces
    // and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has %zu command interfaces. 3 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s'has %zu state interfaces. 3 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_accelerations_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION,
      &hw_commands_accelerations_[i]));
  }

  return command_interfaces;
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
        hw_commands_velocities_[i] = 0;
        hw_commands_accelerations_[i] = 0;
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
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Activating... please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // Set some default values
  for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
  {
    if (std::isnan(hw_states_positions_[i]))
    {
      hw_states_positions_[i] = 0;
    }
    if (std::isnan(hw_states_velocities_[i]))
    {
      hw_states_velocities_[i] = 0;
    }
    if (std::isnan(hw_states_accelerations_[i]))
    {
      hw_states_accelerations_[i] = 0;
    }
    if (std::isnan(hw_commands_positions_[i]))
    {
      hw_commands_positions_[i] = 0;
    }
    if (std::isnan(hw_commands_velocities_[i]))
    {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_accelerations_[i]))
    {
      hw_commands_accelerations_[i] = 0;
    }
    control_level_[i] = integration_level_t::UNDEFINED;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "System successfully activated! %u",
    control_level_[0]);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemMultiInterfaceHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Deactivating... please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemMultiInterfaceHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
  {
    switch (control_level_[i])
    {
      case integration_level_t::UNDEFINED:
        RCLCPP_INFO(
          rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
          "Nothing is using the hardware interface!");
        return hardware_interface::return_type::OK;
        break;
      case integration_level_t::POSITION:
        hw_states_accelerations_[i] = 0;
        hw_states_velocities_[i] = 0;
        hw_states_positions_[i] +=
          (hw_commands_positions_[i] - hw_states_positions_[i]) / hw_slowdown_;
        break;
      case integration_level_t::VELOCITY:
        hw_states_accelerations_[i] = 0;
        hw_states_velocities_[i] = hw_commands_velocities_[i];
        hw_states_positions_[i] += (hw_states_velocities_[i] * period.seconds()) / hw_slowdown_;
        break;
      case integration_level_t::ACCELERATION:
        hw_states_accelerations_[i] = hw_commands_accelerations_[i];
        hw_states_velocities_[i] += (hw_states_accelerations_[i] * period.seconds()) / hw_slowdown_;
        hw_states_positions_[i] += (hw_states_velocities_[i] * period.seconds()) / hw_slowdown_;
        break;
    }
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
      "Got pos: %.5f, vel: %.5f, acc: %.5f for joint %lu!", hw_states_positions_[i],
      hw_states_velocities_[i], hw_states_accelerations_[i], i);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemMultiInterfaceHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  for (std::size_t i = 0; i < hw_commands_positions_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
      "Got the commands pos: %.5f, vel: %.5f, acc: %.5f for joint %lu, control_lvl:%u",
      hw_commands_positions_[i], hw_commands_velocities_[i], hw_commands_accelerations_[i], i,
      control_level_[i]);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_3

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_3::RRBotSystemMultiInterfaceHardware,
  hardware_interface::SystemInterface)
