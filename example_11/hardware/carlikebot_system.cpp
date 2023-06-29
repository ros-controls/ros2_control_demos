// Copyright 2021 ros2_control Development Team
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

#include "ros2_control_demo_example_11/carlikebot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_11
{
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  m_running_simulation = std::stod(info_.hardware_parameters["is_simulation"]);

  // Check if the number of joints is correct based on the mode of operation
  if (m_running_simulation && info_.joints.size() != 4)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CarlikeBotSystemHardware"),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 4 for simulation.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!m_running_simulation && info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CarlikeBotSystemHardware"),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2 for physical.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("CarlikeBotSystemHardware"), "CarlikeBotSystemHardware is %s.",
    m_running_simulation ? "running in simulation" : "running on hardware");

  // All joints should have one state interface and one command interface
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CarlikeBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CarlikeBotSystemHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check that the joints have the correct interfaces
    // The steering joints have position interfaces and the drive joints velocity
    if (joint.name.find("steering") != std::string::npos)
    {
      // Steering joints should have a single position command and state interface
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      }
    }
    else
    {
      // Drive joints should have a single velocity command and state interface
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      }
    }
  }

  // Running in simulation: we have individual control of two front steering wheels and two rear
  // drive wheels
  if (m_running_simulation)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    // Two steering joints and two drive joints
    hw_positions_.resize(2, std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(2, std::numeric_limits<double>::quiet_NaN());
    // Which means 4 commands
    hw_commands_.resize(4, std::numeric_limits<double>::quiet_NaN());
  }
  // Running on hardware: we have a single front steering and a single rear drive motor
  else if (!m_running_simulation)
  {
    // A single steering joint and a single drive joint
    hw_positions_.resize(1, std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(1, std::numeric_limits<double>::quiet_NaN());
    // Which means 2 commands
    hw_commands_.resize(2, std::numeric_limits<double>::quiet_NaN());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // int commands_idx = 0;

  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    bool is_steering_joint = info_.joints[i].name.find("steering") != std::string::npos;

    RCLCPP_DEBUG(
      rclcpp::get_logger("CarlikeBotSystemHardware"),
      "Joint '%s' has state interfaces: '%s', '%s'.", info_.joints[i].name.c_str(),
      info_.joints[i].state_interfaces[0].name.c_str(),
      info_.joints[i].state_interfaces[1].name.c_str());

    if (is_steering_joint)
    {
      // hw_positions_.insert(std::make_pair(info_.joints[i].name,
      // std::numeric_limits<double>::quiet_NaN()));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[0]));
      // hw_names_.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      // commands_idx); commands_idx++;
    }
    else
    {
      // hw_velocities_.insert(std::make_pair(info_.joints[i].name,
      // std::numeric_limits<double>::quiet_NaN()));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]));
      // hw_names_.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      // commands_idx); commands_idx++;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported %zu state interfaces.",
    state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported state interface '%s'.",
      s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int position_commands_counter = 0;
  int velocity_commands_counter = 0;
  int commands_counter = 0;

  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    bool is_steering_joint = info_.joints[i].name.find("steering") != std::string::npos;

    if (is_steering_joint)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
      commands_counterpart_.emplace_back(
        commands_counter, hardware_interface::HW_IF_POSITION, position_commands_counter);
      position_commands_counter++;
      commands_counter++;
    }
    else
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
      commands_counterpart_.emplace_back(
        commands_counter, hardware_interface::HW_IF_VELOCITY, velocity_commands_counter);
      velocity_commands_counter++;
      commands_counter++;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported %zu command interfaces.",
    command_interfaces.size());

  for (int i = 0u; i < commands_positions_idx_.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("CarlikeBotSystemHardware"),
      "Exported position joint %d at command interface %d.", commands_positions_idx_[i].first,
      commands_positions_idx_[i].second);
  }

  for (int i = 0u; i < commands_velocities_idx_.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("CarlikeBotSystemHardware"),
      "Exported velocity joint %d at command interface %d.", commands_velocities_idx_[i].first,
      commands_velocities_idx_[i].second);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (m_running_simulation)
  {
    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Activating ...please wait...");

    for (auto i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
    }

    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
      hw_positions_[i] = 0.0;
    }

    for (auto i = 0u; i < hw_velocities_.size(); i++)
    {
      hw_velocities_[i] = 0.0;
    }

    for (auto i = 0u; i < hw_commands_.size(); i++)
    {
      hw_commands_[i] = 0.0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (m_running_simulation)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Deactivating ...please wait...");

    for (auto i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code
  }
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (m_running_simulation)
  {
    // Inside the write method both position and velocity states for the simulation are updated
  }
  else
  {
    // Robot specific code to read data from the hardware
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_11 ::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Writing...");

  if (m_running_simulation)
  {
    for (auto i = 0u; i < hw_commands_.size(); i++)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Got command %.5f for '%s'!",
        hw_commands_[i], info_.joints[i].name.c_str());
    }

    for (auto i = 0u; i < hw_commands_.size(); i++)
    {
      // Find the tuple in commands_counterpart_ that has the first value of it equal to i
      auto it = std::find_if(
        commands_counterpart_.begin(), commands_counterpart_.end(),
        [i](const std::tuple<int, std::string, int> & element)
        { return std::get<0>(element) == i; });

      // Get the interface_type from the second value of the tuple
      std::string interface_type = std::get<1>(*it);

      // Get the interface specific idx from the third value of the tuple
      int interface_idx = std::get<2>(*it);

      if (interface_type == "position")
      {
        hw_positions_[interface_idx] = hw_commands_[i];
      }
      else
      {
        hw_velocities_[interface_idx] = hw_commands_[i];
      }

      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"),
        "Successfully written to joint %d of type %s!", i, interface_type.c_str());
    }

    // for (auto it : hw_commands_)
    // {
    //   RCLCPP_INFO(
    //     rclcpp::get_logger("CarlikeBotSystemHardware"), "Got command %.5f for '%s'!", it.second,
    //     it.first.c_str());

    //   if

    //   if (it.first.find("steering") != std::string::npos)
    //   {
    //     hw_positions_.at(it.first) = it.second;
    //   }
    //   else
    //   {
    //     hw_velocities_.at(it.first) = it.second;
    //   }
    // }
  }

  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_11

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_11::CarlikeBotSystemHardware, hardware_interface::SystemInterface)
