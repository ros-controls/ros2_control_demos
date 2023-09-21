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

  for (const hardware_interfacec::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = info_.joints[i].name.find("steering") != std::string::npos;

    // Steering joints have a position command and state interface
    if (joint_is_steering)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
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

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      // Drive joints have a velocity command interface and velocity and position state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // Running in simulation: we have individual control of two front steering wheels and two rear
  // drive wheels
  if (m_running_simulation)
  {
    // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
    // code
    hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    // // END: This part here is for exemplary purposes - Please do not copy to your production code
  }
  // Running on hardware: we have a single front steering and a single rear drive motor
  else if (!m_running_simulation)
  {
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // int commands_idx = 0;

  int positions_idx = 0;
  int velocities_idx = 0;

  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    bool joint_is_steering = info_.joints[i].name.find("steering") != std::string::npos;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));

    if (!joint_is_steering)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    else
    {
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

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    bool joint_is_steering = info_.joints[i].name.find("steering") != std::string::npos;

    if (joint_is_steering)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
    else
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported %zu command interfaces.",
    command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported command interface '%s'.",
      command_interfaces[i].get_name().c_str());
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

    for (uint i = 0; i < hw_positions_.size(); i++)
    {
      hw_positions_[i] = 0.0;
      hw_velocities_[i] = 0.0;
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
  for (std::size_t i = 0; i < hw_velocities_.size(); ++i)
  {
    hw_positions_[i] += hw_velocities_[i] * period.seconds();

    RCLCPP_INFO(
      rclcpp::get_logger("F1TENTHSystemHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());
  }

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

  for (auto i = 0u; i < hw_commands_.size(); ++i)
  {
    hw_velocities_[i] = hw_commands_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("F1TENTHSystemHardware"), "Got command %.5f for '%s'", hw_commands_[0],
      info_.joints[i].name.c_str());
  }

  return hardware_interface::return_type::OK;

  if (m_running_simulation)
  {
    for (auto i = 0u; i < hw_commands_.size(); i++)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"),
        "Got command %.5f for joint '%s' with type %s!", hw_commands_[i],
        info_.joints[i].name.c_str(), info_.joints[i].command_interfaces[0].name.c_str());
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
        "Successfully written command %.5f to joint %d of type %s!", hw_commands_[i], i,
        interface_type.c_str());
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