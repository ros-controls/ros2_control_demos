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

#include "ros2_control_demo_hardware/rrbot_transmissions_system_position_only.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"

namespace ros2_control_demo_hardware
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  logger_ =
    std::make_unique<rclcpp::Logger>(
    rclcpp::get_logger(
      "RRBotTransmissionsSystemPositionOnlyHardware"));

  RCLCPP_INFO(*logger_, "Initializing...");

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);

#ifdef UNDEF
  joint_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        *logger_,
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        *logger_,
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        *logger_,
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        *logger_,
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
#endif

  /// @todo check joint data info from the core interfaces is consistent
  /// with the one in the transmissions, i.e., number and name of joints

  const auto num_joints = std::accumulate(
    info_.transmissions.begin(), info_.transmissions.end(), 0,
    [](const auto & acc, const auto & trans_info)
    {
      return acc + trans_info.joints.size();
    });

  const auto num_actuators = std::accumulate(
    info_.transmissions.begin(), info_.transmissions.end(), 0,
    [](const auto & acc, const auto & trans_info)
    {
      return acc + trans_info.actuators.size();
    });

  // prereserve the space needed for joint and actuator interfaces
  joint_interfaces_.reserve(num_joints);
  actuator_interfaces_.reserve(num_actuators);

  // create transmissions, joint and actuator handles
  auto transmission_loader = transmission_interface::SimpleTransmissionLoader();

  for (const auto & transmission_info : info_.transmissions) {
    // only simple transmissions are supported in this demo
    if (transmission_info.type != "transmission_interface/SimpleTransmission") {
      RCLCPP_FATAL(
        *logger_, "Transmission '%s' of type '%s' not supported in this demo",
        transmission_info.name.c_str(), transmission_info.type.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    /// @todo add try/catch pair for this
    auto transmission = transmission_loader.load(transmission_info);

    std::vector<transmission_interface::JointHandle> joint_handles;
    for (const auto & joint_info : transmission_info.joints) {
      if (!(joint_info.interfaces.size() == 1 &&
        joint_info.interfaces[0] == hardware_interface::HW_IF_POSITION))
      {
        RCLCPP_FATAL(
          *logger_, "Invalid transmission joint '%s' configuration", joint_info.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      auto joint_interface =
        joint_interfaces_.insert(joint_interfaces_.end(), InterfaceData(joint_info.name));

      transmission_interface::JointHandle joint_handle(joint_info.name,
        hardware_interface::HW_IF_POSITION, &joint_interface->transmission_);
      joint_handles.push_back(joint_handle);
    }

    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    for (const auto & actuator_info : transmission_info.actuators) {
      // no check for actuators types?

      auto actuator_interface =
        actuator_interfaces_.insert(actuator_interfaces_.end(), InterfaceData(actuator_info.name));
      transmission_interface::ActuatorHandle actuator_handle(actuator_info.name,
        hardware_interface::HW_IF_POSITION, &actuator_interface->transmission_);
      actuator_handles.push_back(actuator_handle);
    }

    /// @note no need to store the joint and actuator handles, the
    /// transmission will keep whatever info it needs

    /// @todo add try/catch pair for this
    transmission->configure(joint_handles, actuator_handles);

    transmissions_.push_back(transmission);
  }

  RCLCPP_INFO(*logger_, "Initialization successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Configuring...");

//  for (int i = 0; i < hw_start_sec_; i++) {
//    rclcpp::sleep_for(std::chrono::seconds(1));
//    RCLCPP_INFO(*logger_, "%.1f seconds left...", hw_start_sec_ - i);
//  }

//  // reset values always when configuring hardware
//  for (uint i = 0; i < joint_states_.size(); i++) {
//    joint_states_[i] = 0;
//    joint_commands_[i] = 0;
//  }


  auto reset_interfaces = [](std::vector<InterfaceData> & interfaces)
    {
      for (auto & interface_data : interfaces) {
        interface_data.command_ = 0.0;
        interface_data.state_ = 0.0;
        interface_data.transmission_ = kNaN;
      }
    };

  reset_interfaces(joint_interfaces_);
  reset_interfaces(actuator_interfaces_);

  RCLCPP_INFO(*logger_, "Configuration successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotTransmissionsSystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto & joint : info_.joints) {
    /// @pre all joint interfaces exist, checked in on_init()
    auto joint_interface =
      std::find_if(
      joint_interfaces_.begin(), joint_interfaces_.end(),
      [&](const InterfaceData & interface) {return interface.name_ == joint.name;});

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name,
        hardware_interface::HW_IF_POSITION, &joint_interface->state_));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotTransmissionsSystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & joint : info_.joints) {
    /// @pre all joint interfaces exist, checked in on_init()
    auto joint_interface =
      std::find_if(
      joint_interfaces_.begin(), joint_interfaces_.end(),
      [&](const InterfaceData & interface) {return interface.name_ == joint.name;});

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name,
        hardware_interface::HW_IF_POSITION, &joint_interface->command_));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Activating...");

//  for (int i = 0; i < hw_start_sec_; i++) {
//    rclcpp::sleep_for(std::chrono::seconds(1));
//    RCLCPP_INFO(*logger_, "%.1f seconds left...", hw_start_sec_ - i);
//  }

//  // command and state should be equal when starting
//  for (uint i = 0; i < joint_states_.size(); i++) {
//    joint_commands_[i] = joint_states_[i];
//  }

  RCLCPP_INFO(*logger_, "Activation successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Deactivating...");

//  for (int i = 0; i < hw_stop_sec_; i++) {
//    rclcpp::sleep_for(std::chrono::seconds(1));
//    RCLCPP_INFO(*logger_, "%.1f seconds left...", hw_stop_sec_ - i);
//  }

  RCLCPP_INFO(*logger_, "Deactivation successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotTransmissionsSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
//  RCLCPP_INFO(*logger_, "Reading...");

//  for (uint i = 0; i < joint_states_.size(); i++) {
//    // Simulate RRBot's movement
//    joint_states_[i] = joint_states_[i] + (joint_commands_[i] - joint_states_[i]) / hw_slowdown_;
//    RCLCPP_INFO(*logger_, "Got state %.5f for joint %d!", joint_states_[i], i);
//  }
//  RCLCPP_INFO(*logger_, "Joints successfully read!");

  // actuator: state -> transmission
  std::for_each(
    actuator_interfaces_.begin(), actuator_interfaces_.end(),
    [](auto & actuator_interface) {
      actuator_interface.transmission_ = actuator_interface.state_;
    });

  // transmission: actuator -> joint
  std::for_each(
    transmissions_.begin(), transmissions_.end(), [](auto & transmission) {
      transmission->actuator_to_joint();
    });

  // joint: transmission -> state
  std::for_each(
    joint_interfaces_.begin(), joint_interfaces_.end(), [](auto & joint_interface) {
      joint_interface.state_ = joint_interface.transmission_;
    });

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotTransmissionsSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
//  RCLCPP_INFO(*logger_, "Writing...");

//  for (uint i = 0; i < joint_commands_.size(); i++) {
//    // Simulate sending commands to the hardware
//    RCLCPP_INFO(*logger_, "Got command %.5f for joint %d!", joint_commands_[i], i);
//  }
//  RCLCPP_INFO(*logger_, "Joints successfully written!");

  // joint: command -> transmission
  std::for_each(
    joint_interfaces_.begin(), joint_interfaces_.end(), [](auto & joint_interface) {
      joint_interface.transmission_ = joint_interface.command_;
    });

  // transmission: joint -> actuator
  std::for_each(
    transmissions_.begin(), transmissions_.end(), [](auto & transmission) {
      transmission->joint_to_actuator();
    });

  // actuator: transmission -> command
  std::for_each(
    actuator_interfaces_.begin(), actuator_interfaces_.end(),
    [](auto & actuator_interface) {
      actuator_interface.command_ = actuator_interface.transmission_;
    });

  // simulate motor motion
  /// @todo for now suppose a perfect actuator with infinite velocity,
  /// need to add ramping
  std::for_each(
    actuator_interfaces_.begin(), actuator_interfaces_.end(),
    [](auto & actuator_interface) {actuator_interface.state_ = actuator_interface.command_;});

  return hardware_interface::return_type::OK;
}

RRBotTransmissionsSystemPositionOnlyHardware::InterfaceData::InterfaceData(const std::string & name)
: name_(name), command_(kNaN), state_(kNaN), transmission_(kNaN)
{
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::RRBotTransmissionsSystemPositionOnlyHardware,
  hardware_interface::SystemInterface)
