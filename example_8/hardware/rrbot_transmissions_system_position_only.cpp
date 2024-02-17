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

#include "ros2_control_demo_example_8/rrbot_transmissions_system_position_only.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

namespace ros2_control_demo_example_8
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  logger_ = std::make_unique<rclcpp::Logger>(
    rclcpp::get_logger("RRBotTransmissionsSystemPositionOnlyHardware"));

  clock_ = std::make_unique<rclcpp::Clock>();

  RCLCPP_INFO(*logger_, "Initializing...");

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  actuator_slowdown_ = hardware_interface::stod(info_.hardware_parameters["actuator_slowdown"]);

  const auto num_joints = std::accumulate(
    info_.transmissions.begin(), info_.transmissions.end(), 0ul,
    [](const auto & acc, const auto & trans_info) { return acc + trans_info.joints.size(); });

  const auto num_actuators = std::accumulate(
    info_.transmissions.begin(), info_.transmissions.end(), 0ul,
    [](const auto & acc, const auto & trans_info) { return acc + trans_info.actuators.size(); });

  // reserve the space needed for joint and actuator data structures
  joint_interfaces_.reserve(num_joints);
  actuator_interfaces_.reserve(num_actuators);

  // create transmissions, joint and actuator handles
  auto transmission_loader = transmission_interface::SimpleTransmissionLoader();

  for (const auto & transmission_info : info_.transmissions)
  {
    // only simple transmissions are supported in this demo
    if (transmission_info.type != "transmission_interface/SimpleTransmission")
    {
      RCLCPP_FATAL(
        *logger_, "Transmission '%s' of type '%s' not supported in this demo",
        transmission_info.name.c_str(), transmission_info.type.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::shared_ptr<transmission_interface::Transmission> transmission;
    try
    {
      transmission = transmission_loader.load(transmission_info);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
      RCLCPP_FATAL(
        *logger_, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<transmission_interface::JointHandle> joint_handles;
    for (const auto & joint_info : transmission_info.joints)
    {
      // this demo supports only one interface per joint
      if (!(joint_info.state_interfaces.size() == 1 &&
            joint_info.state_interfaces[0] == hardware_interface::HW_IF_POSITION &&
            joint_info.command_interfaces.size() == 1 &&
            joint_info.command_interfaces[0] == hardware_interface::HW_IF_POSITION))
      {
        RCLCPP_FATAL(
          *logger_, "Invalid transmission joint '%s' configuration for this demo",
          joint_info.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      const auto joint_interface =
        joint_interfaces_.insert(joint_interfaces_.end(), InterfaceData(joint_info.name));

      transmission_interface::JointHandle joint_handle(
        joint_info.name, hardware_interface::HW_IF_POSITION,
        &joint_interface->transmission_passthrough_);
      joint_handles.push_back(joint_handle);
    }

    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    for (const auto & actuator_info : transmission_info.actuators)
    {
      // no check for actuators types

      const auto actuator_interface =
        actuator_interfaces_.insert(actuator_interfaces_.end(), InterfaceData(actuator_info.name));
      transmission_interface::ActuatorHandle actuator_handle(
        actuator_info.name, hardware_interface::HW_IF_POSITION,
        &actuator_interface->transmission_passthrough_);
      actuator_handles.push_back(actuator_handle);
    }

    /// @note no need to store the joint and actuator handles, the transmission
    /// will keep whatever info it needs after is done with them

    try
    {
      transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
      RCLCPP_FATAL(
        *logger_, "Error while configuring %s: %s", transmission_info.name.c_str(), exc.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    transmissions_.push_back(transmission);
  }

  RCLCPP_INFO(*logger_, "Initialization successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Configuring...");

  auto reset_interfaces = [](std::vector<InterfaceData> & interfaces)
  {
    for (auto & interface_data : interfaces)
    {
      interface_data.command_ = 0.0;
      interface_data.state_ = 0.0;
      interface_data.transmission_passthrough_ = kNaN;
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
  for (const auto & joint : info_.joints)
  {
    /// @pre all joint interfaces exist, checked in on_init()
    auto joint_interface = std::find_if(
      joint_interfaces_.begin(), joint_interfaces_.end(),
      [&](const InterfaceData & interface) { return interface.name_ == joint.name; });

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &joint_interface->state_));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotTransmissionsSystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & joint : info_.joints)
  {
    /// @pre all joint interfaces exist, checked in on_init()
    auto joint_interface = std::find_if(
      joint_interfaces_.begin(), joint_interfaces_.end(),
      [&](const InterfaceData & interface) { return interface.name_ == joint.name; });

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &joint_interface->command_));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Activating...");
  RCLCPP_INFO(*logger_, "Activation successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotTransmissionsSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Deactivating...");
  RCLCPP_INFO(*logger_, "Deactivation successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotTransmissionsSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // actuator: state -> transmission
  std::for_each(
    actuator_interfaces_.begin(), actuator_interfaces_.end(),
    [](auto & actuator_interface)
    { actuator_interface.transmission_passthrough_ = actuator_interface.state_; });

  // transmission: actuator -> joint
  std::for_each(
    transmissions_.begin(), transmissions_.end(),
    [](auto & transmission) { transmission->actuator_to_joint(); });

  // joint: transmission -> state
  std::for_each(
    joint_interfaces_.begin(), joint_interfaces_.end(),
    [](auto & joint_interface)
    { joint_interface.state_ = joint_interface.transmission_passthrough_; });

  // log state data
  std::stringstream ss;
  ss << "State data:";
  for (const auto & transmission_info : info_.transmissions)
  {
    // again, this only for simple transmissions, we know there is only one joint
    const auto joint_interface = std::find_if(
      joint_interfaces_.cbegin(), joint_interfaces_.cend(),
      [&](const auto & joint_interface)
      { return joint_interface.name_ == transmission_info.joints[0].name; });

    const auto actuator_interface = std::find_if(
      actuator_interfaces_.cbegin(), actuator_interfaces_.cend(),
      [&](const auto & actuator_interface)
      { return actuator_interface.name_ == transmission_info.actuators[0].name; });

    const auto & reduction = transmission_info.joints[0].mechanical_reduction;

    ss << std::endl
       << "\t" << joint_interface->name_ << ": " << joint_interface->state_ << " <-- "
       << transmission_info.name << "(R=" << reduction << ") <-- " << actuator_interface->name_
       << ": " << actuator_interface->state_;
  }
  RCLCPP_INFO_THROTTLE(*logger_, *clock_, 1000, "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotTransmissionsSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // joint: command -> transmission
  std::for_each(
    joint_interfaces_.begin(), joint_interfaces_.end(),
    [](auto & joint_interface)
    { joint_interface.transmission_passthrough_ = joint_interface.command_; });

  // transmission: joint -> actuator
  std::for_each(
    transmissions_.begin(), transmissions_.end(),
    [](auto & transmission) { transmission->joint_to_actuator(); });

  // actuator: transmission -> command
  std::for_each(
    actuator_interfaces_.begin(), actuator_interfaces_.end(),
    [](auto & actuator_interface)
    { actuator_interface.command_ = actuator_interface.transmission_passthrough_; });

  // simulate motor motion
  std::for_each(
    actuator_interfaces_.begin(), actuator_interfaces_.end(),
    [&](auto & actuator_interface)
    {
      actuator_interface.state_ =
        actuator_interface.state_ +
        (actuator_interface.command_ - actuator_interface.state_) / actuator_slowdown_;
    });

  // log command data
  std::stringstream ss;
  ss << "Command data:";
  for (const auto & transmission_info : info_.transmissions)
  {
    // again, this only for simple transmissions, we know there is only one joint
    const auto joint_interface = std::find_if(
      joint_interfaces_.cbegin(), joint_interfaces_.cend(),
      [&](const auto & joint_interface)
      { return joint_interface.name_ == transmission_info.joints[0].name; });

    const auto actuator_interface = std::find_if(
      actuator_interfaces_.cbegin(), actuator_interfaces_.cend(),
      [&](const auto & actuator_interface)
      { return actuator_interface.name_ == transmission_info.actuators[0].name; });

    const auto & reduction = transmission_info.joints[0].mechanical_reduction;

    ss << std::endl
       << "\t" << joint_interface->name_ << ": " << joint_interface->command_ << " --> "
       << transmission_info.name << "(R=" << reduction << ") --> " << actuator_interface->name_
       << ": " << actuator_interface->command_;
  }
  RCLCPP_INFO_THROTTLE(*logger_, *clock_, 1000, "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

RRBotTransmissionsSystemPositionOnlyHardware::InterfaceData::InterfaceData(const std::string & name)
: name_(name), command_(kNaN), state_(kNaN), transmission_passthrough_(kNaN)
{
}

}  // namespace ros2_control_demo_example_8

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_8::RRBotTransmissionsSystemPositionOnlyHardware,
  hardware_interface::SystemInterface)
