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

#include "ros2_control_demo_hardware/rrbot_system_with_sensor.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{

return_type RRBotSystemWithSensorHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  hw_read_sec_ = stod(info_.hardware_parameters["example_param_read_for_sec"]);
  hw_write_sec_ = stod(info_.hardware_parameters["example_param_write_for_sec"]);
  hw_joint_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_joint_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(),
                          std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // RRBotSystemWithSensor has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithSensorHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithSensorHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithSensorHardware"),
        "Joint '%s' has %d state interface. 1 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemWithSensorHardware"),
        "Joint '%s' have %s state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }
  }

  // RRBotSystemWithSensor has six state interfaces for one sensor
  if (info_.sensors[0].state_interfaces.size() != 6) {
    RCLCPP_FATAL(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"),
      "Sensor '%s' has %d state interface. 6 expected.",
      info_.sensors[0].name.c_str(), info_.sensors[0].state_interfaces.size());
    return return_type::ERROR;
  }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemWithSensorHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_states_[i]));
  }

  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++) {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemWithSensorHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_commands_[i]));
  }

  return command_interfaces;
}

return_type RRBotSystemWithSensorHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemWithSensorHardware"),
    "Starting ...please wait...");

  // set some default values for joints
  for (uint i = 0; i < hw_joint_states_.size(); i++) {
    if (std::isnan(hw_joint_states_[i])) {
      hw_joint_states_[i] = 0;
      hw_joint_commands_[i] = 0;
    }
  }

  // set default value for sensor
  if (std::isnan(hw_sensor_states_[0])) {
    hw_sensor_states_[0] = 0;
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemWithSensorHardware"),
    "System Sucessfully started!");

  return return_type::OK;
}

return_type RRBotSystemWithSensorHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemWithSensorHardware"),
    "Stopping ...please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemWithSensorHardware"),
    "System sucessfully stopped!");

  return return_type::OK;
}

hardware_interface::return_type RRBotSystemWithSensorHardware::read()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemWithSensorHardware"),
    "Reading...please wait...");
  for (int i = 0; i <= hw_read_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"),
      "%.1f seconds left...", hw_read_sec_ - i);
  }

  for (uint i = 0; i < hw_joint_states_.size(); i++) {
    // Simulate RRBot's movement
    hw_joint_states_[i] = hw_joint_commands_[i] +
                          (hw_joint_states_[i] - hw_joint_commands_[i]) / 2.0;
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"),
      "Got state %.5f for joint %d!", hw_joint_states_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemWithSensorHardware"),
    "Joints sucessfully read!");

  // Simulate RRBot's sensor data
  hw_sensor_states_[0] = 1.1;
  RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"),
      "Got state %.5f for sensor 0!", hw_sensor_states_[0]);
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemWithSensorHardware"),
    "Sensors sucessfully read!");

  return return_type::OK;
}

hardware_interface::return_type ros2_control_demo_hardware::RRBotSystemWithSensorHardware::write()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemWithSensorHardware"),
    "Writing...please wait...");

  for (int i = 0; i <= hw_write_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"),
      "%.1f seconds left...", hw_write_sec_ - i);
  }

  for (uint i = 0; i < hw_joint_commands_.size(); i++) {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemWithSensorHardware"),
      "Got command %.5f for joint %d!", hw_joint_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemWithSensorHardware"),
    "Joints sucessfully written!");

  return return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::RRBotSystemWithSensorHardware,
  hardware_interface::SystemInterface
)
