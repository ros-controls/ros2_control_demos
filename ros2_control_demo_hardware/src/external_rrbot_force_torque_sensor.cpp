// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

//
// Authors: Subhas Das, Denis Stogl
//

#include "ros2_control_demo_hardware/external_rrbot_force_torque_sensor.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{
hardware_interface::return_type ExternalRRBotForceTorqueSensorHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_sensor_change_ = stod(info_.hardware_parameters["example_param_max_sensor_change"]);

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ExternalRRBotForceTorqueSensorHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("ExternalRRBotForceTorqueSensorHardware"),
    "Starting ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("ExternalRRBotForceTorqueSensorHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("ExternalRRBotForceTorqueSensorHardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ExternalRRBotForceTorqueSensorHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("ExternalRRBotForceTorqueSensorHardware"),
    "Stopping ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("ExternalRRBotForceTorqueSensorHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("ExternalRRBotForceTorqueSensorHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ExternalRRBotForceTorqueSensorHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("ExternalRRBotForceTorqueSensorHardware"), "Reading...");

  for (uint i = 0; i < hw_sensor_states_.size(); i++) {
    // Simulate RRBot's sensor data
    unsigned int seed = time(NULL) + i;
    hw_sensor_states_[i] =
      static_cast<float>(rand_r(&seed)) / (static_cast<float>(RAND_MAX / hw_sensor_change_));
    RCLCPP_INFO(
      rclcpp::get_logger("ExternalRRBotForceTorqueSensorHardware"), "Got state %e for sensor %zu!",
      hw_sensor_states_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("ExternalRRBotForceTorqueSensorHardware"),
    "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::ExternalRRBotForceTorqueSensorHardware,
  hardware_interface::SensorInterface)
