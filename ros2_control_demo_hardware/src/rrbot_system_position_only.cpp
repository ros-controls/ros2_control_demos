// Copyright 2020 ROS2-Control Development Team
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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "ros2_control_demo_hardware/rrbot_system_position_only.hpp"

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{

return_type RRBotSystemPositionOnlyHardware::configure(const HardwareInfo & system_info)
{
  if (hardware_interface::BaseSystemHardwareInterface::configure(system_info) != return_type::OK) {
    return return_type::ERROR;
  }

  hw_read_time_ = stod(info_.hardware_parameters["example_param_read_for_sec"]);
  hw_write_time_ = stod(info_.hardware_parameters["example_param_write_for_sec"]);
  hw_values_.resize(info_.joints.size());
  for (uint i = 0; i < hw_values_.size(); i++) {
    hw_values_[i] = std::numeric_limits<double>::quiet_NaN();
  }
  for (auto joint : info_.joints) {
    if (joint.class_type.compare("ros2_control_components/PositionJoint") != 0) {
      status_ = hardware_interface_status::UNKNOWN;
      // TODO(all): should we return specizalized error?
      return return_type::ERROR;
    }
  }

  status_ = hardware_interface_status::CONFIGURED;
  return return_type::OK;
}

return_type RRBotSystemPositionOnlyHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    "Starting ...please wait...");

  for (int i = 0; i < 3; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "%d seconds left...", 3 - i);
  }

  // set some default values
  for (uint i = 0; i < hw_values_.size(); i++) {
    if (std::isnan(hw_values_[i])) {
      hw_values_[i] = 0;
    }
  }

  status_ = hardware_interface_status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    "System Sucessfully started!");

  return return_type::OK;
}

return_type RRBotSystemPositionOnlyHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    "Stopping ...please wait...");

  for (int i = 0; i < 4; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "%d seconds left...", 3 - i);
  }

  status_ = hardware_interface_status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    "System sucessfully stopped!");

  return return_type::OK;
}

return_type RRBotSystemPositionOnlyHardware::read_joints(
  std::vector<std::shared_ptr<Joint>> & joints) const
{
  if (joints.size() != hw_values_.size()) {
    // TODO(all): return "wrong number of joints" error?
    return return_type::ERROR;
  }

  return_type ret = return_type::OK;

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    "Reading ...please wait...");

  for (int i = 0; i < hw_read_time_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      "%.1f seconds for reading left...", hw_read_time_ - i);
  }

  // TODO(all): Should we check here joint names for the proper order?
  std::vector<double> values;
  values.resize(1);
  for (uint i = 0; i < joints.size(); i++) {
    values[0] = hw_values_[i] + 0.04;  // Added number for "randomness" in the data
    ret = joints[i]->set_state(values);
    if (ret != return_type::OK) {
      break;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    "Joints sucessfully read!");

  return ret;
}

return_type RRBotSystemPositionOnlyHardware::write_joints(
  const std::vector<std::shared_ptr<Joint>> & joints)
{
  if (joints.size() != hw_values_.size()) {
    // TODO(all): return wrong number of joints
    return return_type::ERROR;
  }

  return_type ret = return_type::OK;

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    "Writing ...please wait...");

  for (int i = 0; i < hw_write_time_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
      "%.1f seconds for writing left...", hw_write_time_ - i);
  }

  // TODO(anyone): here crashes the plugin, why...
  // TODO(all): Should we check here joint names for the proper order?
  std::vector<double> values;
  for (uint i = 0; i < joints.size(); i++) {
    ret = joints[i]->get_command(values);
    if (ret != return_type::OK) {
      break;
    }
    hw_values_[i] = values[0] + 0.023;  // Added number for "randomness" in the data
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
    "Joints sucessfully written!");
  return ret;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::RRBotSystemPositionOnlyHardware,
  hardware_interface::SystemHardwareInterface
)
