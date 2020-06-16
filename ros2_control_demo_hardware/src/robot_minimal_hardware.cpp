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


#include "ros2_control_demo_hardware/robot_minimal_hardware.hpp"

namespace ros2_control_demo_minimal_hardware
{

hardware_interface::hardware_interface_ret_t RobotMinimalHardware::init()
{
  RCLCPP_INFO(rclcpp::get_logger("robot_hardware"), "Initalizing RobotMinimalHardware ...please wait...");

  for (int i=0; i < 3; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("robot_hardware"), "%d seconds left...", 3-i);
  }

  RCLCPP_INFO(rclcpp::get_logger("robot_hardware"), "RobotMinimalHardware sucessfully initalized!");

  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t RobotMinimalHardware::read()
{
//   RCLCPP_INFO(rclcpp::get_logger(), "Initalizing RobotMinimalHardware ...please wait...");

  for (int i=0; i < 3; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    //     RCLCPP_INFO(logging_interface_->get_logger(), "%d seconds left...", 3-i);
  }

  //   RCLCPP_INFO(logging_interface_->get_logger(), "RobotMinimalHardware sucessfully initalized!");

  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t RobotMinimalHardware::write()
{
  //   RCLCPP_INFO(logging_interface_->get_logger(), "Initalizing RobotMinimalHardware ...please wait...");

  for (int i=0; i < 3; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    //     RCLCPP_INFO(logging_interface_->get_logger(), "%d seconds left...", 3-i);
  }

  //   RCLCPP_INFO(logging_interface_->get_logger(), "RobotMinimalHardware sucessfully initalized!");

  return hardware_interface::HW_RET_OK;
}

} //  namespace ros2_control_demo_minimal_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_minimal_hardware::RobotMinimalHardware, hardware_interface::RobotHardwareInterface)
