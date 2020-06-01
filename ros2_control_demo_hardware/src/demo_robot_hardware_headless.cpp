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


#include "ros2_control_demo_hardware/demo_robot_hardware_headless.hpp"

using namespace ros2_control_demo_hardware_headless;

ros2_control_types::return_type DemoRobotHardwareHeadless::init_hardware()
{
  RCLCPP_INFO(logging_interface_->get_logger(), "Initalizing DemoRobotHardwareHeadless ...please wait...");

  for (int i=0; i < 3; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(logging_interface_->get_logger(), "%d seconds left...", 3-i);
  }

  RCLCPP_INFO(logging_interface_->get_logger(), "DemoRobotHardwareHeadless sucessfully initalized!");

  return ros2_control_types::ROS2C_RETURN_OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware_headless::DemoRobotHardwareHeadless, ros2_control_core_hardware::RobotHardware)
