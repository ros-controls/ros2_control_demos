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


#ifndef ROS2_CONTROL_DEMO_HARDWARE__DEMO_ROBOT_HARDWARE_HEADLESS_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__DEMO_ROBOT_HARDWARE_HEADLESS_HPP_

#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_core/hardware/robot_hardware.hpp"
#include "ros2_control_demo_hardware/visibility_control.h"


namespace ros2_control_demo_hardware_headless
{
class DemoRobotHardwareHeadless : public ros2_control_core_hardware::RobotHardware
{
public:
  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC DemoRobotHardwareHeadless() = default;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC ~DemoRobotHardwareHeadless() = default;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC ros2_control_types::return_type init_hardware();

protected:
};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_DEMO_HARDWARE__DEMO_ROBOT_HARDWARE_HEADLESS_HPP_
