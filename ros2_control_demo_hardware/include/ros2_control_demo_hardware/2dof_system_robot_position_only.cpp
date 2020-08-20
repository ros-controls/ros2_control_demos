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


#ifndef ROS2_CONTROL_DEMO_HARDWARE__ROBOT_MINIMAL_HARDWARE_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__ROBOT_MINIMAL_HARDWARE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "ros2_control_demo_hardware/visibility_control.h"

using hardware_interface::hardware_interface_ret_t;

namespace ros2_control_demo_minimal_hardware
{
class RobotMinimalHardware : public hardware_interface::RobotHardware
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotMinimalHardware);

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC RobotMinimalHardware() = default;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC ~RobotMinimalHardware() = default;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC hardware_interface_ret_t init();

  hardware_interface_ret_t read();

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC hardware_interface_ret_t write();

protected:
};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_DEMO_HARDWARE__ROBOT_MINIMAL_HARDWARE_HPP_
