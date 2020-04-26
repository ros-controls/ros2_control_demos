// Copyright 2020 CNRS
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

/* Author: Olivier Stasse
   Desc:   Example ros2_generic_robot_hardware interface that performs
   load parameters and creates a generic robot.
*/

#ifndef ROS2_CONTROL_DEMOS_GENERIC_ROBOT_HARDWARE_HPP_
#define ROS2_CONTROL_DEMOS_GENERIC_ROBOT_HARDWARE_HPP_

#include "hardware_interface/robot_hardware.hpp"

#include "rclcpp/rclcpp.hpp"

#include "control_demos_robot_headless/visibility_control.h"

#include <urdf/model.h>

namespace control_demos
{

namespace robot_headless
{

class GenericRobotHardware: public hardware_interface::RobotHardware,
                            public rclcpp::Node
{
 public:
  
  GenericRobotHardware();
  virtual ~GenericRobotHardware();

  void loadURDF();

  hardware_interface::hardware_interface_ret_t init();
  hardware_interface::hardware_interface_ret_t read();
  hardware_interface::hardware_interface_ret_t write();
  
 protected:
  urdf::Model *urdf_model_;
};

} // namespace robot_headless
} // namespace control_demos
#endif /* ROS2_CONTROL_DEMOS_GENERIC_ROBOT_HARDWARE_HPP_ */


