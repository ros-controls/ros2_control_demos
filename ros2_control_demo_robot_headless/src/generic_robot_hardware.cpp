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
#include "control_demos_robot_headless/generic_robot_hardware.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace control_demos {

namespace robot_headless {

GenericRobotHardware::GenericRobotHardware()
    : rclcpp::Node::Node("generic_robot_hardware") {
  loadURDF();
}

GenericRobotHardware::~GenericRobotHardware() {}

void GenericRobotHardware::loadURDF() {
  // Wait for rosparemeter service
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      this, "robot_state_publisher");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  // Load parameter
  rclcpp::Parameter urdf_string;
  get_parameter_or("robot_description", urdf_string, rclcpp::Parameter(""));

  // Display the parameter:
  std::cout << urdf_string << std::endl;
}

hardware_interface::hardware_interface_ret_t GenericRobotHardware::init() {
  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t GenericRobotHardware::read() {
  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t GenericRobotHardware::write() {
  return hardware_interface::HW_RET_OK;
}

}  // namespace robot_headless
}  // namespace control_demos
