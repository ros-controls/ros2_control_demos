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
using std::placeholders::_1;

namespace control_demos {

namespace robot_headless {

GenericRobotHardware::GenericRobotHardware()
    : rclcpp::Node::Node("generic_robot_hardware") {

  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/robot_description",
    1, std::bind(&GenericRobotHardware::loadURDFCallback,
                 this, _1));
  
  getURDFParameter();
  
}

GenericRobotHardware::~GenericRobotHardware()
{}

void GenericRobotHardware::loadURDFCallback
(const std_msgs::msg::String::SharedPtr msg)
{
  std::string urdf_string = msg->data;
  loadURDFString(urdf_string);
}

void GenericRobotHardware::getURDFParameter()
{
  // Synchronized access to robot_state_publisher node
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
  
  // Load parameter robot_description
  auto get_parameters_result = parameters_client->get_parameters
               ({"robot_description"});

  // Test the resulting vector of parameters
  if ((get_parameters_result.size() != 1) ||
      (get_parameters_result[0].get_type()
       == rclcpp::ParameterType::PARAMETER_NOT_SET))
  {
    RCLCPP_ERROR(get_logger(),
                 "No /robot_state_publisher/robot_description parameter");
    
    return;
  }

  // Load the urdf model.
  std::string urdf_value = get_parameters_result[0].value_to_string();
  loadURDFString(urdf_value);
  
}

void GenericRobotHardware::loadURDFString(std::string &urdf_value)
{
  if (!urdf_model_.initString(urdf_value)) {
    RCLCPP_ERROR(get_logger(), "Unable to load URDF model");
    return;
  }
  else  {
    RCLCPP_DEBUG(get_logger(), "Received URDF from param server");
  }

  RCLCPP_INFO(get_logger(),urdf_value);
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
