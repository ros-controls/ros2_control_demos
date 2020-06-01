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


#ifndef ROS2_CONTROL_DEMO_COMMUNICATION__DEMO_ROBOT_HEADLESS_HPP_
#define ROS2_CONTROL_DEMO_COMMUNICATION__DEMO_ROBOT_HEADLESS_HPP_

#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_core/communication_interface/hardware_communication_interface.hpp"
#include "ros2_control_core/ros2_control_types.h"
#include "ros2_control_demo_communication_headless/visibility_control.h"


namespace ros2_control_demo_communication_headless
{
class DemoRobotHeadless : public ros2_control_core_communication_interface::HardwareCommunicationInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DemoRobotHeadless);

  ROS2_CONTROL_DEMO_COMMUNICATION_HEADLESS_PUBLIC DemoRobotHeadless() = default;

  ROS2_CONTROL_DEMO_COMMUNICATION_HEADLESS_PUBLIC virtual ~DemoRobotHeadless() = default;

  ROS2_CONTROL_DEMO_COMMUNICATION_HEADLESS_PUBLIC ros2_control_types::return_type configure(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface);

  ROS2_CONTROL_DEMO_COMMUNICATION_HEADLESS_PUBLIC ros2_control_types::return_type init();

protected:
};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_DEMO_COMMUNICATION__DEMO_ROBOT_HEADLESS_HPP_
