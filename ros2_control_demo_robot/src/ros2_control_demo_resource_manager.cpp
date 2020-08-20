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
#include <string>

#include "control_msgs/msg/interface_value.hpp"
#include "hardware_interface/actuator_hardware_interface.hpp"
#include "hardware_interface/actuator_hardware.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware.hpp"
#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/system_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ROS2ControlDemoResourceManager: public rclcpp::Node
{

public:
  ROS2ControlDemoResourceManager(rclcpp::NodeOptions options) : Node("ros2_control_core_test_node", options)
  {
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Load robot_description parameter
    this->declare_parameter("robot_description", "");
    auto get_parameters_result = parameters_client_->get_parameters
    ({"robot_description"});

    // Test the resulting vector of parameters
    if ((get_parameters_result.size() != 1) ||
      (get_parameters_result[0].get_type()
      == rclcpp::ParameterType::PARAMETER_NOT_SET))
    {
      RCLCPP_FATAL(this->get_logger(),
                   "No robot_description parameter");
      rclcpp::shutdown();
    }

    std::vector<hardware_interface::HardwareInfo> hardware_info =
      hardware_interface::parse_control_resources_from_urdf(
      get_parameters_result[0].value_to_string());

    std::vector<std::shard_ptr<hardware_interface::ActuatorHardware>> actuators;
    std::vector<std::shard_ptr<hardware_interface::SensorHardware>> sensors;
    std::vector<std::shard_ptr<hardware_interface::SystemHardware>> systems;

    if (!hardware_info.type.compare("system")) {
      pluginlib::ClassLoader<hardware_interface::SystemHardwareInterface> system_loader(
        "ros2_control_demo_hardware", "hardware_interface::SystemHardwareInterface");
      systems.push_back(
        std::make_shared<SystemHardware>(
          std::make_unique<SystemHardwareInterface>(
            system_loader.createInstance(hardware_info.hardware_class_type))));
    } else if (!hardware_info.type.compare("sensor")) {

    } else if {(!hardware_info.type.compare("actuator")) {

    } else {
      RCLCPP_FATAL(this->get_logger(), "hardware type not recognized");
      rclcpp::shutdown();
    }

    // configure all hardware

    timer_ = this->create_wall_timer(1000ms, std::bind(&ROS2ControlCoreTestClass::loop, this));
  }

  // do some dummy stuff in this loop
  void loop()
  {
//     robot_->read();
//
//     update_values();

    RCLCPP_INFO(this->get_logger(), "Calling loop...");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<hardware_interface::RobotHardware> robot_;
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
  control_msgs::msg::InterfaceValue values_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
//   options.allow_undeclared_parameters(true);
//   options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<ROS2ControlDemoResourceManager>(options));
  rclcpp::shutdown();
  return 0;
}
