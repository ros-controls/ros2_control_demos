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
#include "hardware_interface/robot_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_control_components/ros2_control_utils.hpp"


// use virtual representation of Robot in the future and not RobotHardwareInterface directly
// #include "ros2_control_components/robot.hpp"

using namespace std::chrono_literals;


class ROS2ControlCoreTestClass: public rclcpp::Node
{
typedef ros2_control_utils::ROS2ControlLoaderPluginlib<hardware_interface::RobotHardwareInterface> RobotHardwareInterfaceLoaderType;

public:
  ROS2ControlCoreTestClass(rclcpp::NodeOptions options) : Node("ros2_control_core_test_node", options)
  {
    RobotHardwareInterfaceLoaderType robot_hardware_loader = RobotHardwareInterfaceLoaderType("hardware_interface", "hardware_interface/RobotMinimalHardware");

    robot_ = robot_hardware_loader.create("ros2_control_demo_minimal_hardware/RobotMinimalHardware");

    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Load robot_description parameter
    auto get_parameters_result = parameters_client_->get_parameters
    ({"robot_description"});

    // Test the resulting vector of parameters
    if ((get_parameters_result.size() != 1) ||
      (get_parameters_result[0].get_type()
      == rclcpp::ParameterType::PARAMETER_NOT_SET))
    {
      RCLCPP_ERROR(get_logger(),
                   "No robot_description parameter");

      return;
    }

    robot_->init();

    timer_ = this->create_wall_timer(1000ms, std::bind(&ROS2ControlCoreTestClass::loop, this));
  }

  void loop()
  {
//     robot_->read();
//
//     update_values();

    RCLCPP_INFO(this->get_logger(), "Calling loop...");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<hardware_interface::RobotHardwareInterface> robot_;
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
  control_msgs::msg::InterfaceValue values_;

//   void update_values()
//   {
//
//   }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
//   options.allow_undeclared_parameters(true);
//   options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<ROS2ControlCoreTestClass>(options));
  rclcpp::shutdown();
  return 0;
}
