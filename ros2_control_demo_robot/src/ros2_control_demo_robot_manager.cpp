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

#include "rclcpp/rclcpp.hpp"

#include "ros2_control_core/components/robot.hpp"


using namespace std::chrono_literals;


class ROS2ControlCoreTestClass: public rclcpp::Node
{
public:
  ROS2ControlCoreTestClass(rclcpp::NodeOptions options) : Node("ros2_control_core_test_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Here start... options");

    robot_.reset(new ros2_control_core_components::Robot(std::string("TestRobot"), this->get_node_logging_interface(), this->get_node_parameters_interface(), this->get_node_services_interface(), std::string("DemoRobot")));

    timer_ = this->create_wall_timer(1000ms, std::bind(&ROS2ControlCoreTestClass::loop, this));
  }

  void loop()
  {
    robot_->recover();

    RCLCPP_INFO(this->get_logger(), "Calling loop...");
  }

//   std::map<std::string, rclcpp::Parameter> create_param_tree()
//   {
//     std::map<std::string, rclcpp::Parameter> params;
//
//     rclcpp::Parameter param;
//     std::pair<std::string, std::string> pair;
//
//     params.insert( std::pair<std::string, std::string>(
//
//   }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ros2_control_core_components::Robot> robot_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<ROS2ControlCoreTestClass>(options));
  rclcpp::shutdown();
  return 0;
}
