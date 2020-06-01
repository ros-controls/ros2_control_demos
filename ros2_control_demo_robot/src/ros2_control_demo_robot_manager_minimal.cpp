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
    robot_.reset(new ros2_control_core_components::Robot());
    robot_->configure(std::string("DemoRobot"), this->get_node_logging_interface(), this->get_node_parameters_interface(), this->get_node_services_interface());

    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    robot_->init();

    timer_ = this->create_wall_timer(1000ms, std::bind(&ROS2ControlCoreTestClass::loop, this));
  }

  void loop()
  {
//     robot_->read();
//
//     update_values();



    robot_->recover();

    RCLCPP_INFO(this->get_logger(), "Calling loop...");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ros2_control_core_components::Robot> robot_;
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
