#include <chrono>

#include "control_demos_robot_headless/generic_robot_hardware.hpp"

using namespace std::chrono_literals;

namespace control_demos
{

namespace robot_headless
{

GenericRobotHardware::GenericRobotHardware():
    rclcpp::Node::Node("generic_robot_hardware")
{
  loadURDF();
}

GenericRobotHardware::~GenericRobotHardware()
{
}

void GenericRobotHardware::loadURDF()
{
  
  // Wait for rosparemeter service
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  // Load parameter
  auto  urdf_string= get_parameter("robot_description");
}

hardware_interface::hardware_interface_ret_t
GenericRobotHardware::init()
{
  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t
GenericRobotHardware::read()
{
  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t
GenericRobotHardware::write()
{
  return hardware_interface::HW_RET_OK;
}


}
}
