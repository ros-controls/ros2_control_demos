#include "control_demos_headless/generic_robot_hardware.hpp"

GenericRobotHardware::GenericRobotHardware()
{
  loadUrdf();
}

void GenericRobotHardware:loadUrdf()
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
  std::string urdf_string= get_parameter("robot_description");
}



      
