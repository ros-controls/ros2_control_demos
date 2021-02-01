#include "ros2_control_demo_hardware/rrbot_system_multi_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{

return_type RRBotSystemMultiInterfaceHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // RRBotSystemMultiInterface has exactly 3 state interfaces and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has %d command interfaces. 3 expected.",
        joint.name.c_str());
      return return_type::ERROR;
    }
    
    if ( !(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
           joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
           joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_ACCELERATION);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s'has %d state interfaces. 3 expected.",
        joint.name.c_str());
      return return_type::ERROR;
    }

    if ( !(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
           joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
           joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION) ) {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_ACCELERATION);
      return return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_accelerations_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_commands_accelerations_[i]));
  }

  return command_interfaces;
}

return_type RRBotSystemMultiInterfaceHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
    "Starting... please wait...");
  
  for (int i = 0; i <= hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
      "%.1f seconds left...", hw_start_sec_ -i);
  }

  // Set some default values
  for (uint i = 0; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
    }
    if (std::isnan(hw_velocities_[i])) {
      hw_velocities_[i] = 0;
    }
    if (std::isnan(hw_accelerations_[i])) {
      hw_accelerations_[i] = 0;
    }
    if (std::isnan(hw_commands_positions_[i])) {
      hw_commands_positions_[i] = 0;
    }
    if (std::isnan(hw_commands_velocities_[i])) {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_accelerations_[i])) {
      hw_commands_accelerations_[i] = 0;
    }
  }
  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
    "System succesfully started!");
  return return_type::OK;
}

return_type RRBotSystemMultiInterfaceHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
    "Stopping... please wait...");
  
  for (int i = 0; i <= hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
      "%.1f seconds left...", hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
    "System succesfully stopped!");
  
  return return_type::OK;
}



return_type RRBotSystemMultiInterfaceHardware::read()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
    "Reading...");
  
  for (uint  i = 0; i < hw_positions_.size(); i++) {
    // Simulate RRBot's movement, this has three scenarios actually:
    // 1. The position command interfaces have been claimed, update according to them
    // 2. The velocity command interfaces have been claimed, update according to them
    // 3. The acceleration command interfaces have been claimed, update according to them
    // Since I don't know how to figure out which resource has been claimed:
    hw_positions_[i] = hw_commands_positions_[i] + (hw_positions_[i] - hw_commands_positions_[i]) / hw_slowdown_;
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
      "Got pos: %.5f, vel: %.5f, acc: %.5f for joint %d!",
      hw_positions_[i], hw_velocities_[i],
      hw_accelerations_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
    "Joints succesfully read!");

  return return_type::OK;
}

return_type RRBotSystemMultiInterfaceHardware::write()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
    "Writing...");
  for (uint i = 0; i < hw_commands_positions_.size(); i++) {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
      "Got the commands pos: %.5f, vel: %.5f, acc: %.5f for joint %d", 
      hw_commands_positions_[i], hw_commands_velocities_[i],
      hw_commands_accelerations_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
    "Joint succesfully written!");
  return return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::RRBotSystemMultiInterfaceHardware,
  hardware_interface::SystemInterface
)