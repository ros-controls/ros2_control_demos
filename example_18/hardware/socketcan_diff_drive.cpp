#include "ros2_control_demo_example_18/socketcan_diff_drive.hpp"

#include <cmath>
#include <memory>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_18
{
hardware_interface::CallbackReturn SocketCanDiffDriveHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;
  if (info_.joints.size() != 2 || info_.joints[0].command_interfaces.size() != 1 || info_.joints[1].command_interfaces.size() != 1) return hardware_interface::CallbackReturn::ERROR;
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY || joint.state_interfaces.size() != 2 || joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION || joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) return hardware_interface::CallbackReturn::ERROR;
  }
  const auto it = info_.hardware_parameters.find("can_interface");
  can_interface_ = it == info_.hardware_parameters.end() ? "can0" : it->second;
  transport_ = std::make_unique<SocketCanTransport>();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SocketCanDiffDriveHardware::on_configure(const rclcpp_lifecycle::State &)
{
  for (const auto & [name, _] : joint_state_interfaces_) set_state(name, 0.0);
  for (const auto & [name, _] : joint_command_interfaces_) set_command(name, 0.0);
  if (!transport_->open(can_interface_)) { RCLCPP_ERROR(get_logger(), "Failed to open SocketCAN interface '%s'", can_interface_.c_str()); return hardware_interface::CallbackReturn::ERROR; }
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn SocketCanDiffDriveHardware::on_activate(const rclcpp_lifecycle::State &) { return hardware_interface::CallbackReturn::SUCCESS; }
hardware_interface::CallbackReturn SocketCanDiffDriveHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  transport_->send({0.0, 0.0}); transport_->close(); return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SocketCanDiffDriveHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  WheelFeedback feedback;
  const auto result = transport_->receive(feedback);
  if (result == CanTransport::ReceiveResult::ERROR) return hardware_interface::return_type::ERROR;
  if (result == CanTransport::ReceiveResult::OK) {
    const auto left = info_.joints[0].name; const auto right = info_.joints[1].name;
    set_state(left + "/" + hardware_interface::HW_IF_VELOCITY, feedback.left_velocity);
    set_state(right + "/" + hardware_interface::HW_IF_VELOCITY, feedback.right_velocity);
    set_state(left + "/" + hardware_interface::HW_IF_POSITION, get_state(left + "/" + hardware_interface::HW_IF_POSITION) + feedback.left_velocity * period.seconds());
    set_state(right + "/" + hardware_interface::HW_IF_POSITION, get_state(right + "/" + hardware_interface::HW_IF_POSITION) + feedback.right_velocity * period.seconds());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SocketCanDiffDriveHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto left = info_.joints[0].name; const auto right = info_.joints[1].name;
  if (!transport_->send({get_command(left + "/" + hardware_interface::HW_IF_VELOCITY), get_command(right + "/" + hardware_interface::HW_IF_VELOCITY)})) return hardware_interface::return_type::ERROR;
  return hardware_interface::return_type::OK;
}
}  // namespace ros2_control_demo_example_18

PLUGINLIB_EXPORT_CLASS(ros2_control_demo_example_18::SocketCanDiffDriveHardware, hardware_interface::SystemInterface)
