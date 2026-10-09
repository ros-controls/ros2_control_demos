// Copyright 2026 ros2_control Development Team
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

#include "ros2_control_demo_example_18/socketcan_diff_drive.hpp"

#include <cmath>
#include <memory>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_18
{
std::unique_ptr<CanTransport> SocketCanDiffDriveHardware::create_transport()
{
  return std::make_unique<SocketCanTransport>();
}

hardware_interface::CallbackReturn SocketCanDiffDriveHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (
    info_.joints.size() != 2 || info_.joints[0].command_interfaces.size() != 1 ||
    info_.joints[1].command_interfaces.size() != 1)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints)
  {
    if (
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY ||
      joint.state_interfaces.size() != 2 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (info_.joints[0].name == info_.joints[1].name)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  left_position_name_ = info_.joints[0].name + "/" + hardware_interface::HW_IF_POSITION;
  left_velocity_name_ = info_.joints[0].name + "/" + hardware_interface::HW_IF_VELOCITY;
  right_position_name_ = info_.joints[1].name + "/" + hardware_interface::HW_IF_POSITION;
  right_velocity_name_ = info_.joints[1].name + "/" + hardware_interface::HW_IF_VELOCITY;
  const auto it = info_.hardware_parameters.find("can_interface");
  can_interface_ = it == info_.hardware_parameters.end() ? "can0" : it->second;
  const auto timeout = info_.hardware_parameters.find("feedback_timeout_sec");
  if (timeout != info_.hardware_parameters.end())
  {
    try
    {
      feedback_timeout_sec_ = std::stod(timeout->second);
    }
    catch (...)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (!std::isfinite(feedback_timeout_sec_) || feedback_timeout_sec_ <= 0.0)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  transport_ = create_transport();
  if (!transport_)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SocketCanDiffDriveHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  for (const auto & [name, _] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, _] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  feedback_age_sec_ = 0.0;
  if (!transport_->open(can_interface_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to open SocketCAN interface '%s'", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn SocketCanDiffDriveHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn SocketCanDiffDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (transport_)
  {
    if (!transport_->send({0.0, 0.0}))
    {
      RCLCPP_WARN(get_logger(), "Failed to send the best-effort zero command during deactivation");
    }
    transport_->close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SocketCanDiffDriveHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (period.seconds() < 0.0 || !std::isfinite(period.seconds()))
  {
    return hardware_interface::return_type::ERROR;
  }
  if (!transport_)
  {
    return hardware_interface::return_type::ERROR;
  }
  WheelFeedback feedback;
  const auto result = transport_->receive(feedback);
  if (result == CanTransport::ReceiveResult::ERROR)
  {
    return hardware_interface::return_type::ERROR;
  }
  if (result == CanTransport::ReceiveResult::OK)
  {
    if (!std::isfinite(feedback.left_velocity) || !std::isfinite(feedback.right_velocity))
    {
      return hardware_interface::return_type::ERROR;
    }
    feedback_age_sec_ = 0.0;
    set_state(left_velocity_name_, feedback.left_velocity);
    set_state(right_velocity_name_, feedback.right_velocity);
    set_state(
      left_position_name_,
      get_state(left_position_name_) + feedback.left_velocity * period.seconds());
    set_state(
      right_position_name_,
      get_state(right_position_name_) + feedback.right_velocity * period.seconds());
  }
  else
  {
    feedback_age_sec_ += period.seconds();
    if (feedback_age_sec_ > feedback_timeout_sec_)
    {
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SocketCanDiffDriveHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const WheelCommand command{get_command(left_velocity_name_), get_command(right_velocity_name_)};
  if (
    !transport_ || !std::isfinite(command.left_velocity) ||
    !std::isfinite(command.right_velocity) || !transport_->send(command))
  {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}
}  // namespace ros2_control_demo_example_18

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_18::SocketCanDiffDriveHardware, hardware_interface::SystemInterface)
