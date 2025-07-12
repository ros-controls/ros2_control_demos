// Copyright 2020 ros2_control Development Team
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

#include "ros2_control_demo_example_17/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_17
{
hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get Default Node added to executor
  auto default_node = get_node();
  if (default_node)
  {
    default_status_publisher_ =
      default_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/rrbot_diagnostics", 10);
    auto default_timer_callback = [this]() -> void
    {
      if (default_status_publisher_)
      {
        auto diagnostic_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
        diagnostic_msg->header.staamp = get_node()->now();
        diagnostic_msg->status.resize(1);

        // Populate the status
        diagnostic_msg->status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diagnostic_msg->status[0].name = this->get_name();
        diagnostic_msg->status[0].message = "Hardware is OK";

        // Publish the message
        default_status_publisher_->publish(std::move(diagnostic_msg));
      }
    };
    using namespace std::chrono_literals;
    default_status_timer_ = default_node->create_wall_timer(2.5s, default_timer_callback);
  }

  // Get Weak Pointer to Executor from HardwareComponentInterfaceParams
  executor_ = params.executor;

  // Ensure that the executor is available before creating the custom status node
  if (auto locked_executor = executor_.lock())
  {
    std::string node_name = info_.name + "_custom_node";
    custom_status_node_ = std::make_shared<rclcpp::Node>(node_name);

    custom_status_publisher_ =
      custom_status_node_->create_publisher<std_msgs::msg::String>("rrbot_custom_status", 10);

    auto custom_timer_callback = [this]() -> void
    {
      if (custom_status_publisher_)
      {
        std_msgs::msg::String msg;
        msg.data = "RRBot '" + info_.name + "' custom node is alive at " +
                   std::to_string(custom_status_node_->now().seconds());
        custom_status_publisher_->publish(std::move(msg));
      }
    };

    using namespace std::chrono_literals;
    custom_status_timer_ = custom_status_node_->create_wall_timer(2s, custom_timer_callback);

    locked_executor->add_node(custom_status_node_->get_node_base_interface());
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  RCLCPP_INFO(get_logger(), "Robot hardware_component update_rate is %dHz", info_.rw_rate);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";

  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    // Simulate RRBot's movement
    auto new_value = get_state(name) + (get_command(name) - get_state(name)) / hw_slowdown_;
    set_state(name, new_value);
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << get_state(name) << " for joint '" << name << "'";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";

  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    // Simulate sending commands to the hardware
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << get_command(name) << " for joint '" << name << "'";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_17

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_17::RRBotSystemPositionOnlyHardware,
  hardware_interface::SystemInterface)
