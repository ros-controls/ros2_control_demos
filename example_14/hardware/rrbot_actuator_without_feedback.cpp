// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

//
// Authors: Denis Stogl
//

#include "ros2_control_demo_example_14/rrbot_actuator_without_feedback.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_14
{
hardware_interface::CallbackReturn RRBotActuatorWithoutFeedback::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(
    "controller_manager.resource_manager.hardware_component.actuator.RRBotModularJoint"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  socket_port_ = std::stoi(info_.hardware_parameters["example_param_socket_port"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_joint_command_ = std::numeric_limits<double>::quiet_NaN();

  const hardware_interface::ComponentInfo & joint = info_.joints[0];
  // RRBotActuatorWithoutFeedback has exactly one command interface and one joint
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
      joint.command_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
  {
    RCLCPP_FATAL(
      get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
      joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
      hardware_interface::HW_IF_VELOCITY);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  // Initialize objects for fake mechanical connection
  sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_ < 0)
  {
    RCLCPP_FATAL(get_logger(), "Creating socket failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto server = gethostbyname("localhost");

  address_.sin_family = AF_INET;
  bcopy(
    reinterpret_cast<char *>(server->h_addr), reinterpret_cast<char *>(&address_.sin_addr.s_addr),
    server->h_length);
  address_.sin_port = htons(socket_port_);

  const int max_retries = 5;
  const int initial_delay_ms = 1000;  // Initial delay of 1 second

  RCLCPP_INFO(get_logger(), "Trying to connect to port %d.", socket_port_);

  int retries = 0;
  int delay_ms = initial_delay_ms;
  bool connected = false;

  while (retries < max_retries)
  {
    if (connect(sock_, (struct sockaddr *)&address_, sizeof(address_)) == 0)
    {
      connected = true;
      break;
    }

    RCLCPP_WARN(
      get_logger(), "Connection attempt %d failed: %s. Retrying in %d ms...", retries + 1,
      strerror(errno), delay_ms);

    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    delay_ms *= 2;  // Exponential backoff
    retries++;
  }

  if (!connected)
  {
    RCLCPP_FATAL(
      get_logger(), "Connection over socket failed after %d attempts: %s", retries,
      strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Successfully connected to port %d.", socket_port_);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotActuatorWithoutFeedback::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  shutdown(sock_, SHUT_RDWR);  // shutdown socket

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotActuatorWithoutFeedback::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotActuatorWithoutFeedback::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_command_));

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotActuatorWithoutFeedback::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values for joints
  if (std::isnan(hw_joint_command_))
  {
    hw_joint_command_ = 0;
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotActuatorWithoutFeedback::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
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

hardware_interface::return_type RRBotActuatorWithoutFeedback::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_14::RRBotActuatorWithoutFeedback::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (std::isfinite(hw_joint_command_))
  {
    // START: This part here is for exemplary purposes - Please do not copy to your production code
    std::stringstream ss;
    ss << "Writing..." << std::endl;
    ss << std::fixed << std::setprecision(2);
    ss << "Writing command: " << hw_joint_command_ << " for joint '" << info_.joints[0].name << "'"
       << std::endl;

    std::ostringstream data;
    data << hw_joint_command_;
    ss << "Sending data command: " << data.str() << std::endl;
    RCLCPP_INFO(get_logger(), ss.str().c_str());

    // Simulate sending commands to the hardware
    send(sock_, data.str().c_str(), strlen(data.str().c_str()), 0);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
  }
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_14

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_14::RRBotActuatorWithoutFeedback, hardware_interface::ActuatorInterface)
