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

#include "ros2_control_demo_example_14/rrbot_sensor_for_position_feedback.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_14
{
hardware_interface::CallbackReturn RRBotSensorPositionFeedback::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.sensor."
                       "RRBotModularPositionSensorJoint"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = hardware_interface::stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  socket_port_ = std::stoi(info_.hardware_parameters["example_param_socket_port"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_joint_state_ = std::numeric_limits<double>::quiet_NaN();

  const hardware_interface::ComponentInfo & joint = info_.joints[0];
  // RRBotSensorPositionFeedback has exactly one state interface and one joint
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

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  // Initialize objects for fake mechanical connection
  obj_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (obj_socket_ < 0)
  {
    RCLCPP_FATAL(get_logger(), "Creating socket failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Setting socket options.");
  if (setsockopt(obj_socket_, SOL_SOCKET, SO_REUSEADDR, &sockoptval_, sizeof(sockoptval_)))
  {
    RCLCPP_FATAL(get_logger(), "Setting socket failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  address_length_ = sizeof(address_);

  address_.sin_family = AF_INET;
  address_.sin_addr.s_addr = INADDR_ANY;
  address_.sin_port = htons(socket_port_);

  RCLCPP_INFO(get_logger(), "Binding to socket address.");
  if (bind(obj_socket_, reinterpret_cast<struct sockaddr *>(&address_), sizeof(address_)) < 0)
  {
    RCLCPP_FATAL(
      get_logger(), "Binding to socket failed: %s",
      strerror(errno));  // Print the error message
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Storage and Thread for incoming data
  rt_incomming_data_ptr_.writeFromNonRT(std::numeric_limits<double>::quiet_NaN());
  incoming_data_thread_ = std::thread(
    [this]()
    {
      // Await and accept connection
      RCLCPP_INFO(get_logger(), "Listening for connection on port %d.", socket_port_);
      if (listen(obj_socket_, 1) < 0)
      {
        RCLCPP_FATAL(get_logger(), "Cannot listen from the server.");
        return hardware_interface::CallbackReturn::ERROR;
      }

      sock_ = accept(
        obj_socket_, reinterpret_cast<struct sockaddr *>(&address_),
        reinterpret_cast<socklen_t *>(&address_length_));
      if (sock_ < 0)
      {
        RCLCPP_FATAL(get_logger(), "Cannot accept on the server.");
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(get_logger(), "Accepting on socket.");

      int incoming_data_read_rate = 1000;  // Hz
      RCLCPP_INFO(
        get_logger(),
        "Creating thread for incoming data and read them with %d Hz to not miss any data.",
        incoming_data_read_rate);

      // Variables for reading from a socket
      const size_t reading_size_bytes = 1024;
      char buffer[reading_size_bytes] = {0};

      // Use nanoseconds to avoid chrono's rounding
      std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000 / incoming_data_read_rate));

      RCLCPP_INFO(get_logger(), "Receiving data");
      while (rclcpp::ok())
      {
        if (recv(sock_, buffer, reading_size_bytes, 0) > 0)
        {
          RCLCPP_DEBUG(get_logger(), "Read form buffer sockets data: '%s'", buffer);

          rt_incomming_data_ptr_.writeFromNonRT(hardware_interface::stod(buffer));
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Data not yet received from socket.");
          rt_incomming_data_ptr_.writeFromNonRT(std::numeric_limits<double>::quiet_NaN());
        }

        bzero(buffer, reading_size_bytes);
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000 / incoming_data_read_rate));
      }
      return hardware_interface::CallbackReturn::SUCCESS;
    });
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSensorPositionFeedback::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  incoming_data_thread_.join();      // stop reading thread
  shutdown(sock_, SHUT_RDWR);        // shutdown socket
  shutdown(obj_socket_, SHUT_RDWR);  // shutdown socket

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSensorPositionFeedback::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_joint_state_));

  return state_interfaces;
}

hardware_interface::CallbackReturn RRBotSensorPositionFeedback::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // set some default values for joints
  if (std::isnan(hw_joint_state_))
  {
    hw_joint_state_ = 0;
  }
  last_measured_velocity_ = 0;

  // In general after a hardware is configured it can be read
  last_timestamp_ = get_clock()->now();

  RCLCPP_INFO(get_logger(), "Configuration successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSensorPositionFeedback::on_activate(
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

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSensorPositionFeedback::on_deactivate(
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

hardware_interface::return_type RRBotSensorPositionFeedback::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  current_timestamp = get_clock()->now();
  rclcpp::Duration duration = current_timestamp - last_timestamp_;
  last_timestamp_ = current_timestamp;

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading..." << std::endl;

  // Simulate RRBot's movement
  measured_velocity = *(rt_incomming_data_ptr_.readFromRT());
  if (!std::isnan(measured_velocity))
  {
    last_measured_velocity_ = measured_velocity;
  }
  hw_joint_state_ += (last_measured_velocity_ * duration.seconds()) / hw_slowdown_;

  ss << std::fixed << std::setprecision(2);
  ss << "Got measured velocity " << measured_velocity << std::endl;
  ss << "Got state " << hw_joint_state_ << " for joint '" << info_.joints[0].name << "'"
     << std::endl;
  RCLCPP_INFO(get_logger(), ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_14

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_14::RRBotSensorPositionFeedback, hardware_interface::SensorInterface)
