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
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/actuator_hardware_interface.hpp"
#include "hardware_interface/actuator_hardware.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware.hpp"
#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/system_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace hardware_interface
{

class ResourceManager
{
public:
  ResourceManager()
  {
    actuator_loader_.reset(new pluginlib::ClassLoader<ActuatorHardwareInterface>(
        "hardware_interface", "hardware_interface::ActuatorHardwareInterface"));
    sensor_loader_.reset(new pluginlib::ClassLoader<SensorHardwareInterface>(
        "hardware_interface", "hardware_interface::SensorHardwareInterface"));
    system_loader_.reset(new pluginlib::ClassLoader<SystemHardwareInterface>(
        "hardware_interface", "hardware_interface::SystemHardwareInterface"));

    joint_loader_.reset(new pluginlib::ClassLoader<components::Joint>(
        "hardware_interface", "hardware_interface::components::Joint"));
  }

  ~ResourceManager() = default;

  //  Non real-time safe functions
  return_type load_and_configure_resources_from_urdf(std::string urdf_string)
  {
    std::vector<HardwareInfo> hardware_info_list =
      hardware_interface::parse_control_resources_from_urdf(urdf_string);

    return_type ret = return_type::OK;
    for (auto hardware_info : hardware_info_list) {
      RCLCPP_INFO(rclcpp::get_logger("ros2_control_ResourceManager"),
        "Loading hardware plugin: " + hardware_info.hardware_class_type);
      if (!hardware_info.type.compare("system")) {
        // TODO(anyone): this here is really not nice...
        std::unique_ptr<SystemHardwareInterface> sys_hw_if;
        sys_hw_if.reset(system_loader_->createUnmanagedInstance(hardware_info.hardware_class_type));
        std::shared_ptr<SystemHardware> system_hw = std::make_shared<SystemHardware>(
          std::move(sys_hw_if));
        ret = system_hw->configure(hardware_info);
        if (ret != return_type::OK) {
          return ret;
        }
        systems_.push_back(system_hw);
        // TODO(anyone): enable this for sensors and actuators
//       } else if (!hardware_info.type.compare("sensor")) {
//         std::shared_ptr<SensorHardware> sensor_hw = std::make_shared<SensorHardware>(
//             sensor_loader_->createUniqueInstance(hardware_info.hardware_class_type));
//         ret = sensor_hw->configure(hardware_info);
//         if (ret != return_type::OK) {
//           return ret;
//         }
//         sensors_.push_back(sensor_hw);
//       } else if (!hardware_info.type.compare("actuator")) {
//         std::shared_ptr<ActuatorHardware> actuator_hw = std::make_shared<ActuatorHardware>(
//             actuator_loader_->createUniqueInstance(hardware_info.hardware_class_type));
//         ret = actuator_hw->configure(hardware_info);
//         if (ret != return_type::OK) {
//           return ret;
//         }
//         actuators_.push_back(actuator_hw);
      } else {
        RCLCPP_FATAL(rclcpp::get_logger("ros2_control_ResourceManager"),
          "hardware type not recognized");
        return return_type::ERROR;
      }

      std::vector<std::shared_ptr<components::Joint>> joints;
      for (auto joint_info : hardware_info.joints) {
        RCLCPP_INFO(rclcpp::get_logger("ros2_control_ResourceManager"),
          "Loading joint plugin: " + joint_info.class_type);
        std::shared_ptr<components::Joint> joint = joint_loader_->createSharedInstance(
          joint_info.class_type);
        ret = joint->configure(joint_info);
        if (ret != return_type::OK) {
          return ret;
        }
        joints.push_back(joint);
        joint_to_hardware_mapping_[joint_info.name] = hardware_info.name;
      }
      joint_components_[hardware_info.name] = joints;

      // TODO(anyone): add implementation for sensors
    }

    RCLCPP_INFO(rclcpp::get_logger("ros2_control_ResourceManager"),
      "All hardware and component plugins loaded and configured successfully.");
    return return_type::OK;
  }

  return_type start_all_resources()
  {
    return_type ret = return_type::OK;
    for (auto system : systems_) {
      ret = system->start();
      if (ret != return_type::OK) {
        return ret;
      }
      // initial read of joints
      ret = system->read_joints(joint_components_[system->get_name()]);
      if (ret != return_type::OK) {
        return ret;
      }
      // TODO(anyone): add support to read sensors of a system is they exist
    }
    // TODO(anyone): add sensors and actuators
    return return_type::OK;
  }

  return_type stop_all_resources()
  {
    return_type ret = return_type::OK;
    for (auto system : systems_) {
      ret = system->stop();
      if (ret != return_type::OK) {
        return ret;
      }
      // TODO(anyone): add support to read sensors of a system is they exist
    }
    // TODO(anyone): add sensors and actuators
    return return_type::OK;
  }

  //  Real-time safe functions (at least the goal is to be...)
  return_type read_all_resources()
  {
    return_type ret = return_type::OK;
    for (auto system : systems_) {
      ret = system->read_joints(joint_components_[system->get_name()]);
      if (ret != return_type::OK) {
        return ret;
      }
      // TODO(anyone): add support to read sensors of a system is they exist
    }
    // TODO(anyone): add sensors and actuators
    return ret;
  }

  return_type write_all_resources()
  {
    return_type ret = return_type::OK;
    for (auto system : systems_) {
      ret = system->write_joints(joint_components_[system->get_name()]);
      if (ret != return_type::OK) {
        return ret;
      }
      // TODO(anyone): add support to read sensors of a system is they exist
    }
    // TODO(anyone): add sensors and actuators
    return ret;
  }

private:
  // TODO(all): make this unique?
  std::vector<std::shared_ptr<ActuatorHardware>> actuators_;
  std::vector<std::shared_ptr<SensorHardware>> sensors_;
  std::vector<std::shared_ptr<SystemHardware>> systems_;

  std::map<std::string, std::vector<std::shared_ptr<components::Joint>>> joint_components_;
  std::map<std::string, std::vector<std::shared_ptr<components::Sensor>>> sensor_components_;

  // This is used to resolve a joint name to hardware when requested by the controller manager
  std::map<std::string, std::string> joint_to_hardware_mapping_;

  std::shared_ptr<pluginlib::ClassLoader<ActuatorHardwareInterface>> actuator_loader_;
  std::shared_ptr<pluginlib::ClassLoader<SensorHardwareInterface>> sensor_loader_;
  std::shared_ptr<pluginlib::ClassLoader<SystemHardwareInterface>> system_loader_;

  std::shared_ptr<pluginlib::ClassLoader<components::Joint>> joint_loader_;
};

}  //  namespace hardware_interface


class ROS2ControlManager : public rclcpp::Node
{
public:
  explicit ROS2ControlManager(rclcpp::NodeOptions options)
  : Node("ros2_control_manager", options)
  {
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    //  Load robot_description parameter
    this->declare_parameter("robot_description", "");
    auto get_parameters_result = parameters_client_->get_parameters({"robot_description"});

    //  Test the resulting vector of parameters
    if ((get_parameters_result.size() != 1) ||
      (get_parameters_result[0].get_type() ==
      rclcpp::ParameterType::PARAMETER_NOT_SET))
    {
      RCLCPP_FATAL(this->get_logger(),
        "No robot_description parameter");
      rclcpp::shutdown();
    }

    //  load and configure all resources
    if (resource_manager_.load_and_configure_resources_from_urdf(
        get_parameters_result[0].value_to_string()) != hardware_interface::return_type::OK)
    {
      RCLCPP_FATAL(this->get_logger(), "hardware type not recognized");
      rclcpp::shutdown();
    }

    resource_manager_.start_all_resources();

    timer_ = this->create_wall_timer(1000ms, std::bind(&ROS2ControlManager::loop, this));
  }

  // do some dummy stuff in this loop
  void loop()
  {
    RCLCPP_INFO(this->get_logger(), "Calling loop...");
    resource_manager_.read_all_resources();

    // TODO(anyone): integrate here the controller_manager
//     controller_manager_.update_controllers();

    resource_manager_.write_all_resources();
  }

private:
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  hardware_interface::ResourceManager resource_manager_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
//   options.allow_undeclared_parameters(true);
//   options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<ROS2ControlManager>(options));
  rclcpp::shutdown();
  return 0;
}
