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


#ifndef ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_

#include <memory>
#include <vector>

#include "rclcpp/macros.hpp"

#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "ros2_control_demo_hardware/visibility_control.h"

using hardware_interface::components::Joint;
using hardware_interface::components::Sensor;
using hardware_interface::HardwareInfo;
using hardware_interface::hardware_interface_status;
using hardware_interface::return_type;

namespace hardware_interface
{

// TODO(all): This could be templated for Joint, Sensor and System Interface
class BaseSystemHardwareInterface : public SystemHardwareInterface
{
public:
  return_type configure(const HardwareInfo & system_info) override
  {
    info_ = system_info;
    status_ = hardware_interface_status::CONFIGURED;
    return return_type::OK;
  }

  hardware_interface_status get_status() const final
  {
    return status_;
  }

protected:
  HardwareInfo info_;
  hardware_interface_status status_;
};

}  //  namespace hardware_interface

namespace ros2_control_demo_hardware
{
class RRBotSystemPositionOnlyHardware : public hardware_interface::BaseSystemHardwareInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemPositionOnlyHardware);

  return_type configure(const HardwareInfo & system_info) override;

  return_type start() override;

  return_type stop() override;

  // TODO(all): Add a new "sensor not exitst" error?
  return_type read_sensors(std::vector<std::shared_ptr<Sensor>> & /*sensors*/) const override
  {
    return return_type::ERROR;
  }

  return_type read_joints(std::vector<std::shared_ptr<Joint>> & joints) const override;

  return_type write_joints(const std::vector<std::shared_ptr<Joint>> & joints) override;

private:
  double hw_write_time_, hw_read_time_;
  std::vector<double> hw_values_;
};

}  //  namespace ros2_control_demo_hardware

#endif  // ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_
