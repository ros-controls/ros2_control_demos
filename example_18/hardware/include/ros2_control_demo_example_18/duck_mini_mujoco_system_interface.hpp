// Copyright (C) 2026 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Julia Jia

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_18__DUCK_MINI_MUJOCO_SYSTEM_INTERFACE_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_18__DUCK_MINI_MUJOCO_SYSTEM_INTERFACE_HPP_

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <mujoco_ros2_control/mujoco_system_interface.hpp>

namespace ros2_control_demo_example_18
{

struct ContactDetectionData
{
  enum class ConsumerMode
  {
    COLLISION,  // raw contact (minimal filtering)
    GAIT        // debounced/hysteresis contact (stable for gait state estimation)
  };

  std::string name;
  double contact_value;           // exported "contact" value (may be filtered depending on mode)
  double raw_contact_value{0.0};  // exported "contact_raw" value (always unfiltered)

  std::string body1_name;  // First body name in MuJoCo model
  std::string body2_name;  // Second body name in MuJoCo model
  int body1_id;            // MuJoCo body ID for body1
  int body2_id;            // MuJoCo body ID for body2

  // Consumer configuration
  ConsumerMode mode{ConsumerMode::COLLISION};
  int debounce_on_steps{1};   // steps of raw contact required to switch to contact=1
  int debounce_off_steps{1};  // steps of raw no-contact required to switch to contact=0

  // Consumer state
  bool filtered_contact_state{false};
  int on_counter{0};
  int off_counter{0};
};

class DuckMiniMujocoSystemInterface : public mujoco_ros2_control::MujocoSystemInterface
{
public:
  DuckMiniMujocoSystemInterface();
  ~DuckMiniMujocoSystemInterface() override = default;

  hardware_interface::CallbackReturn
#if ROS_DISTRO_HUMBLE
  on_init(const hardware_interface::HardwareInfo & info) override;
#else
  on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
#endif

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void register_contact_detection();
  void update_contact_detection();
  std::vector<ContactDetectionData> contact_detection_data_;
};

}  // namespace ros2_control_demo_example_18

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_18__DUCK_MINI_MUJOCO_SYSTEM_INTERFACE_HPP_
