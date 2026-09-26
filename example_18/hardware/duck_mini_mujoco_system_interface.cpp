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

#include "ros2_control_demo_example_18/duck_mini_mujoco_system_interface.hpp"

#include <hardware_interface/version.h>
#include <rclcpp/rclcpp.hpp>

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

namespace ros2_control_demo_example_18
{

DuckMiniMujocoSystemInterface::DuckMiniMujocoSystemInterface()
: mujoco_ros2_control::MujocoSystemInterface()
{
}

#if ROS_DISTRO_HUMBLE
hardware_interface::CallbackReturn DuckMiniMujocoSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
#else
hardware_interface::CallbackReturn DuckMiniMujocoSystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
#endif
{
#if ROS_DISTRO_HUMBLE
  auto ret = mujoco_ros2_control::MujocoSystemInterface::on_init(info);
#else
  auto ret = mujoco_ros2_control::MujocoSystemInterface::on_init(params);
#endif
  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }
  register_contact_detection();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void DuckMiniMujocoSystemInterface::register_contact_detection()
{
  // Body IDs resolved lazily on first read() when model is ready
  const auto & hardware_info = get_hardware_info();
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    auto sensor = hardware_info.sensors.at(sensor_index);
    const std::string sensor_name = sensor.name;

    if (sensor.parameters.count("mujoco_type") == 0)
    {
      continue;
    }
    const auto mujoco_type = sensor.parameters.at("mujoco_type");

    if (mujoco_type != "contact")
    {
      continue;
    }

    ContactDetectionData sensor_data;
    sensor_data.name = sensor_name;

    if (sensor.parameters.count("body1_name") == 0 || sensor.parameters.count("body2_name") == 0)
    {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Contact detection '"
                        << sensor_name << "' requires 'body1_name' and 'body2_name' parameters");
      continue;
    }

    sensor_data.body1_name = sensor.parameters.at("body1_name");
    sensor_data.body2_name = sensor.parameters.at("body2_name");

    contact_detection_data_.push_back(sensor_data);
    RCLCPP_DEBUG_STREAM(
      get_logger(), "Contact '" << sensor_name << "' " << sensor_data.body1_name << "-"
                                << sensor_data.body2_name);
  }
}

std::vector<hardware_interface::StateInterface>
DuckMiniMujocoSystemInterface::export_state_interfaces()
{
  auto state_interfaces = mujoco_ros2_control::MujocoSystemInterface::export_state_interfaces();
  const auto & hardware_info = get_hardware_info();
  for (auto & sensor : contact_detection_data_)
  {
    for (const auto & sensor_info : hardware_info.sensors)
    {
      if (sensor_info.name != sensor.name)
      {
        continue;
      }

      for (const auto & state_if : sensor_info.state_interfaces)
      {
        if (state_if.name == "contact_raw")
        {
          state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.contact_raw_value);
        }
      }
      break;
    }
  }

  return state_interfaces;
}

hardware_interface::return_type DuckMiniMujocoSystemInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto result = mujoco_ros2_control::MujocoSystemInterface::read(time, period);
  if (result != hardware_interface::return_type::OK)
  {
    return result;
  }

  update_contact_detection();

  return hardware_interface::return_type::OK;
}

void DuckMiniMujocoSystemInterface::update_contact_detection()
{
  mjModel * mj_model = nullptr;
  mjData * mj_data = nullptr;

  try
  {
    get_model(mj_model);
    get_data(mj_data);
  }
  catch (const std::exception & e)
  {
    RCLCPP_DEBUG(get_logger(), "Contact detection: %s", e.what());
    if (mj_model != nullptr)
    {
      mj_deleteModel(mj_model);
    }
    if (mj_data != nullptr)
    {
      mj_deleteData(mj_data);
    }
    return;
  }
  catch (...)
  {
    RCLCPP_DEBUG(get_logger(), "Contact detection: unknown exception");
    if (mj_model != nullptr)
    {
      mj_deleteModel(mj_model);
    }
    if (mj_data != nullptr)
    {
      mj_deleteData(mj_data);
    }
    return;
  }

  if (mj_model == nullptr || mj_data == nullptr)
  {
    RCLCPP_DEBUG(get_logger(), "MuJoCo model/data not ready for contact");
    if (mj_model != nullptr)
    {
      mj_deleteModel(mj_model);
    }
    if (mj_data != nullptr)
    {
      mj_deleteData(mj_data);
    }
    return;
  }

  // Resolve body IDs lazily on first call
  for (auto & sensor : contact_detection_data_)
  {
    if (sensor.body1_id == -1 || sensor.body2_id == -1)
    {
      sensor.body1_id = mj_name2id(mj_model, mjOBJ_BODY, sensor.body1_name.c_str());
      sensor.body2_id = mj_name2id(mj_model, mjOBJ_BODY, sensor.body2_name.c_str());

      if (sensor.body1_id == -1)
      {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Failed to find body '" << sensor.body1_name
                                                << "' in MuJoCo model for contact detection '"
                                                << sensor.name << "'");
        continue;
      }
      if (sensor.body2_id == -1)
      {
        RCLCPP_ERROR_STREAM(
          get_logger(), "Failed to find body '" << sensor.body2_name
                                                << "' in MuJoCo model for contact detection '"
                                                << sensor.name << "'");
        continue;
      }
      RCLCPP_DEBUG_STREAM(
        get_logger(), "Resolved body IDs for contact detection '"
                        << sensor.name << "': " << sensor.body1_name << "=" << sensor.body1_id
                        << ", " << sensor.body2_name << "=" << sensor.body2_id);
    }
  }

  // Check collisions via mj_data->contact[]
  struct ContactMatchInfo
  {
    int contact_index{-1};
    int geom1{-1};
    int geom2{-1};
    int geom1_body_id{-1};
    int geom2_body_id{-1};
  };

  auto raw_contact_between_bodies =
    [&](const mjData * data, int body1_id, int body2_id, ContactMatchInfo * match_info) -> bool
  {
    if (!data || !mj_model || data->ncon <= 0 || data->ncon > mj_model->nconmax)
    {
      return false;
    }

    for (int i = 0; i < data->ncon; ++i)
    {
      const mjContact & contact = data->contact[i];
      const int geom1_body_id = mj_model->geom_bodyid[contact.geom1];
      const int geom2_body_id = mj_model->geom_bodyid[contact.geom2];

      if (
        (geom1_body_id == body1_id && geom2_body_id == body2_id) ||
        (geom1_body_id == body2_id && geom2_body_id == body1_id))
      {
        if (match_info)
        {
          match_info->contact_index = i;
          match_info->geom1 = contact.geom1;
          match_info->geom2 = contact.geom2;
          match_info->geom1_body_id = geom1_body_id;
          match_info->geom2_body_id = geom2_body_id;
        }
        return true;
      }
    }

    return false;
  };

  for (auto & sensor : contact_detection_data_)
  {
    bool in_contact =
      raw_contact_between_bodies(mj_data, sensor.body1_id, sensor.body2_id, nullptr);
    sensor.contact_raw_value = in_contact ? 1.0 : 0.0;
  }

  if (mj_model != nullptr)
  {
    mj_deleteModel(mj_model);
  }
  if (mj_data != nullptr)
  {
    mj_deleteData(mj_data);
  }
}

}  // namespace ros2_control_demo_example_18

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_18::DuckMiniMujocoSystemInterface, hardware_interface::SystemInterface)
