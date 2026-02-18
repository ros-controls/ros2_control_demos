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

#include <gtest/gtest.h>
#include <hardware_interface/version.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <thread>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_control_demo_example_18/duck_mini_mujoco_system_interface.hpp"

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

namespace
{
hardware_interface::ComponentInfo make_contact_sensor(
  const std::string & name, const std::string & body1, const std::string & body2,
  const std::map<std::string, std::string> & extra_params = {})
{
  hardware_interface::ComponentInfo info;
  info.name = name;
  info.type = "sensor";
  info.parameters["mujoco_type"] = "contact";
  info.parameters["body1_name"] = body1;
  info.parameters["body2_name"] = body2;
  for (const auto & [k, v] : extra_params)
  {
    info.parameters[k] = v;
  }
  hardware_interface::InterfaceInfo contact_if;
  contact_if.name = "contact";
  info.state_interfaces.push_back(contact_if);
  hardware_interface::InterfaceInfo contact_raw_if;
  contact_raw_if.name = "contact_raw";
  info.state_interfaces.push_back(contact_raw_if);
  return info;
}

bool get_interface_value(
  std::vector<hardware_interface::StateInterface> & interfaces, const std::string & interface_name,
  double & value)
{
  for (auto & iface : interfaces)
  {
    if (iface.get_name() == interface_name)
    {
      return iface.get_value(value, false);
    }
  }
  return false;
}

bool has_interface(
  const std::vector<hardware_interface::StateInterface> & interfaces,
  const std::string & interface_name)
{
  for (const auto & iface : interfaces)
  {
    if (iface.get_name() == interface_name)
    {
      return true;
    }
  }
  return false;
}

hardware_interface::CallbackReturn call_on_init(
  ros2_control_demo_example_18::DuckMiniMujocoSystemInterface & iface,
  const hardware_interface::HardwareInfo & info)
{
#if ROS_DISTRO_HUMBLE
  return iface.on_init(info);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  return iface.on_init(params);
#endif
}
}  // namespace

class ContactDetectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    // Create a simple MuJoCo model with two bodies that can contact
    create_test_model();

    // Initialize hardware interface
    hardware_info_ = create_hardware_info();
    interface_ = std::make_shared<ros2_control_demo_example_18::DuckMiniMujocoSystemInterface>();

    // Initialize the interface
    ASSERT_EQ(
      call_on_init(*interface_, hardware_info_), hardware_interface::CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    // Deactivate interface before destroying to stop threads cleanly
    if (interface_)
    {
      rclcpp_lifecycle::State inactive_state(0, "inactive");
      interface_->on_deactivate(inactive_state);
      interface_.reset();
    }

    // Clean up ROS
    rclcpp::shutdown();

    // Clean up test file
    if (std::filesystem::exists(test_model_path_))
    {
      std::filesystem::remove(test_model_path_);
    }
  }

  void create_test_model()
  {
    // Create a simple MuJoCo XML with two bodies and a floor
    test_model_path_ = "/tmp/test_contact_detection_model.xml";
    std::ofstream file(test_model_path_);
    file << R"(<?xml version="1.0"?>
<mujoco model="test_contact_detection">
  <option timestep="0.002"/>

  <size nconmax="100"/>

  <worldbody>
    <!-- Floor as infinite plane directly in worldbody (static by default) -->
    <!-- Plane size: [half_x half_y spacing] where spacing is for rendering grid -->
    <geom name="floor_geom" type="plane" size="0 0 1"
          contype="1" conaffinity="1"/>

    <body name="box1" pos="0 0 0.1">
      <freejoint name="box1_joint"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box1_geom" type="box" size="0.05 0.05 0.05"
            contype="1" conaffinity="1" friction="0.6"/>
    </body>

    <body name="box2" pos="0.2 0 0.1">
      <freejoint name="box2_joint"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box2_geom" type="box" size="0.05 0.05 0.05"
            contype="1" conaffinity="1" friction="0.6"/>
    </body>
  </worldbody>
</mujoco>
)";
    file.close();
  }

  hardware_interface::HardwareInfo create_hardware_info()
  {
    hardware_interface::HardwareInfo info;
    info.name = "test_duck_mini_mujoco";
    info.type = "system";
    info.hardware_parameters["mujoco_model"] = test_model_path_;
    info.hardware_parameters["meshdir"] = "";
    info.hardware_parameters["headless"] = "true";  // Enable headless mode for CI compatibility
    info.hardware_parameters["disable_rendering"] =
      "true";  // Disable cameras/lidar to avoid OpenGL issues in tests

    // Add contact detection. floor_geom is in worldbody, so it belongs to "world" (body ID 0)
    info.sensors.push_back(make_contact_sensor("box1_contact", "box1", "world"));
    info.sensors.push_back(make_contact_sensor("box2_contact", "box2", "world"));
    info.sensors.push_back(make_contact_sensor(
      "box1_contact_gait", "box1", "world",
      {{"contact_consumer", "gait"}, {"debounce_on_steps", "3"}, {"debounce_off_steps", "3"}}));

    return info;
  }

  std::string test_model_path_;
  hardware_interface::HardwareInfo hardware_info_;
  std::shared_ptr<ros2_control_demo_example_18::DuckMiniMujocoSystemInterface> interface_;
};

TEST_F(ContactDetectionTest, ContactDetectionRegistration)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto state_interfaces = interface_->export_state_interfaces();

  const char * required[] = {"box1_contact/contact",      "box1_contact/contact_raw",
                             "box2_contact/contact",      "box2_contact/contact_raw",
                             "box1_contact_gait/contact", "box1_contact_gait/contact_raw"};
  for (const auto & name : required)
  {
    EXPECT_TRUE(has_interface(state_interfaces, name)) << name << " interface not found";
  }
}

TEST_F(ContactDetectionTest, ContactDetectionWhenInContact)
{
  rclcpp_lifecycle::State active_state(0, "active");
  ASSERT_EQ(interface_->on_activate(active_state), hardware_interface::CallbackReturn::SUCCESS);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::this_thread::sleep_for(std::chrono::seconds(1));  // boxes fall and settle

  interface_->read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.002));
  auto state_interfaces = interface_->export_state_interfaces();

  double box1_contact, box2_contact, box1_raw, box2_raw;
  ASSERT_TRUE(get_interface_value(state_interfaces, "box1_contact/contact", box1_contact));
  ASSERT_TRUE(get_interface_value(state_interfaces, "box1_contact/contact_raw", box1_raw));
  ASSERT_TRUE(get_interface_value(state_interfaces, "box2_contact/contact", box2_contact));
  ASSERT_TRUE(get_interface_value(state_interfaces, "box2_contact/contact_raw", box2_raw));

  EXPECT_EQ(box1_contact, box1_raw);
  EXPECT_EQ(box2_contact, box2_raw);
  EXPECT_GE(box1_contact, 0.0);
  EXPECT_LE(box1_contact, 1.0);
  EXPECT_GE(box2_contact, 0.0);
  EXPECT_LE(box2_contact, 1.0);
  EXPECT_EQ(box1_contact, 1.0) << "box1 should be in contact with floor after settling";
  EXPECT_EQ(box2_contact, 1.0) << "box2 should be in contact with floor after settling";
}

TEST_F(ContactDetectionTest, GaitConsumerDebounceOn)
{
  rclcpp_lifecycle::State active_state(0, "active");
  ASSERT_EQ(interface_->on_activate(active_state), hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // debounce_on_steps=3: raw becomes 1 immediately; filtered contact needs 3 read() calls
  auto state_interfaces = interface_->export_state_interfaces();
  auto read_gait_values = [&](double & c, double & r)
  {
    ASSERT_TRUE(get_interface_value(state_interfaces, "box1_contact_gait/contact", c));
    ASSERT_TRUE(get_interface_value(state_interfaces, "box1_contact_gait/contact_raw", r));
  };

  interface_->read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.002));
  double c1, r1;
  read_gait_values(c1, r1);
  EXPECT_EQ(r1, 1.0);
  EXPECT_EQ(c1, 0.0);

  interface_->read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.002));
  double c2, r2;
  read_gait_values(c2, r2);
  EXPECT_EQ(r2, 1.0);
  EXPECT_EQ(c2, 0.0);

  interface_->read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.002));
  double c3, r3;
  read_gait_values(c3, r3);
  EXPECT_EQ(r3, 1.0);
  EXPECT_EQ(c3, 1.0);
}

TEST_F(ContactDetectionTest, ContactValueRange)
{
  rclcpp_lifecycle::State active_state(0, "active");
  ASSERT_EQ(interface_->on_activate(active_state), hardware_interface::CallbackReturn::SUCCESS);

  auto state_interfaces = interface_->export_state_interfaces();
  double box1_contact;
  ASSERT_TRUE(get_interface_value(state_interfaces, "box1_contact/contact", box1_contact));

  EXPECT_TRUE(box1_contact == 0.0 || box1_contact == 1.0)
    << "Contact value should be binary (0.0 or 1.0), got " << box1_contact;
}

TEST_F(ContactDetectionTest, InvalidBodyNameHandling)
{
  hardware_interface::HardwareInfo bad_info;
  bad_info.name = "test_duck_mini_mujoco";
  bad_info.type = "system";
  bad_info.hardware_parameters["mujoco_model"] = test_model_path_;
  bad_info.hardware_parameters["meshdir"] = "";
  bad_info.hardware_parameters["headless"] = "true";

  bad_info.sensors.push_back(make_contact_sensor("bad_contact", "nonexistent_body", "world"));

  auto bad_interface =
    std::make_shared<ros2_control_demo_example_18::DuckMiniMujocoSystemInterface>();
  auto result = call_on_init(*bad_interface, bad_info);

  EXPECT_TRUE(
    result == hardware_interface::CallbackReturn::SUCCESS ||
    result == hardware_interface::CallbackReturn::ERROR);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
