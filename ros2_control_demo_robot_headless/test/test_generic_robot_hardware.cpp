// Copyright 2020 CNRS
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

#include <string>
#include <vector>

#include "control_demos_robot_headless/generic_robot_hardware.hpp"
#include "gtest/gtest.h"

using namespace control_demos::robot_headless;

class TestGenericRobotHardware : public ::testing::Test {
 protected:
  void SetUp() {}
};

TEST_F(TestGenericRobotHardware, initialize) {
  int argc = 1;
  char *argv[1];
  argv[0] = new char[6];
  strcpy(argv[0], "test");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenericRobotHardware>());
  rclcpp::shutdown();
  
  delete argv[0];
}
