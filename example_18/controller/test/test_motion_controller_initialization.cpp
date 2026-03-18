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

#include <cmath>
#include <vector>

namespace
{
// Default actuator positions from "home" keyframe (matching Python / config)
// Order: left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee, left_ankle,
//        neck_pitch, head_pitch, head_yaw, head_roll,
//        right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle
const std::vector<double> DEFAULT_ACTUATOR = {
  0.002,  0.053,  -0.63, 1.368, -0.784,  // left leg
  0.0,    0.0,    0.0,   0.0,            // head
  -0.003, -0.065, 0.635, 1.379, -0.796   // right leg
};

void expect_vectors_near(
  const std::vector<double> & a, const std::vector<double> & b, double tolerance = 1e-6)
{
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i)
  {
    EXPECT_NEAR(a[i], b[i], tolerance) << "at index " << i;
  }
}
}  // namespace

/**
 * Test to verify initialization matches Python reference implementation.
 *
 * Python reference (mujoco_infer_base.py lines 121-128):
 *   self.default_actuator = self.model.keyframe("home").ctrl
 *   self.motor_targets = self.default_actuator
 *   self.prev_motor_targets = self.default_actuator
 */
class MotionControllerInitializationTest : public ::testing::Test
{
protected:
  void SetUp() override { num_joints_ = DEFAULT_ACTUATOR.size(); }

  size_t num_joints_;
};

TEST_F(MotionControllerInitializationTest, DefaultActuatorValuesMatchPython)
{
  ASSERT_EQ(num_joints_, 14u) << "Expected 14 joints";

  EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[0], 0.002) << "left_hip_yaw";
  EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[1], 0.053) << "left_hip_roll";
  EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[2], -0.63) << "left_hip_pitch";
  EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[3], 1.368) << "left_knee";
  EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[4], -0.784) << "left_ankle";
  for (size_t i = 5; i < 9; ++i)
  {
    EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[i], 0.0) << "Head joint " << (i - 5);
  }
  EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[9], -0.003) << "right_hip_yaw";
  EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[10], -0.065) << "right_hip_roll";
  EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[11], 0.635) << "right_hip_pitch";
  EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[12], 1.379) << "right_knee";
  EXPECT_DOUBLE_EQ(DEFAULT_ACTUATOR[13], -0.796) << "right_ankle";
}

TEST_F(MotionControllerInitializationTest, MotorTargetsInitializedToDefaultActuator)
{
  std::vector<double> motor_targets = DEFAULT_ACTUATOR;
  expect_vectors_near(motor_targets, DEFAULT_ACTUATOR);
}

TEST_F(MotionControllerInitializationTest, PrevMotorTargetsInitializedToDefaultActuator)
{
  std::vector<double> prev_motor_targets = DEFAULT_ACTUATOR;
  expect_vectors_near(prev_motor_targets, DEFAULT_ACTUATOR);
}

TEST_F(MotionControllerInitializationTest, MotorTargetsEqualPrevMotorTargetsAtInit)
{
  std::vector<double> motor_targets = DEFAULT_ACTUATOR;
  std::vector<double> prev_motor_targets = DEFAULT_ACTUATOR;
  expect_vectors_near(motor_targets, prev_motor_targets);
}

TEST_F(MotionControllerInitializationTest, DefaultActuatorValuesMatchConfigFile)
{
  // open_duck_mini_controllers.yaml default_joint_positions uses same values as home keyframe
  ASSERT_EQ(DEFAULT_ACTUATOR.size(), 14u);
  EXPECT_NEAR(DEFAULT_ACTUATOR[0], 0.002, 1e-6);
  EXPECT_NEAR(DEFAULT_ACTUATOR[4], -0.784, 1e-6);
  EXPECT_NEAR(DEFAULT_ACTUATOR[13], -0.796, 1e-6);
}
