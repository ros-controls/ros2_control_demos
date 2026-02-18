// Copyright (C) 2026 ros2_control Development Team
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
// Authors: Julia Jia

#include <gtest/gtest.h>

#include <vector>

#include "motion_controller/action_processor.hpp"

namespace
{
void expect_all_equal(const std::vector<double> & vec, double expected)
{
  for (size_t i = 0; i < vec.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(vec[i], expected) << "at index " << i;
  }
}
}  // namespace

class TestActionProcessor : public ::testing::Test
{
protected:
  void SetUp() override
  {
    joint_names_ = {
      "leg_left_hip_roll_joint",    "leg_left_hip_yaw_joint",      "leg_left_hip_pitch_joint",
      "leg_left_knee_pitch_joint",  "leg_left_ankle_pitch_joint",  "leg_left_ankle_roll_joint",
      "leg_right_hip_roll_joint",   "leg_right_hip_yaw_joint",     "leg_right_hip_pitch_joint",
      "leg_right_knee_pitch_joint", "leg_right_ankle_pitch_joint", "leg_right_ankle_roll_joint"};
    num_joints_ = joint_names_.size();
  }

  std::vector<std::string> joint_names_;
  size_t num_joints_;
};

TEST_F(TestActionProcessor, ScaleAndOffset)
{
  motion_controller::ActionProcessor processor(joint_names_, 0.25, true);
  auto processed =
    processor.process(std::vector<double>(num_joints_, 0.4), std::vector<double>(num_joints_, 0.5));
  expect_all_equal(processed, 0.6);  // 0.4 * 0.25 + 0.5
}

TEST_F(TestActionProcessor, ScaleOnly)
{
  motion_controller::ActionProcessor processor(joint_names_, 0.25, false);
  auto processed =
    processor.process(std::vector<double>(num_joints_, 0.4), std::vector<double>(num_joints_, 0.5));
  expect_all_equal(processed, 0.1);  // 0.4 * 0.25
}

TEST_F(TestActionProcessor, ZeroScale)
{
  motion_controller::ActionProcessor processor(joint_names_, 1.0, true);
  auto processed =
    processor.process(std::vector<double>(num_joints_, 0.2), std::vector<double>(num_joints_, 0.3));
  expect_all_equal(processed, 0.5);  // 0.2 * 1.0 + 0.3
}

TEST_F(TestActionProcessor, NegativeValues)
{
  motion_controller::ActionProcessor processor(joint_names_, 0.25, true);
  auto processed = processor.process(
    std::vector<double>(num_joints_, -0.2), std::vector<double>(num_joints_, 0.5));
  expect_all_equal(processed, 0.45);  // -0.2 * 0.25 + 0.5
}

TEST_F(TestActionProcessor, SizeMismatch)
{
  motion_controller::ActionProcessor processor(joint_names_, 0.25, true);

  std::vector<double> wrong_size_outputs(num_joints_ + 1, 0.1);
  std::vector<double> default_positions(num_joints_, 0.5);

  // Should throw exception
  EXPECT_THROW(processor.process(wrong_size_outputs, default_positions), std::invalid_argument);
}

TEST_F(TestActionProcessor, GetParameters)
{
  motion_controller::ActionProcessor processor(joint_names_, 0.25, true);

  EXPECT_DOUBLE_EQ(processor.get_action_scale(), 0.25);
  EXPECT_TRUE(processor.get_use_default_offset());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
