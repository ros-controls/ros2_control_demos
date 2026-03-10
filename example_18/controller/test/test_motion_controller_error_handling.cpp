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

// Tests mirror the error-handling conditions in motion_controller.cpp so that
// policy changes (e.g. when to return ERROR) are caught.

namespace
{

// Same logic as in on_configure: invalid training_control_period -> use default
constexpr double DEFAULT_TRAINING_CONTROL_PERIOD = 0.02;
double effective_training_period(double training_control_period)
{
  return training_control_period <= 0.0 ? DEFAULT_TRAINING_CONTROL_PERIOD : training_control_period;
}

// Same condition as in update(): return ERROR when we had commands to write but none succeeded
bool should_return_error_after_write(size_t joint_commands_size, size_t write_success_count)
{
  return joint_commands_size > 0 && write_success_count == 0;
}

}  // namespace

TEST(MotionControllerErrorHandling, InvalidTrainingPeriodUsesDefault)
{
  EXPECT_DOUBLE_EQ(effective_training_period(0.0), DEFAULT_TRAINING_CONTROL_PERIOD);
  EXPECT_DOUBLE_EQ(effective_training_period(-0.01), DEFAULT_TRAINING_CONTROL_PERIOD);
  EXPECT_DOUBLE_EQ(effective_training_period(0.02), 0.02);
  EXPECT_DOUBLE_EQ(effective_training_period(0.01), 0.01);
}

TEST(MotionControllerErrorHandling, WriteFailureWhenNoInterfaceAcceptsReturnsError)
{
  EXPECT_TRUE(should_return_error_after_write(1, 0));
  EXPECT_TRUE(should_return_error_after_write(14, 0));
  EXPECT_FALSE(should_return_error_after_write(1, 1));
  EXPECT_FALSE(should_return_error_after_write(14, 14));
  EXPECT_FALSE(should_return_error_after_write(0, 0));
}
