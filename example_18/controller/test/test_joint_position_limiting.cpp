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
#include <algorithm>
#include <vector>

namespace
{

std::vector<bool> apply_position_limiting(
  std::vector<double> & joint_commands, const std::vector<double> & min_limits,
  const std::vector<double> & max_limits, bool limits_enabled)
{
  std::vector<bool> clipped_by_position(joint_commands.size(), false);

  if (!limits_enabled)
  {
    return clipped_by_position;
  }

  for (size_t i = 0; i < joint_commands.size(); ++i)
  {
    const double original = joint_commands[i];
    joint_commands[i] = std::clamp(joint_commands[i], min_limits[i], max_limits[i]);
    if (joint_commands[i] != original)
    {
      clipped_by_position[i] = true;
    }
  }

  return clipped_by_position;
}

}  // namespace

TEST(JointPositionLimiting, ClipsAboveMaxLimit)
{
  std::vector<double> commands = {1.585485};
  const std::vector<double> min_limits = {-1.570796};
  const std::vector<double> max_limits = {1.570796};

  const auto clipped = apply_position_limiting(commands, min_limits, max_limits, true);

  EXPECT_TRUE(clipped[0]);
  EXPECT_DOUBLE_EQ(commands[0], 1.570796);
}

TEST(JointPositionLimiting, ClipsBelowMinLimit)
{
  std::vector<double> commands = {-1.8};
  const std::vector<double> min_limits = {-1.570796};
  const std::vector<double> max_limits = {1.570796};

  const auto clipped = apply_position_limiting(commands, min_limits, max_limits, true);

  EXPECT_TRUE(clipped[0]);
  EXPECT_DOUBLE_EQ(commands[0], -1.570796);
}

TEST(JointPositionLimiting, LeavesInRangeCommandsUnchanged)
{
  std::vector<double> commands = {1.379, 0.053};
  const std::vector<double> min_limits = {-1.570796, -0.436332};
  const std::vector<double> max_limits = {1.570796, 0.436332};

  const auto clipped = apply_position_limiting(commands, min_limits, max_limits, true);

  EXPECT_FALSE(clipped[0]);
  EXPECT_FALSE(clipped[1]);
  EXPECT_DOUBLE_EQ(commands[0], 1.379);
  EXPECT_DOUBLE_EQ(commands[1], 0.053);
}

TEST(JointPositionLimiting, DisabledWhenLimitsNotConfigured)
{
  std::vector<double> commands = {5.0};
  const std::vector<double> min_limits = {};
  const std::vector<double> max_limits = {};

  const auto clipped = apply_position_limiting(commands, min_limits, max_limits, false);

  EXPECT_FALSE(clipped[0]);
  EXPECT_DOUBLE_EQ(commands[0], 5.0);
}
