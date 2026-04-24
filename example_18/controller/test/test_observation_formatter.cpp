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

#include "control_msgs/msg/float64_values.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motion_controller/observation_formatter.hpp"

namespace
{
control_msgs::msg::Float64Values make_interface_data_with_imu(
  size_t num_joints, double ori_x, double ori_y, double ori_z, double ori_w)
{
  control_msgs::msg::Float64Values msg;
  msg.values.resize(10 + 2 * num_joints, 0.0);
  msg.values[0] = ori_x;
  msg.values[1] = ori_y;
  msg.values[2] = ori_z;
  msg.values[3] = ori_w;
  return msg;
}
}  // namespace

class TestObservationFormatter : public ::testing::Test
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
    formatter_ = std::make_unique<motion_controller::ObservationFormatter>(joint_names_);

    // Build interface names list following broadcaster configuration (position/velocity per joint)
    interface_names_.clear();
    for (const auto & joint_name : joint_names_)
    {
      interface_names_.push_back(joint_name + "/position");
      interface_names_.push_back(joint_name + "/velocity");
    }
    formatter_->set_interface_names(interface_names_);
  }

  std::vector<std::string> joint_names_;
  size_t num_joints_;
  std::vector<std::string> interface_names_;
  std::unique_ptr<motion_controller::ObservationFormatter> formatter_;
};

TEST_F(TestObservationFormatter, ObservationDimension)
{
  // Expected: 17 + 6*N where N = 12 joints (Open Duck Mini format)
  // 3 (gyro) + 3 (accelero) + 7 (commands) + 12 (joint_pos) + 12 (joint_vel)
  // + 12 (last_action) + 12 (last_last_action) + 12 (last_last_last_action)
  // + 12 (motor_targets) + 2 (feet_contacts) + 2 (phase)
  size_t expected_dim = 17 + 6 * num_joints_;
  EXPECT_EQ(formatter_->get_observation_dim(), expected_dim);
}

TEST_F(TestObservationFormatter, ObservationVectorFormat)
{
  // Create test interface data
  control_msgs::msg::Float64Values interface_data;
  geometry_msgs::msg::Twist velocity_cmd;
  velocity_cmd.linear.x = 0.5;
  velocity_cmd.linear.y = 0.0;
  velocity_cmd.angular.z = 0.0;

  std::vector<double> previous_action(num_joints_, 0.1);

  // Set default joint positions
  std::vector<double> default_positions(num_joints_, 0.5);
  formatter_->set_default_joint_positions(default_positions);

  // Set velocity commands (7D: 3 base + 4 head)
  std::vector<double> velocity_commands = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  formatter_->set_velocity_commands(velocity_commands);

  // Format observation
  std::vector<float> observation =
    formatter_->format(interface_data, velocity_cmd, previous_action);

  // Verify dimension
  EXPECT_EQ(observation.size(), formatter_->get_observation_dim());

  // Verify order: gyro (3D) - should be zeros from empty interface_data
  EXPECT_FLOAT_EQ(observation[0], 0.0f);
  EXPECT_FLOAT_EQ(observation[1], 0.0f);
  EXPECT_FLOAT_EQ(observation[2], 0.0f);

  // Verify order: accelero (3D) - should be zeros from empty interface_data
  // Note: accelero[0] has a bias of 1.3 applied (observation_formatter.cpp line 100)
  EXPECT_FLOAT_EQ(observation[3], 1.3f);
  EXPECT_FLOAT_EQ(observation[4], 0.0f);
  EXPECT_FLOAT_EQ(observation[5], 0.0f);

  // Verify order: commands (7D: 3 base + 4 head)
  EXPECT_FLOAT_EQ(observation[6], 0.5f);   // lin_vel_x
  EXPECT_FLOAT_EQ(observation[7], 0.0f);   // lin_vel_y
  EXPECT_FLOAT_EQ(observation[8], 0.0f);   // ang_vel_z
  EXPECT_FLOAT_EQ(observation[9], 0.0f);   // head_pos_1
  EXPECT_FLOAT_EQ(observation[10], 0.0f);  // head_pos_2
  EXPECT_FLOAT_EQ(observation[11], 0.0f);  // head_pos_3
  EXPECT_FLOAT_EQ(observation[12], 0.0f);  // head_pos_4

  // Verify order: joint_positions (N joints, relative to default)
  // Since interface_data is empty, positions are 0.0, relative to default 0.5 = -0.5
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_FLOAT_EQ(observation[13 + i], -0.5f);
  }

  // Verify order: joint_velocities * 0.05 (N joints)
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_FLOAT_EQ(observation[13 + num_joints_ + i], 0.0f);
  }

  // Verify order: last_action (N joints)
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_FLOAT_EQ(observation[13 + 2 * num_joints_ + i], 0.1f);
  }
}

TEST_F(TestObservationFormatter, RelativeJointPositions)
{
  auto interface_data = make_interface_data_with_imu(num_joints_, 0.0, 0.0, 0.0, 1.0);
  geometry_msgs::msg::Twist velocity_cmd;

  for (size_t i = 0; i < num_joints_; ++i)
  {
    interface_data.values[10 + i] = 1.0;
  }

  // Set default positions to 0.5
  std::vector<double> default_positions(num_joints_, 0.5);
  formatter_->set_default_joint_positions(default_positions);

  // Set velocity commands
  std::vector<double> velocity_commands(7, 0.0);
  formatter_->set_velocity_commands(velocity_commands);

  std::vector<double> previous_action(num_joints_, 0.0);

  // Format observation
  std::vector<float> observation =
    formatter_->format(interface_data, velocity_cmd, previous_action);

  // Verify joint positions are relative (1.0 - 0.5 = 0.5)
  // Joint positions now start at index 13 (after 3 gyro + 3 accelero + 7 commands)
  for (size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_FLOAT_EQ(observation[13 + i], 0.5f);
  }
}

TEST_F(TestObservationFormatter, VelocityCommandFormat)
{
  control_msgs::msg::Float64Values interface_data;
  geometry_msgs::msg::Twist velocity_cmd;
  velocity_cmd.linear.x = 0.3;
  velocity_cmd.linear.y = 0.2;
  velocity_cmd.angular.z = 0.1;

  // Set velocity commands (7D: 3 base + 4 head)
  std::vector<double> velocity_commands = {0.3, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0};
  formatter_->set_velocity_commands(velocity_commands);

  std::vector<double> previous_action(num_joints_, 0.0);
  std::vector<float> observation =
    formatter_->format(interface_data, velocity_cmd, previous_action);

  // Verify velocity commands are at indices 6-12 (after gyro 0-2 and accelero 3-5)
  EXPECT_FLOAT_EQ(observation[6], 0.3f);   // lin_vel_x
  EXPECT_FLOAT_EQ(observation[7], 0.2f);   // lin_vel_y
  EXPECT_FLOAT_EQ(observation[8], 0.1f);   // ang_vel_z
  EXPECT_FLOAT_EQ(observation[9], 0.0f);   // head_pos_1
  EXPECT_FLOAT_EQ(observation[10], 0.0f);  // head_pos_2
  EXPECT_FLOAT_EQ(observation[11], 0.0f);  // head_pos_3
  EXPECT_FLOAT_EQ(observation[12], 0.0f);  // head_pos_4
}

TEST_F(TestObservationFormatter, ImuUpsideDownInversion)
{
  // MuJoCo reports negative z when standing (ground pushes down in sensor frame)
  auto interface_data = make_interface_data_with_imu(num_joints_, 0.0, 0.0, 0.0, 1.0);
  geometry_msgs::msg::Twist velocity_cmd;

  // Set IMU gyroscope (indices 4-6)
  interface_data.values[4] = 0.1;  // gyro x
  interface_data.values[5] = 0.2;  // gyro y
  interface_data.values[6] = 0.3;  // gyro z

  // Set IMU accelerometer (indices 7-9)
  // Simulate MuJoCo's behavior: negative z when standing
  interface_data.values[7] = 0.5;   // accel x
  interface_data.values[8] = -0.3;  // accel y
  interface_data.values[9] = -9.8;  // accel z (negative when standing)

  // Set default joint positions
  std::vector<double> default_positions(num_joints_, 0.0);
  formatter_->set_default_joint_positions(default_positions);

  // Set velocity commands
  std::vector<double> velocity_commands(7, 0.0);
  formatter_->set_velocity_commands(velocity_commands);

  std::vector<double> previous_action(num_joints_, 0.0);

  // Test 1: imu_upside_down = false (default, no inversion)
  formatter_->set_imu_upside_down(false);
  std::vector<float> observation_no_invert =
    formatter_->format(interface_data, velocity_cmd, previous_action);

  // Verify accelerometer values (indices 3-5)
  // x-axis: 0.5 + 1.3 (bias) = 1.8
  EXPECT_FLOAT_EQ(observation_no_invert[3], 1.8f);   // accel x + bias
  EXPECT_FLOAT_EQ(observation_no_invert[4], -0.3f);  // accel y (no change)
  EXPECT_FLOAT_EQ(observation_no_invert[5], -9.8f);  // accel z (no inversion)

  // Test 2: imu_upside_down = true (invert z-axis)
  formatter_->set_imu_upside_down(true);
  std::vector<float> observation_invert =
    formatter_->format(interface_data, velocity_cmd, previous_action);

  // Verify accelerometer values (indices 3-5)
  // x-axis: 0.5 + 1.3 (bias) = 1.8 (unchanged)
  EXPECT_FLOAT_EQ(observation_invert[3], 1.8f);   // accel x + bias (unchanged)
  EXPECT_FLOAT_EQ(observation_invert[4], -0.3f);  // accel y (unchanged)
  EXPECT_FLOAT_EQ(observation_invert[5], 9.8f);   // accel z (inverted: -(-9.8) = +9.8)

  // Verify x and y are not affected by inversion
  EXPECT_FLOAT_EQ(observation_no_invert[3], observation_invert[3]);
  EXPECT_FLOAT_EQ(observation_no_invert[4], observation_invert[4]);
  EXPECT_NE(observation_no_invert[5], observation_invert[5]);  // z should differ

  // Verify gyroscope is not affected (indices 0-2)
  EXPECT_FLOAT_EQ(observation_no_invert[0], observation_invert[0]);
  EXPECT_FLOAT_EQ(observation_no_invert[1], observation_invert[1]);
  EXPECT_FLOAT_EQ(observation_no_invert[2], observation_invert[2]);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
