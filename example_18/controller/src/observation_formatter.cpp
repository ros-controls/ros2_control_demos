// Copyright 2025 ros2_control Development Team
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
// Author: Julia Jia

#include "motion_controller/observation_formatter.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace motion_controller
{

ObservationFormatter::ObservationFormatter(const std::vector<std::string> & joint_names)
: joint_names_(joint_names),
  num_joints_(joint_names.size()),
  default_joint_positions_(num_joints_, 0.0),
  default_joint_positions_set_(false),
  imu_upside_down_(false),
  gyro_deadband_(0.0),
  last_action_(num_joints_, 0.0),
  last_last_action_(num_joints_, 0.0),
  last_last_last_action_(num_joints_, 0.0),
  motor_targets_(num_joints_, 0.0),
  left_contact_(0.0),
  right_contact_(0.0),
  imitation_i_(0.0),
  num_steps_in_gait_period_(27.0),
  imitation_phase_(2, 0.0),
  velocity_commands_(7, 0.0)
{
  observation_dim_ = 17 + 6 * num_joints_;
  imitation_phase_[0] = 1.0;  // cos(0) = 1
  imitation_phase_[1] = 0.0;  // sin(0) = 0
}

std::vector<float> ObservationFormatter::format(
  const control_msgs::msg::Float64Values & interface_data,
  const geometry_msgs::msg::Twist & velocity_cmd, const std::vector<double> & previous_action)
{
  (void)velocity_cmd;

  update_action_history(previous_action);
  std::vector<double> gyro;
  std::vector<double> accelero;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  extract_interface_data(interface_data, gyro, accelero, joint_positions, joint_velocities);
  std::vector<float> observation;
  observation.reserve(observation_dim_);

  if (gyro.size() != 3)
  {
    throw std::runtime_error("Gyro size mismatch: expected 3, got " + std::to_string(gyro.size()));
  }
  for (const auto & val : gyro)
  {
    double filtered_val = (std::abs(val) < gyro_deadband_) ? 0.0 : val;
    observation.push_back(static_cast<float>(filtered_val));
  }

  if (accelero.size() != 3)
  {
    throw std::runtime_error(
      "Accelerometer size mismatch: expected 3, got " + std::to_string(accelero.size()));
  }

  observation.push_back(static_cast<float>(accelero[0] + 1.3));
  observation.push_back(static_cast<float>(accelero[1]));
  float accel_z_processed = static_cast<float>(imu_upside_down_ ? -accelero[2] : accelero[2]);
  observation.push_back(accel_z_processed);

  if (velocity_commands_.size() != 7)
  {
    throw std::runtime_error(
      "Velocity commands size mismatch: expected 7, got " +
      std::to_string(velocity_commands_.size()));
  }
  for (const auto & val : velocity_commands_)
  {
    observation.push_back(static_cast<float>(val));
  }

  format_joint_data(joint_positions, joint_velocities, observation);
  append_doubles_as_floats(last_action_, observation);
  append_doubles_as_floats(last_last_action_, observation);
  append_doubles_as_floats(last_last_last_action_, observation);
  append_doubles_as_floats(motor_targets_, observation);
  double left_contact = left_contact_;
  double right_contact = right_contact_;

  observation.push_back(static_cast<float>(left_contact));
  observation.push_back(static_cast<float>(right_contact));
  observation.push_back(static_cast<float>(imitation_phase_[0]));
  observation.push_back(static_cast<float>(imitation_phase_[1]));
  if (observation.size() != observation_dim_)
  {
    throw std::runtime_error(
      "Observation size mismatch: expected " + std::to_string(observation_dim_) + ", got " +
      std::to_string(observation.size()));
  }

  return observation;
}

std::vector<double> ObservationFormatter::extract_joint_positions(
  const control_msgs::msg::Float64Values & interface_data)
{
  std::vector<double> base_angular_velocity;
  std::vector<double> projected_gravity;
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  extract_interface_data(
    interface_data, base_angular_velocity, projected_gravity, joint_positions, joint_velocities);

  return joint_positions;
}

void ObservationFormatter::set_default_joint_positions(
  const std::vector<double> & default_positions)
{
  if (default_positions.size() == num_joints_)
  {
    default_joint_positions_ = default_positions;
    default_joint_positions_set_ = true;
  }
}

void ObservationFormatter::set_interface_names(const std::vector<std::string> & interface_names)
{
  interface_names_ = interface_names;
}

void ObservationFormatter::extract_interface_data(
  const control_msgs::msg::Float64Values & msg, std::vector<double> & gyro,
  std::vector<double> & accelero, std::vector<double> & joint_positions,
  std::vector<double> & joint_velocities)
{
  gyro.clear();
  accelero.clear();
  joint_positions.clear();
  joint_velocities.clear();
  gyro.resize(3, 0.0);
  accelero.resize(3, 0.0);
  joint_positions.resize(num_joints_, 0.0);
  joint_velocities.resize(num_joints_, 0.0);

  const size_t count = std::min(interface_names_.size(), msg.values.size());
  if (count >= 7)
  {
    gyro[0] = msg.values[4];
    gyro[1] = msg.values[5];
    gyro[2] = msg.values[6];
  }
  if (count >= 10)
  {
    accelero[0] = msg.values[7];
    accelero[1] = msg.values[8];
    accelero[2] = msg.values[9];
  }

  const size_t joint_pos_start = 10;
  const size_t joint_vel_start = 10 + num_joints_;
  if (count >= joint_vel_start + num_joints_)
  {
    for (size_t i = 0; i < num_joints_; ++i)
    {
      joint_positions[i] = msg.values[joint_pos_start + i];
      joint_velocities[i] = msg.values[joint_vel_start + i];
    }
  }
  else if (count >= joint_pos_start + num_joints_)
  {
    for (size_t i = 0; i < num_joints_; ++i)
    {
      joint_positions[i] = msg.values[joint_pos_start + i];
    }
  }
}

void ObservationFormatter::format_joint_data(
  const std::vector<double> & joint_positions, const std::vector<double> & joint_velocities,
  std::vector<float> & observation)
{
  if (joint_positions.size() != num_joints_)
  {
    throw std::runtime_error(
      "Joint positions size mismatch: positions=" + std::to_string(joint_positions.size()) +
      ", expected=" + std::to_string(num_joints_));
  }
  if (joint_velocities.size() != num_joints_)
  {
    throw std::runtime_error(
      "Joint velocities size mismatch: velocities=" + std::to_string(joint_velocities.size()) +
      ", expected=" + std::to_string(num_joints_));
  }
  const double velocity_scale = 0.05;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    observation.push_back(static_cast<float>(joint_positions[i] - default_joint_positions_[i]));
  }
  for (size_t i = 0; i < num_joints_; ++i)
  {
    observation.push_back(static_cast<float>(joint_velocities[i] * velocity_scale));
  }
}

void ObservationFormatter::append_doubles_as_floats(
  const std::vector<double> & src, std::vector<float> & dst)
{
  for (const auto & val : src)
  {
    dst.push_back(static_cast<float>(val));
  }
}

void ObservationFormatter::update_action_history(const std::vector<double> & action)
{
  last_last_last_action_ = last_last_action_;
  last_last_action_ = last_action_;
  if (action.size() == num_joints_)
  {
    last_action_ = action;
  }
}

void ObservationFormatter::set_motor_targets(const std::vector<double> & motor_targets)
{
  if (motor_targets.size() == num_joints_)
  {
    motor_targets_ = motor_targets;
  }
}

void ObservationFormatter::set_feet_contacts(double left_contact, double right_contact)
{
  left_contact_ = left_contact;
  right_contact_ = right_contact;
}

void ObservationFormatter::update_imitation_phase(double phase_frequency_factor)
{
  imitation_i_ += 1.0 * phase_frequency_factor;
  imitation_i_ = std::fmod(imitation_i_, num_steps_in_gait_period_);
  const double PI = 3.14159265358979323846;
  double theta = (imitation_i_ / num_steps_in_gait_period_) * 2.0 * PI;
  imitation_phase_[0] = std::cos(theta);
  imitation_phase_[1] = std::sin(theta);
}

void ObservationFormatter::set_num_steps_in_gait_period(double num_steps)
{
  num_steps_in_gait_period_ = num_steps;
}

void ObservationFormatter::set_velocity_commands(const std::vector<double> & commands)
{
  if (commands.size() == 7)
  {
    velocity_commands_ = commands;
  }
}

}  // namespace motion_controller
