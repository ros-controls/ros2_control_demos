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

#ifndef MOTION_CONTROLLER__OBSERVATION_FORMATTER_HPP_
#define MOTION_CONTROLLER__OBSERVATION_FORMATTER_HPP_

#include <string>
#include <vector>

#include "control_msgs/msg/float64_values.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace motion_controller
{

class ObservationFormatter
{
public:
  explicit ObservationFormatter(
    const std::vector<std::string> & joint_names, const std::string & imu_sensor_name = "imu_2");

  // Order: gyro(3), accel(3), commands(7), joint_pos(N), joint_vel(N), last_action*3(N),
  // motor_targets(N), feet(2), phase(2). Ref: v2_rl_walk_mujoco.py
  std::vector<float> format(
    const control_msgs::msg::Float64Values & interface_data,
    const geometry_msgs::msg::Twist & velocity_cmd, const std::vector<double> & previous_action);

  size_t get_observation_dim() const { return observation_dim_; }

  std::vector<double> extract_joint_positions(
    const control_msgs::msg::Float64Values & interface_data);

  void set_default_joint_positions(const std::vector<double> & default_positions);
  void set_interface_names(const std::vector<std::string> & interface_names);
  void update_action_history(const std::vector<double> & action);
  void set_motor_targets(const std::vector<double> & motor_targets);
  void set_feet_contacts(double left_contact, double right_contact);
  void update_imitation_phase(double phase_frequency_factor);
  std::vector<double> get_imitation_phase() const { return imitation_phase_; }

  void set_num_steps_in_gait_period(double num_steps);
  void set_velocity_commands(const std::vector<double> & commands);
  void set_imu_upside_down(bool upside_down) { imu_upside_down_ = upside_down; }
  void set_gyro_deadband(double deadband) { gyro_deadband_ = deadband; }

private:
  void extract_interface_data(
    const control_msgs::msg::Float64Values & msg, std::vector<double> & gyro,
    std::vector<double> & accelero, std::vector<double> & joint_positions,
    std::vector<double> & joint_velocities);

  void format_joint_positions_relative(
    const std::vector<double> & joint_positions, std::vector<float> & observation);
  void format_joint_velocities_scaled(
    const std::vector<double> & joint_velocities, std::vector<float> & observation);

  std::vector<std::string> joint_names_;
  size_t num_joints_;
  size_t observation_dim_;  // 17 + 6*N
  std::vector<double> default_joint_positions_;
  bool default_joint_positions_set_;
  std::string imu_sensor_name_;
  bool imu_upside_down_;
  double gyro_deadband_;
  std::vector<std::string> interface_names_;
  std::vector<double> last_action_;
  std::vector<double> last_last_action_;
  std::vector<double> last_last_last_action_;
  std::vector<double> motor_targets_;
  double left_contact_;
  double right_contact_;
  double imitation_i_;
  double num_steps_in_gait_period_;
  std::vector<double> imitation_phase_;
  std::vector<double> velocity_commands_;
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__OBSERVATION_FORMATTER_HPP_
