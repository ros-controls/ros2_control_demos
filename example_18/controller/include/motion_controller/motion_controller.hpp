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

#ifndef MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/float64_values.hpp"
#include "control_msgs/msg/keys.hpp"
#include "controller_interface/controller_interface.hpp"
#include "example_18_motion_controller_msgs/msg/velocity_command_with_head.hpp"
#include "motion_controller/action_processor.hpp"
#include "motion_controller/observation_formatter.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#if defined(ONNXRUNTIME_FOUND) && __has_include("onnxruntime_cxx_api.h")
#include <onnxruntime_cxx_api.h>
#else
#undef ONNXRUNTIME_FOUND
#endif

namespace motion_controller
{

class MotionController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool load_model(const std::string & model_path);
  std::vector<double> run_model_inference(const std::vector<float> & inputs);
  void initialize_motor_targets_from_defaults();
  size_t write_commands_to_hardware(const std::vector<double> & joint_commands);
  void apply_blend_in(std::vector<double> & joint_commands, double blend_factor);
  void apply_reference_motion_blending(std::vector<double> & joint_commands);
  std::vector<bool> apply_rate_limiting(
    std::vector<double> & joint_commands, double control_period);

#ifdef ONNXRUNTIME_FOUND
  static std::string format_shape_string(const std::vector<int64_t> & shape);
  void validate_model_structure(size_t num_inputs, size_t num_outputs);
#endif

  std::vector<std::string> joint_names_;
  std::string model_path_;
  int model_input_size_;
  int model_output_size_;
  std::string interfaces_broadcaster_topic_;
  std::string interfaces_broadcaster_names_topic_;
  std::string velocity_command_topic_;
  bool use_contact_sensors_{false};
  bool log_contact_sensors_{true};
  std::string left_contact_sensor_name_{"left_foot_contact"};
  std::string right_contact_sensor_name_{"right_foot_contact"};
  std::vector<std::string> interface_names_cache_;

#ifdef ONNXRUNTIME_FOUND
  std::unique_ptr<Ort::Session> onnx_session_;
  std::unique_ptr<Ort::Env> onnx_env_;
  std::unique_ptr<Ort::MemoryInfo> onnx_memory_info_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<const char *> input_name_ptrs_;
  std::vector<const char *> output_name_ptrs_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
#endif

  rclcpp::Subscription<control_msgs::msg::Float64Values>::SharedPtr interface_data_subscriber_;
  rclcpp::Subscription<control_msgs::msg::Keys>::SharedPtr interfaces_names_subscriber_;
  rclcpp::Subscription<example_18_motion_controller_msgs::msg::VelocityCommandWithHead>::SharedPtr
    velocity_command_subscriber_;
  realtime_tools::RealtimeThreadSafeBox<control_msgs::msg::Float64Values> rt_interface_data_;
  realtime_tools::RealtimeThreadSafeBox<std::vector<std::string>> rt_interface_names_;
  realtime_tools::RealtimeThreadSafeBox<
    example_18_motion_controller_msgs::msg::VelocityCommandWithHead>
    rt_velocity_command_;
  std::vector<double> previous_action_;
  bool model_loaded_;
  std::vector<std::string> command_interface_names_;
  std::unique_ptr<ObservationFormatter> observation_formatter_;
  std::unique_ptr<ActionProcessor> action_processor_;
  std::vector<double> default_joint_positions_;
  bool default_joint_positions_initialized_;
  double max_motor_velocity_;
  std::vector<double> motor_targets_;
  std::vector<double> prev_motor_targets_;
  bool prev_motor_targets_initialized_;
  bool command_received_;
  double reference_motion_blend_factor_;
  std::vector<double> smoothed_reference_action_;
  double phase_frequency_factor_offset_;
  double phase_period_;
  double training_control_period_;
  double gyro_deadband_;
  int stabilization_delay_;
  int blend_in_steps_;
  size_t stabilization_steps_;
  size_t onnx_active_steps_;
  size_t update_count_;
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__MOTION_CONTROLLER_HPP_
