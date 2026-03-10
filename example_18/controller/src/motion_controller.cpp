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

#include <algorithm>
#include <regex>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/controller_interface.hpp"

#include "motion_controller/motion_controller.hpp"

namespace motion_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

// Constants for throttled logging (steps)
namespace
{
constexpr int LOG_INTERVAL_CONTROL = 50;     // ~1 second at 50Hz
constexpr int LOG_INTERVAL_CLIP = 25;        // Every 25 updates
constexpr int LOG_INTERVAL_CMD_WRITE = 250;  // Less frequent
}  // anonymous namespace

CallbackReturn MotionController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  model_loaded_ = false;
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration MotionController::command_interface_configuration() const
{
  InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

InterfaceConfiguration MotionController::state_interface_configuration() const
{
  // This controller doesn't need state interfaces - it subscribes to topics
  return InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

CallbackReturn MotionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  joint_names_ = params_.joints;

  // Get model path and expand $(find-pkg-share ...) substitution
  std::string raw_model_path = params_.model_path;
  std::regex pkg_share_regex(R"(\$\(find-pkg-share\s+([^\)]+)\))");
  std::smatch match;
  if (std::regex_search(raw_model_path, match, pkg_share_regex))
  {
    std::string package_name = match[1].str();
    std::string package_share = ament_index_cpp::get_package_share_directory(package_name);
    model_path_ = std::regex_replace(raw_model_path, pkg_share_regex, package_share);
  }
  else
  {
    model_path_ = raw_model_path;
  }
  interfaces_broadcaster_topic_ = params_.interfaces_broadcaster_topic;
  interfaces_broadcaster_names_topic_ = params_.interfaces_broadcaster_names_topic;
  velocity_command_topic_ = params_.velocity_command_topic;
  left_contact_sensor_name_ = params_.left_contact_sensor_name;
  right_contact_sensor_name_ = params_.right_contact_sensor_name;

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
    return CallbackReturn::ERROR;
  }

  if (model_path_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model path not specified");
    return CallbackReturn::ERROR;
  }

  model_input_size_ = static_cast<int>(params_.model_input_size);
  model_output_size_ = static_cast<int>(params_.model_output_size);

  command_interface_names_.clear();
  for (const auto & joint_name : joint_names_)
  {
    command_interface_names_.push_back(joint_name + "/position");
  }

  // TODO(juliaj): Check whether this is valid.
  previous_action_.resize(joint_names_.size(), 0.0);

  update_count_ = 0;

  bool imu_upside_down = params_.imu_upside_down;
  phase_frequency_factor_offset_ = params_.phase_frequency_factor_offset;
  num_steps_in_gait_period_ = params_.num_steps_in_gait_period;

  observation_formatter_ = std::make_unique<ObservationFormatter>(joint_names_);
  observation_formatter_->set_num_steps_in_gait_period(num_steps_in_gait_period_);
  observation_formatter_->set_imu_upside_down(imu_upside_down);
  observation_formatter_->set_gyro_deadband(params_.gyro_deadband);

  RCLCPP_DEBUG(get_node()->get_logger(), "IMU: upside_down=%s", imu_upside_down ? "true" : "false");

  action_processor_ = std::make_unique<ActionProcessor>(joint_names_, params_.action_scale, true);
  std::vector<double> param_default_positions = params_.default_joint_positions;

  default_joint_positions_.resize(joint_names_.size(), 0.0);
  if (param_default_positions.size() == joint_names_.size())
  {
    default_joint_positions_ = param_default_positions;
    observation_formatter_->set_default_joint_positions(default_joint_positions_);
    default_joint_positions_initialized_ = true;

    RCLCPP_DEBUG(
      get_node()->get_logger(), "Default positions from config (%zu joints)",
      default_joint_positions_.size());
  }
  else
  {
    default_joint_positions_initialized_ = false;
    RCLCPP_WARN(
      get_node()->get_logger(),
      "default_joint_positions parameter size (%zu) != joint count (%zu). Will read from sensors "
      "on first update.",
      param_default_positions.size(), joint_names_.size());
  }

  max_motor_velocity_ = params_.max_motor_velocity;
  training_control_period_ = params_.training_control_period;
  if (training_control_period_ <= 0.0)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Invalid training_control_period (%.6f). Must be positive. Using default 0.02s.",
      training_control_period_);
    training_control_period_ = 0.02;
  }

  gyro_deadband_ = params_.gyro_deadband;
  stabilization_delay_ = static_cast<int>(params_.stabilization_delay);
  blend_in_steps_ = static_cast<int>(params_.blend_in_steps);
  stabilization_steps_ = 0;
  onnx_active_steps_ = 0;

  // Ref: mujoco_infer.py lines 124-125
  motor_targets_.resize(joint_names_.size(), 0.0);
  prev_motor_targets_.resize(joint_names_.size(), 0.0);
  if (default_joint_positions_initialized_)
  {
    initialize_motor_targets_from_defaults();
  }
  else
  {
    prev_motor_targets_initialized_ = false;
  }

  command_received_ = false;

  // Load model
  if (!load_model(model_path_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load model from: %s", model_path_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Create subscribers
  interface_data_subscriber_ = get_node()->create_subscription<control_msgs::msg::Float64Values>(
    interfaces_broadcaster_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const control_msgs::msg::Float64Values::SharedPtr msg)
    { rt_interface_data_.set(*msg); });

  interfaces_names_subscriber_ = get_node()->create_subscription<control_msgs::msg::Keys>(
    interfaces_broadcaster_names_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
    [this](const control_msgs::msg::Keys::SharedPtr msg) { rt_interface_names_.set(msg->keys); });

  velocity_command_subscriber_ =
    get_node()
      ->create_subscription<example_18_motion_controller_msgs::msg::VelocityCommandWithHead>(
        velocity_command_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const example_18_motion_controller_msgs::msg::VelocityCommandWithHead::SharedPtr msg)
        { rt_velocity_command_.set(*msg); });

  RCLCPP_DEBUG(get_node()->get_logger(), "Configure complete");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!model_loaded_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model not loaded");
    return CallbackReturn::ERROR;
  }

  // Validate command interfaces
  if (command_interfaces_.size() != command_interface_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), command_interfaces_.size());
    return CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "Activate complete");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MotionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

return_type MotionController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto interface_data_op = rt_interface_data_.try_get();
  if (!interface_data_op.has_value())
  {
    return return_type::OK;
  }
  control_msgs::msg::Float64Values interface_data = interface_data_op.value();

  auto velocity_cmd_op = rt_velocity_command_.try_get();
  example_18_motion_controller_msgs::msg::VelocityCommandWithHead velocity_cmd;
  if (velocity_cmd_op.has_value())
  {
    velocity_cmd = velocity_cmd_op.value();
    command_received_ = true;
  }
  else if (update_count_ % (LOG_INTERVAL_CMD_WRITE) == 0)  // Log missing command less frequently
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000,
      "No velocity command received on topic '%s'. Robot will hold default pose.",
      velocity_command_topic_.c_str());
  }

  std::vector<double> velocity_commands_7d = {
    velocity_cmd.base_velocity.linear.x,   // lin_vel_x
    velocity_cmd.base_velocity.linear.y,   // lin_vel_y
    velocity_cmd.base_velocity.angular.z,  // ang_vel_z
    velocity_cmd.head_commands.size() > 0 ? velocity_cmd.head_commands[0]
                                          : 0.0,  // head_pos_1 (neck_pitch)
    velocity_cmd.head_commands.size() > 1 ? velocity_cmd.head_commands[1]
                                          : 0.0,  // head_pos_2 (head_pitch)
    velocity_cmd.head_commands.size() > 2 ? velocity_cmd.head_commands[2]
                                          : 0.0,  // head_pos_3 (head_yaw)
    velocity_cmd.head_commands.size() > 3 ? velocity_cmd.head_commands[3]
                                          : 0.0  // head_pos_4 (head_roll)
  };
  observation_formatter_->set_velocity_commands(velocity_commands_7d);

  if (auto names_op = rt_interface_names_.try_get(); names_op.has_value())
  {
    observation_formatter_->set_interface_names(names_op.value());
    interface_names_cache_ = names_op.value();
  }

  auto get_state_value = [&](const std::string & full_name) -> std::optional<double>
  {
    if (interface_names_cache_.empty())
    {
      return std::nullopt;
    }
    for (size_t i = 0; i < interface_names_cache_.size(); ++i)
    {
      if (interface_names_cache_[i] == full_name)
      {
        if (i < interface_data.values.size())
        {
          return interface_data.values[i];
        }
        return std::nullopt;
      }
    }
    return std::nullopt;
  };

  const std::string left_contact_name = left_contact_sensor_name_ + "/contact_raw";
  const std::string right_contact_name = right_contact_sensor_name_ + "/contact_raw";

  const auto left_contact_sensor = get_state_value(left_contact_name);
  const auto right_contact_sensor = get_state_value(right_contact_name);

  if (!default_joint_positions_initialized_)
  {
    default_joint_positions_ = observation_formatter_->extract_joint_positions(interface_data);
    observation_formatter_->set_default_joint_positions(default_joint_positions_);
    default_joint_positions_initialized_ = true;

    stabilization_steps_ = 0;
    onnx_active_steps_ = 0;

    initialize_motor_targets_from_defaults();

    RCLCPP_DEBUG(
      get_node()->get_logger(), "Default positions from sensors (%zu joints)",
      default_joint_positions_.size());
  }

  if (!command_received_)
  {
    if (!prev_motor_targets_initialized_ && default_joint_positions_initialized_)
    {
      initialize_motor_targets_from_defaults();
    }

    if (prev_motor_targets_initialized_ && command_interfaces_.size() == motor_targets_.size())
    {
      const size_t write_success_count = write_commands_to_hardware(motor_targets_);
      if (!motor_targets_.empty() && write_success_count == 0)
      {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 5000,
          "Failed to write to any command interface");
        return return_type::ERROR;
      }
    }
    return return_type::OK;
  }

  const double actual_control_period = period.seconds();
  if (std::abs(actual_control_period - training_control_period_) > 0.001)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Controller update period (%.4f s) differs from training period (%.4f s). "
      "Consider setting update_rate to %.1f Hz.",
      actual_control_period, training_control_period_, 1.0 / training_control_period_);
  }
  double phase_freq_factor = 1.0 + phase_frequency_factor_offset_;
  observation_formatter_->update_imitation_phase(phase_freq_factor);

  double left_contact = left_contact_sensor.has_value() ? left_contact_sensor.value() : 0.0;
  double right_contact = right_contact_sensor.has_value() ? right_contact_sensor.value() : 0.0;
  observation_formatter_->set_feet_contacts(left_contact, right_contact);

  observation_formatter_->set_motor_targets(motor_targets_);
  std::vector<float> model_inputs;
  try
  {
    model_inputs =
      observation_formatter_->format(interface_data, velocity_cmd.base_velocity, previous_action_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000, "Failed to format observation: %s",
      e.what());
    return return_type::ERROR;
  }

  size_t expected_dim = observation_formatter_->get_observation_dim();
  if (model_inputs.size() != expected_dim)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Observation size mismatch: got %zu, expected %zu", model_inputs.size(), expected_dim);
    return return_type::ERROR;
  }

  stabilization_steps_++;
  std::vector<double> joint_commands;
  std::vector<double> model_outputs;
  if (stabilization_steps_ < static_cast<size_t>(stabilization_delay_))
  {
    joint_commands = default_joint_positions_;
  }
  else
  {
    if (stabilization_steps_ == static_cast<size_t>(stabilization_delay_))
    {
      RCLCPP_DEBUG(get_node()->get_logger(), "ONNX active (blend-in: %d steps)", blend_in_steps_);
      onnx_active_steps_ = 0;
    }

    if (!run_model_inference(model_inputs, model_outputs))
    {
      return return_type::ERROR;
    }
    joint_commands = action_processor_->process(model_outputs, default_joint_positions_);
    onnx_active_steps_++;
    double blend_factor =
      std::min(1.0, static_cast<double>(onnx_active_steps_) / static_cast<double>(blend_in_steps_));
    apply_blend_in(joint_commands, blend_factor);
    previous_action_ = model_outputs;
  }

  std::vector<double> original_joint_commands = joint_commands;
  const double max_change =
    prev_motor_targets_initialized_ ? max_motor_velocity_ * actual_control_period : 0.0;
  // Ref: mujoco_infer.py lines 221-226 (velocity limits only)
  std::vector<bool> clipped_by_velocity =
    apply_rate_limiting(joint_commands, actual_control_period);

  static int clip_log_counter = 0;
  if (++clip_log_counter % LOG_INTERVAL_CLIP == 0)
  {
    for (size_t i = 0; i < clipped_by_velocity.size(); ++i)
    {
      if (clipped_by_velocity[i])
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "[CLIP] Joint '%s' clipped by VELOCITY limit: requested=%.4f, clamped=%.4f, prev=%.4f, "
          "max_change=%.4f",
          i < joint_names_.size() ? joint_names_[i].c_str() : "unknown", original_joint_commands[i],
          joint_commands[i], prev_motor_targets_initialized_ ? prev_motor_targets_[i] : 0.0,
          max_change);
      }
    }
  }

  motor_targets_ = joint_commands;
  double motor_diff = 0.0;  // max |motor_targets - prev_motor_targets|
  if (prev_motor_targets_initialized_ && prev_motor_targets_.size() == motor_targets_.size())
  {
    for (size_t i = 0; i < motor_targets_.size(); ++i)
    {
      motor_diff = std::max(motor_diff, std::abs(motor_targets_[i] - prev_motor_targets_[i]));
    }
  }

  if (update_count_ % LOG_INTERVAL_CONTROL == 0)
  {
    RCLCPP_DEBUG(
      get_node()->get_logger(), "Control step=%zu motor_diff=%.4f vel=[%.2f,%.2f,%.2f]",
      update_count_ + 1, motor_diff, velocity_cmd.base_velocity.linear.x,
      velocity_cmd.base_velocity.linear.y, velocity_cmd.base_velocity.angular.z);
  }

  prev_motor_targets_ = joint_commands;
  const size_t write_success_count = write_commands_to_hardware(joint_commands);
  if (!joint_commands.empty() && write_success_count == 0)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Failed to write to any command interface");
    return return_type::ERROR;
  }

  update_count_++;

  return return_type::OK;
}

void MotionController::initialize_motor_targets_from_defaults()
{
  if (!default_joint_positions_initialized_)
  {
    return;
  }

  motor_targets_ = default_joint_positions_;
  prev_motor_targets_ = default_joint_positions_;
  prev_motor_targets_initialized_ = true;
  observation_formatter_->set_motor_targets(motor_targets_);
}

size_t MotionController::write_commands_to_hardware(const std::vector<double> & joint_commands)
{
  size_t write_success_count = 0;
  for (size_t i = 0; i < command_interfaces_.size() && i < joint_commands.size(); ++i)
  {
    const bool write_success = command_interfaces_[i].set_value(joint_commands[i]);
    if (!write_success)
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set command for joint '%s' to %.6f", joint_names_[i].c_str(), joint_commands[i]);
    }
    else
    {
      write_success_count++;
    }
  }
  return write_success_count;
}

void MotionController::apply_blend_in(std::vector<double> & joint_commands, double blend_factor)
{
  blend_factor = std::clamp(blend_factor, 0.0, 1.0);
  for (size_t i = 0; i < joint_commands.size() && i < default_joint_positions_.size(); ++i)
  {
    joint_commands[i] =
      (1.0 - blend_factor) * default_joint_positions_[i] + blend_factor * joint_commands[i];
  }
}

std::vector<bool> MotionController::apply_rate_limiting(
  std::vector<double> & joint_commands, double control_period)
{
  std::vector<bool> clipped_by_velocity(joint_commands.size(), false);

  if (!prev_motor_targets_initialized_)
  {
    return clipped_by_velocity;
  }

  const double max_change = max_motor_velocity_ * control_period;
  for (size_t i = 0; i < joint_commands.size(); ++i)
  {
    const double original = joint_commands[i];
    joint_commands[i] = std::clamp(
      joint_commands[i], prev_motor_targets_[i] - max_change, prev_motor_targets_[i] + max_change);
    if (joint_commands[i] != original)
    {
      clipped_by_velocity[i] = true;
    }
  }

  return clipped_by_velocity;
}

bool MotionController::load_model(const std::string & model_path)
{
  try
  {
    onnx_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "LocomotionController");
    onnx_memory_info_ = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
    onnx_session_ = std::make_unique<Ort::Session>(*onnx_env_, model_path.c_str(), session_options);
    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_inputs = onnx_session_->GetInputCount();
    size_t num_outputs = onnx_session_->GetOutputCount();
    input_names_.clear();
    for (size_t i = 0; i < num_inputs; ++i)
    {
      auto input_name = onnx_session_->GetInputNameAllocated(i, allocator);
      input_names_.push_back(std::string(input_name.get()));
      if (i == 0)
      {
        auto input_type_info = onnx_session_->GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        input_shape_ = input_tensor_info.GetShape();
      }
    }
    output_names_.clear();
    for (size_t i = 0; i < num_outputs; ++i)
    {
      auto output_name = onnx_session_->GetOutputNameAllocated(i, allocator);
      output_names_.push_back(std::string(output_name.get()));
      if (i == 0)
      {
        auto output_type_info = onnx_session_->GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        output_shape_ = output_tensor_info.GetShape();
      }
    }

    RCLCPP_INFO(
      get_node()->get_logger(), "ONNX IO shapes (from model metadata): input=%s output=%s",
      format_shape_string(input_shape_).c_str(), format_shape_string(output_shape_).c_str());

    validate_model_structure(num_inputs, num_outputs);
    input_name_ptrs_.clear();
    for (const auto & name : input_names_)
    {
      input_name_ptrs_.push_back(name.c_str());
    }
    output_name_ptrs_.clear();
    for (const auto & name : output_names_)
    {
      output_name_ptrs_.push_back(name.c_str());
    }

    RCLCPP_INFO(get_node()->get_logger(), "Model loaded: %s", model_path.c_str());
    model_loaded_ = true;
    return true;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load ONNX model: %s", e.what());
    return false;
  }
}

bool MotionController::run_model_inference(
  const std::vector<float> & inputs, std::vector<double> & outputs)
{
  outputs.clear();
  if (!model_loaded_ || !onnx_session_)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000, "Model not loaded");
    return false;
  }

  try
  {
    // Prepare input tensor
    size_t input_size = inputs.size();

    // Use model's expected input shape if available, otherwise use [input_size]
    std::vector<int64_t> input_shape;
    if (!input_shape_.empty())
    {
      input_shape = input_shape_;
      for (auto & dim : input_shape)
      {
        if (dim == -1)
        {
          dim = static_cast<int64_t>(input_size);
        }
      }
    }
    else
    {
      input_shape = {static_cast<int64_t>(input_size)};
    }

    RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Creating input tensor with shape from %zu inputs", input_size);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      *onnx_memory_info_, const_cast<float *>(inputs.data()), input_size, input_shape.data(),
      input_shape.size());
    auto output_tensors = onnx_session_->Run(
      Ort::RunOptions{nullptr}, input_name_ptrs_.data(), &input_tensor, 1, output_name_ptrs_.data(),
      1);
    float * float_array = output_tensors.front().GetTensorMutableData<float>();
    size_t output_size = output_tensors.front().GetTensorTypeAndShapeInfo().GetElementCount();
    RCLCPP_INFO_ONCE(
      get_node()->get_logger(), "ONNX runtime IO sizes: input_size=%zu output_size=%zu", input_size,
      output_size);
    outputs.reserve(output_size);
    for (size_t i = 0; i < output_size; ++i)
    {
      outputs.push_back(static_cast<double>(float_array[i]));
    }

    return true;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000, "Model inference failed: %s",
      e.what());
    outputs.clear();
    return false;
  }
}

std::string MotionController::format_shape_string(const std::vector<int64_t> & shape)
{
  std::string shape_str = "[";
  for (size_t i = 0; i < shape.size(); ++i)
  {
    if (i > 0) shape_str += ", ";
    if (shape[i] == -1)
    {
      shape_str += "dynamic";
    }
    else
    {
      shape_str += std::to_string(shape[i]);
    }
  }
  shape_str += "]";
  return shape_str;
}

void MotionController::validate_model_structure(size_t num_inputs, size_t num_outputs)
{
  if (num_inputs != 1)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model expects %zu inputs, expected 1", num_inputs);
    throw std::runtime_error("Invalid number of model inputs");
  }

  if (num_outputs != 1)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Model expects %zu outputs, expected 1", num_outputs);
    throw std::runtime_error("Invalid number of model outputs");
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Model validation");
  size_t expected_input_size = observation_formatter_->get_observation_dim();
  size_t model_input_size = 1;
  bool has_dynamic_dim = false;
  for (auto dim : input_shape_)
  {
    if (dim == -1)
    {
      has_dynamic_dim = true;
    }
    else if (dim > 0)
    {
      model_input_size *= dim;
    }
  }

  std::string shape_str = format_shape_string(input_shape_);

  RCLCPP_DEBUG(
    get_node()->get_logger(), "Obs dim=%zu input_shape=%s", expected_input_size, shape_str.c_str());

  if (!has_dynamic_dim && model_input_size != expected_input_size)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "MISMATCH: Model input size (%zu) does not match expected observation dimension (%zu). "
      "Please check model or observation formatter configuration.",
      model_input_size, expected_input_size);
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected from Open Duck Mini: 3 (gyro) + 3 (accelero) + 7 (commands) "
      "+ %zu (joint_pos) + %zu (joint_vel) + %zu (last_action) + %zu (last_last_action) "
      "+ %zu (last_last_last_action) + %zu (motor_targets) + 2 (feet_contacts) + 2 (phase) = %zu",
      joint_names_.size(), joint_names_.size(), joint_names_.size(), joint_names_.size(),
      joint_names_.size(), joint_names_.size(), expected_input_size);
    throw std::runtime_error("Model input size does not match observation dimension");
  }
  if (model_input_size_ > 0)
  {
    if (static_cast<size_t>(model_input_size_) != expected_input_size)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Config model_input_size (%d) does not match calculated observation dimension (%zu). "
        "Using calculated value.",
        model_input_size_, expected_input_size);
    }
    else if (!has_dynamic_dim && static_cast<size_t>(model_input_size_) != model_input_size)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Config model_input_size (%d) does not match ONNX model input size (%zu). "
        "ONNX model shape takes precedence.",
        model_input_size_, model_input_size);
    }
    else
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(), "model_input_size=%d matches obs dim", model_input_size_);
    }
  }
  bool has_dynamic_output = false;
  if (!output_shape_.empty())
  {
    for (auto dim : output_shape_)
    {
      if (dim == -1)
      {
        has_dynamic_output = true;
        break;
      }
    }
  }

  if (!has_dynamic_output && !output_shape_.empty())
  {
    int64_t joint_dim = output_shape_.back();
    int64_t total_elements = 1;
    for (auto dim : output_shape_)
    {
      if (dim > 0)
      {
        total_elements *= dim;
      }
    }
    if (
      joint_dim != static_cast<int64_t>(joint_names_.size()) &&
      total_elements != static_cast<int64_t>(joint_names_.size()))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Model output size (last dim: %ld, total elements: %ld) does not match number of joints "
        "(%zu). "
        "This may cause runtime errors if model outputs don't match joint count.",
        joint_dim, total_elements, joint_names_.size());
    }
    if (model_output_size_ > 0)
    {
      if (static_cast<size_t>(model_output_size_) != joint_names_.size())
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Config model_output_size (%d) does not match number of joints (%zu). "
          "Using actual joint count.",
          model_output_size_, joint_names_.size());
      }
      else if (
        joint_dim != static_cast<int64_t>(model_output_size_) &&
        total_elements != static_cast<int64_t>(model_output_size_))
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Config model_output_size (%d) does not match ONNX model output size "
          "(last dim: %ld, total elements: %ld). ONNX model shape takes precedence.",
          model_output_size_, joint_dim, total_elements);
      }
      else
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "model_output_size=%d matches joints", model_output_size_);
      }
    }
  }
  else if (has_dynamic_output)
  {
    RCLCPP_DEBUG(get_node()->get_logger(), "Dynamic output - validate at runtime");
    if (model_output_size_ > 0)
    {
      if (static_cast<size_t>(model_output_size_) != joint_names_.size())
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Config model_output_size (%d) does not match number of joints (%zu). "
          "Will validate at runtime.",
          model_output_size_, joint_names_.size());
      }
      else
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "model_output_size=%d matches joints", model_output_size_);
      }
    }
  }
}

}  // namespace motion_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motion_controller::MotionController, controller_interface::ControllerInterface)
