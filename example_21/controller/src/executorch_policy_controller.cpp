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
// Authors: Julia Jia, Anthony Shoumikhin

#include "executorch_policy_controller/executorch_policy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <utility>

#include "ament_index_cpp/version.h"
#if AMENT_INDEX_CPP_VERSION_MINOR >= 13
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace executorch_policy_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;
using ros2_control_demo_example_18::msg::VelocityCommandWithHead;

CallbackReturn ExecuTorchPolicyController::on_init()
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
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration ExecuTorchPolicyController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, command_interface_names_};
}

InterfaceConfiguration ExecuTorchPolicyController::state_interface_configuration() const
{
  // Observations come from state_interfaces_broadcaster topics, as in example_18.
  return {controller_interface::interface_configuration_type::NONE};
}

CallbackReturn ExecuTorchPolicyController::on_configure(const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();
  joint_names_ = params_.joints;
  const size_t num_joints = joint_names_.size();

  command_interface_names_.clear();
  for (const auto & joint_name : joint_names_)
  {
    command_interface_names_.push_back(joint_name + "/position");
  }
  left_contact_interface_ = params_.left_contact_sensor_name + "/contact_raw";
  right_contact_interface_ = params_.right_contact_sensor_name + "/contact_raw";

  observation_formatter_ =
    std::make_unique<onnx_policy_controller::ObservationFormatter>(joint_names_);
  observation_formatter_->set_num_steps_in_gait_period(params_.num_steps_in_gait_period);
  observation_formatter_->set_imu_upside_down(params_.imu_upside_down);
  observation_formatter_->set_gyro_deadband(params_.gyro_deadband);
  action_processor_ = std::make_unique<onnx_policy_controller::ActionProcessor>(
    joint_names_, params_.action_scale, true);
  previous_action_.assign(num_joints, 0.0);

  default_joint_positions_initialized_ = params_.default_joint_positions.size() == num_joints;
  if (default_joint_positions_initialized_)
  {
    default_joint_positions_ = params_.default_joint_positions;
    observation_formatter_->set_default_joint_positions(default_joint_positions_);
  }
  else
  {
    default_joint_positions_.assign(num_joints, 0.0);
    RCLCPP_WARN(
      get_node()->get_logger(),
      "default_joint_positions has %zu values for %zu joints. Reading them from sensors instead.",
      params_.default_joint_positions.size(), num_joints);
  }

  joint_position_limits_enabled_ = params_.joint_position_limits_min.size() == num_joints &&
                                   params_.joint_position_limits_max.size() == num_joints;
  if (
    !joint_position_limits_enabled_ &&
    !(params_.joint_position_limits_min.empty() && params_.joint_position_limits_max.empty()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "joint_position_limits_min/max do not have one value per joint. Position limiting disabled.");
  }

  training_control_period_ = params_.training_control_period;
  if (training_control_period_ <= 0.0)
  {
    RCLCPP_WARN(get_node()->get_logger(), "training_control_period must be positive. Using 0.02s.");
    training_control_period_ = 0.02;
  }

  stabilization_steps_ = 0;
  policy_active_steps_ = 0;
  motor_targets_.assign(num_joints, 0.0);
  motor_targets_initialized_ = false;
  initialize_motor_targets_from_defaults();

  interface_data_received_ = false;
  velocity_command_received_ = false;
  velocity_cmd_ = VelocityCommandWithHead();

  if (!load_policy())
  {
    return CallbackReturn::ERROR;
  }

  interface_data_subscriber_ = get_node()->create_subscription<control_msgs::msg::Float64Values>(
    params_.interfaces_broadcaster_topic, rclcpp::SystemDefaultsQoS(),
    [this](const control_msgs::msg::Float64Values::SharedPtr msg)
    {
      rt_interface_data_.set(*msg);
      interface_data_received_ = true;
    });
  interfaces_names_subscriber_ = get_node()->create_subscription<control_msgs::msg::Keys>(
    params_.interfaces_broadcaster_names_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
    [this](const control_msgs::msg::Keys::SharedPtr msg) { rt_interface_names_.set(msg->keys); });
  velocity_command_subscriber_ = get_node()->create_subscription<VelocityCommandWithHead>(
    params_.velocity_command_topic, rclcpp::SystemDefaultsQoS(),
    [this](const VelocityCommandWithHead::SharedPtr msg)
    {
      rt_velocity_command_.set(*msg);
      velocity_command_received_ = true;
    });

  return CallbackReturn::SUCCESS;
}

CallbackReturn ExecuTorchPolicyController::on_activate(const rclcpp_lifecycle::State &)
{
  if (!policy_.is_loaded())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Policy not loaded");
    return CallbackReturn::ERROR;
  }
  if (command_interfaces_.size() != command_interface_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), command_interfaces_.size());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExecuTorchPolicyController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

return_type ExecuTorchPolicyController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!interface_data_received_)
  {
    return return_type::OK;
  }
  auto interface_data_op = rt_interface_data_.try_get();
  if (!interface_data_op.has_value())
  {
    return return_type::OK;
  }
  const control_msgs::msg::Float64Values interface_data = std::move(*interface_data_op);

  const bool command_received = velocity_command_received_;
  if (command_received)
  {
    // On lock contention keep the last command instead of falling back to zero velocity.
    if (auto velocity_cmd_op = rt_velocity_command_.try_get(); velocity_cmd_op.has_value())
    {
      velocity_cmd_ = velocity_cmd_op.value();
    }
  }
  observation_formatter_->set_velocity_commands(
    {velocity_cmd_.base_velocity.linear.x, velocity_cmd_.base_velocity.linear.y,
     velocity_cmd_.base_velocity.angular.z, velocity_cmd_.neck_pitch, velocity_cmd_.head_pitch,
     velocity_cmd_.head_yaw, velocity_cmd_.head_roll});
  if (auto names_op = rt_interface_names_.try_get(); names_op.has_value())
  {
    observation_formatter_->set_interface_names(names_op.value());
  }
  observation_formatter_->update_feet_contacts_from_interfaces(
    interface_data, left_contact_interface_, right_contact_interface_);

  if (!default_joint_positions_initialized_)
  {
    default_joint_positions_ = observation_formatter_->extract_joint_positions(interface_data);
    observation_formatter_->set_default_joint_positions(default_joint_positions_);
    default_joint_positions_initialized_ = true;
    stabilization_steps_ = 0;
    policy_active_steps_ = 0;
    initialize_motor_targets_from_defaults();
  }

  // Without a velocity command the policy does not run and the robot holds its standing pose.
  if (!command_received)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 10000,
      "No velocity command received on topic '%s'. Robot will hold default pose.",
      velocity_command_subscriber_->get_topic_name());
    return write_commands_to_hardware(motor_targets_) ? return_type::OK : return_type::ERROR;
  }

  const double control_period = period.seconds();
  if (std::abs(control_period - training_control_period_) > 0.004)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Update period (%.4f s) differs from training period (%.4f s)", control_period,
      training_control_period_);
  }
  observation_formatter_->update_imitation_phase(1.0 + params_.phase_frequency_factor_offset);
  observation_formatter_->set_motor_targets(motor_targets_);

  std::vector<float> observation;
  try
  {
    observation =
      observation_formatter_->format(interface_data, velocity_cmd_.base_velocity, previous_action_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000, "Failed to format observation: %s",
      e.what());
    return return_type::ERROR;
  }
  if (observation.size() != policy_.input_size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Observation has %zu values, the policy expects %zu", observation.size(),
      policy_.input_size());
    return return_type::ERROR;
  }

  std::vector<double> joint_commands;
  if (++stabilization_steps_ < static_cast<size_t>(params_.stabilization_delay))
  {
    joint_commands = default_joint_positions_;
  }
  else
  {
    if (!policy_.run(observation.data(), policy_output_.data()))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000, "Policy inference failed");
      return return_type::ERROR;
    }
    std::copy(policy_output_.begin(), policy_output_.end(), policy_actions_.begin());
    try
    {
      joint_commands = action_processor_->process(policy_actions_, default_joint_positions_);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to process policy actions into joint commands: %s", e.what());
      return return_type::ERROR;
    }
    ++policy_active_steps_;
    apply_blend_in(
      joint_commands,
      static_cast<double>(policy_active_steps_) / static_cast<double>(params_.blend_in_steps));
    previous_action_ = policy_actions_;
  }

  apply_rate_limiting(joint_commands, control_period);
  apply_position_limiting(joint_commands);
  motor_targets_ = joint_commands;
  return write_commands_to_hardware(joint_commands) ? return_type::OK : return_type::ERROR;
}

std::string ExecuTorchPolicyController::resolve_model_path(const std::string & raw_path) const
{
  const std::regex pkg_share_regex(R"(\$\(find-pkg-share\s+([^\)]+)\))");
  std::smatch match;
  if (!std::regex_search(raw_path, match, pkg_share_regex))
  {
    return raw_path;
  }
#if AMENT_INDEX_CPP_VERSION_MINOR >= 13
  const std::string package_share =
    ament_index_cpp::get_package_share_path(match[1].str()).string();
#else
  const std::string package_share = ament_index_cpp::get_package_share_directory(match[1].str());
#endif
  return std::regex_replace(raw_path, pkg_share_regex, package_share);
}

bool ExecuTorchPolicyController::load_policy()
{
  try
  {
    const std::string model_path = resolve_model_path(params_.model_path);
    policy_.load(model_path);
    RCLCPP_INFO(
      get_node()->get_logger(), "Loaded %s (input %zu, output %zu)", model_path.c_str(),
      policy_.input_size(), policy_.output_size());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load policy: %s", e.what());
    return false;
  }
  if (policy_.input_size() != observation_formatter_->get_observation_dim())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Policy input size %zu does not match observation size %zu",
      policy_.input_size(), observation_formatter_->get_observation_dim());
    return false;
  }
  if (policy_.output_size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Policy output size %zu does not match %zu joints",
      policy_.output_size(), joint_names_.size());
    return false;
  }
  policy_output_.assign(policy_.output_size(), 0.0f);
  policy_actions_.assign(policy_.output_size(), 0.0);
  return true;
}

void ExecuTorchPolicyController::initialize_motor_targets_from_defaults()
{
  if (!default_joint_positions_initialized_)
  {
    return;
  }
  motor_targets_ = default_joint_positions_;
  motor_targets_initialized_ = true;
}

void ExecuTorchPolicyController::apply_blend_in(
  std::vector<double> & joint_commands, double blend_factor) const
{
  blend_factor = std::clamp(blend_factor, 0.0, 1.0);
  for (size_t i = 0; i < joint_commands.size(); ++i)
  {
    joint_commands[i] =
      (1.0 - blend_factor) * default_joint_positions_[i] + blend_factor * joint_commands[i];
  }
}

void ExecuTorchPolicyController::apply_rate_limiting(
  std::vector<double> & joint_commands, double control_period) const
{
  if (!motor_targets_initialized_)
  {
    return;
  }
  const double max_change = params_.max_motor_velocity * control_period;
  for (size_t i = 0; i < joint_commands.size(); ++i)
  {
    joint_commands[i] =
      std::clamp(joint_commands[i], motor_targets_[i] - max_change, motor_targets_[i] + max_change);
  }
}

void ExecuTorchPolicyController::apply_position_limiting(std::vector<double> & joint_commands) const
{
  if (!joint_position_limits_enabled_)
  {
    return;
  }
  for (size_t i = 0; i < joint_commands.size(); ++i)
  {
    joint_commands[i] = std::clamp(
      joint_commands[i], params_.joint_position_limits_min[i],
      params_.joint_position_limits_max[i]);
  }
}

bool ExecuTorchPolicyController::write_commands_to_hardware(
  const std::vector<double> & joint_commands)
{
  if (!motor_targets_initialized_)
  {
    return true;
  }
  bool any_written = joint_commands.empty();
  for (size_t i = 0; i < command_interfaces_.size() && i < joint_commands.size(); ++i)
  {
    // Also clamps the standing pose, which does not go through apply_position_limiting().
    const double command = joint_position_limits_enabled_
                             ? std::clamp(
                                 joint_commands[i], params_.joint_position_limits_min[i],
                                 params_.joint_position_limits_max[i])
                             : joint_commands[i];
    any_written |= command_interfaces_[i].set_value(command);
  }
  if (!any_written)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Failed to write to any command interface");
  }
  return any_written;
}

}  // namespace executorch_policy_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  executorch_policy_controller::ExecuTorchPolicyController,
  controller_interface::ControllerInterface)
