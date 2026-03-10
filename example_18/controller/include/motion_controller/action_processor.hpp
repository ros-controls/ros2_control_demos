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

#ifndef MOTION_CONTROLLER__ACTION_PROCESSOR_HPP_
#define MOTION_CONTROLLER__ACTION_PROCESSOR_HPP_

#include <string>
#include <vector>

namespace motion_controller
{

class ActionProcessor
{
public:
  ActionProcessor(
    const std::vector<std::string> & joint_names, double action_scale = 0.25,
    bool use_default_offset = true);

  // motor_targets = default_actuator + action * action_scale (Ref: mujoco_infer.py)
  std::vector<double> process(
    const std::vector<double> & model_outputs, const std::vector<double> & default_joint_positions);

  double get_action_scale() const { return action_scale_; }

private:
  std::vector<std::string> joint_names_;
  size_t num_joints_;
  double action_scale_;
  bool use_default_offset_;
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__ACTION_PROCESSOR_HPP_
