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
// Authors: Anthony Shoumikhin

#ifndef EXECUTORCH_POLICY_CONTROLLER__EXECUTORCH_POLICY_HPP_
#define EXECUTORCH_POLICY_CONTROLLER__EXECUTORCH_POLICY_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "executorch/extension/data_loader/file_data_loader.h"
#include "executorch/extension/memory_allocator/malloc_memory_allocator.h"
#include "executorch/extension/tensor/tensor_ptr.h"
#include "executorch/runtime/backend/backend_options_map.h"
#include "executorch/runtime/backend/options.h"
#include "executorch/runtime/executor/method.h"
#include "executorch/runtime/executor/program.h"

namespace executorch_policy_controller
{

/**
 * @brief Runs a policy exported to an ExecuTorch program (.pte) with one float input and one
 * float output.
 *
 * load() reads the program, allocates the memory the "forward" method was planned with at export
 * time and runs it once. After that, run() of the bundled policy starts no thread and does not
 * allocate heap memory, which the tests check. A program with operators that ask for temporary
 * memory at run time would still allocate it in run().
 */
class ExecuTorchPolicy
{
public:
  ExecuTorchPolicy() = default;
  // The memory manager points at members, so the object must stay where it is.
  ExecuTorchPolicy(const ExecuTorchPolicy &) = delete;
  ExecuTorchPolicy & operator=(const ExecuTorchPolicy &) = delete;

  /**
   * @brief Load the "forward" method of the program at @p path.
   * @throws std::runtime_error if the program cannot be loaded or does not have exactly one
   * float input and one float output.
   */
  void load(const std::string & path);

  bool is_loaded() const { return method_ != nullptr; }

  /// Number of floats the policy reads.
  size_t input_size() const { return input_.size(); }

  /// Number of floats the policy writes.
  size_t output_size() const { return output_size_; }

  /**
   * @brief Run the policy.
   * @param input input_size() floats.
   * @param output Receives output_size() floats.
   * @return false if the policy is not loaded or the program failed.
   */
  bool run(const float * input, float * output);

private:
  std::unique_ptr<executorch::extension::FileDataLoader> data_loader_;
  std::unique_ptr<executorch::runtime::Program> program_;
  executorch::extension::MallocMemoryAllocator method_allocator_;
  std::vector<std::vector<uint8_t>> planned_buffers_;
  std::vector<executorch::runtime::Span<uint8_t>> planned_spans_;
  std::unique_ptr<executorch::runtime::HierarchicalAllocator> planned_memory_;
  std::unique_ptr<executorch::runtime::MemoryManager> memory_manager_;
  // ExecuTorch requires backend options to outlive the methods loaded with them.
  executorch::runtime::BackendOptions<2> xnnpack_options_;
  executorch::runtime::LoadBackendOptionsMap backend_options_;
  std::unique_ptr<executorch::runtime::Method> method_;
  std::vector<float> input_;
  executorch::extension::TensorPtr input_tensor_;
  size_t output_size_ = 0;
};

}  // namespace executorch_policy_controller

#endif  // EXECUTORCH_POLICY_CONTROLLER__EXECUTORCH_POLICY_HPP_
