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

#include "executorch_policy_controller/executorch_policy.hpp"

#include <algorithm>
#include <stdexcept>

#include "executorch/extension/tensor/tensor_ptr_maker.h"
#include "executorch/extension/threadpool/threadpool_guard.h"
#include "executorch/runtime/platform/runtime.h"

namespace executorch_policy_controller
{

using executorch::extension::FileDataLoader;
using executorch::extension::from_blob;
using executorch::extension::threadpool::NoThreadPoolGuard;
using executorch::runtime::BackendOptions;
using executorch::runtime::Error;
using executorch::runtime::HierarchicalAllocator;
using executorch::runtime::LoadBackendOptionsMap;
using executorch::runtime::MemoryManager;
using executorch::runtime::Method;
using executorch::runtime::Program;
using executorch::runtime::Result;
using executorch::runtime::runtime_init;
using executorch::runtime::Span;
using executorch::runtime::TensorInfo;

namespace
{
void check(Error error, const std::string & what)
{
  if (error != Error::Ok)
  {
    throw std::runtime_error(what + ": " + to_string(error));
  }
}

std::vector<executorch::aten::SizesType> float_tensor_sizes(
  Result<TensorInfo> info, const std::string & what)
{
  check(info.error(), what);
  if (info->scalar_type() != executorch::aten::ScalarType::Float)
  {
    throw std::runtime_error(what + " is not a float tensor");
  }
  return {info->sizes().begin(), info->sizes().end()};
}

size_t element_count(const std::vector<executorch::aten::SizesType> & sizes)
{
  size_t count = 1;
  for (const auto size : sizes)
  {
    count *= static_cast<size_t>(size);
  }
  return count;
}
}  // namespace

void ExecuTorchPolicy::load(const std::string & path)
{
  // The method points into the memory and program below, so it goes first.
  method_.reset();
  program_.reset();
  data_loader_.reset();
  method_allocator_.reset();
  runtime_init();

  auto data_loader = FileDataLoader::from(path.c_str());
  check(data_loader.error(), "Cannot open " + path);
  data_loader_ = std::make_unique<FileDataLoader>(std::move(*data_loader));

  auto program = Program::load(data_loader_.get());
  check(program.error(), "Cannot load program " + path);
  program_ = std::make_unique<Program>(std::move(*program));

  auto method_meta = program_->method_meta("forward");
  check(method_meta.error(), "Program has no forward method");
  if (method_meta->num_inputs() != 1 || method_meta->num_outputs() != 1)
  {
    throw std::runtime_error("Policy must have one input and one output");
  }
  const auto input_sizes = float_tensor_sizes(method_meta->input_tensor_meta(0), "Input");
  output_size_ = element_count(float_tensor_sizes(method_meta->output_tensor_meta(0), "Output"));

  // Memory planned at export time: the buffers are allocated here once and reused by every run.
  planned_buffers_.clear();
  planned_spans_.clear();
  for (size_t i = 0; i < method_meta->num_memory_planned_buffers(); ++i)
  {
    const auto size = method_meta->memory_planned_buffer_size(i);
    check(size.error(), "Cannot read planned buffer size");
    planned_buffers_.emplace_back(static_cast<size_t>(*size));
    planned_spans_.emplace_back(planned_buffers_.back().data(), planned_buffers_.back().size());
  }
  planned_memory_ = std::make_unique<HierarchicalAllocator>(
    Span<Span<uint8_t>>(planned_spans_.data(), planned_spans_.size()));
  memory_manager_ = std::make_unique<MemoryManager>(&method_allocator_, planned_memory_.get());

  // By default XNNPACK shares one workspace, guarded by a mutex, between all programs of the
  // process. A workspace of its own lets run() execute without taking that lock. 0 means
  // WorkspaceSharingMode::Disabled (its header is not installed). A misspelled key is ignored.
  xnnpack_options_ = BackendOptions<2>();
  check(xnnpack_options_.set_option("workspace_sharing_mode", 0), "Cannot set XNNPACK option");
  // A shared weights cache would add another process-wide lock to run(), so keep it off even if
  // the build or another component enabled it.
  check(xnnpack_options_.set_option("weight_cache_enabled", false), "Cannot set XNNPACK option");
  backend_options_ = LoadBackendOptionsMap();
  check(
    backend_options_.set_options("XnnpackBackend", xnnpack_options_.view()),
    "Cannot set XNNPACK option");
  {
    // XNNPACK takes ExecuTorch's thread pool when the method is loaded. Without it the policy
    // runs on the calling thread, which keeps the control loop free of extra worker threads.
    NoThreadPoolGuard single_threaded;
    auto method =
      program_->load_method("forward", memory_manager_.get(), nullptr, nullptr, &backend_options_);
    check(method.error(), "Cannot load forward method");
    method_ = std::make_unique<Method>(std::move(*method));
  }

  input_.assign(element_count(input_sizes), 0.0f);
  input_tensor_ = from_blob(input_.data(), input_sizes);

  // The first execution sets up the backend's internal buffers, so do it here, not in update().
  const std::vector<float> warm_up_input(input_.size(), 0.0f);
  std::vector<float> warm_up_output(output_size_);
  if (!run(warm_up_input.data(), warm_up_output.data()))
  {
    method_.reset();
    throw std::runtime_error("Policy failed on a zero input");
  }
}

bool ExecuTorchPolicy::run(const float * input, float * output)
{
  if (!method_)
  {
    return false;
  }
  std::copy(input, input + input_.size(), input_.begin());
  if (method_->set_input(*input_tensor_, 0) != Error::Ok || method_->execute() != Error::Ok)
  {
    return false;
  }
  const auto & result = method_->get_output(0);
  if (!result.isTensor() || static_cast<size_t>(result.toTensor().numel()) != output_size_)
  {
    return false;
  }
  const float * data = result.toTensor().const_data_ptr<float>();
  std::copy(data, data + output_size_, output);
  return true;
}

}  // namespace executorch_policy_controller
