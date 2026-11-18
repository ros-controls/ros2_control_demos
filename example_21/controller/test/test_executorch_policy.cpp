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

#include <gtest/gtest.h>
#include <onnxruntime_cxx_api.h>

#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <iterator>
#include <random>
#include <stdexcept>
#include <thread>
#include <vector>

#include "executorch_policy_controller/executorch_policy.hpp"

using executorch_policy_controller::ExecuTorchPolicy;

// Counts every heap allocation of the process while enabled, including those made inside the
// ExecuTorch and XNNPACK libraries, by replacing glibc's allocation functions.
#if defined(__GLIBC__)
namespace
{
std::atomic<bool> g_count_allocations{false};
std::atomic<size_t> g_allocations{0};

void count_allocation()
{
  if (g_count_allocations.load(std::memory_order_relaxed))
  {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
}
}  // namespace

extern "C"
{
  void * __libc_malloc(size_t size);
  void * __libc_calloc(size_t count, size_t size);
  void * __libc_realloc(void * ptr, size_t size);
  void * __libc_memalign(size_t alignment, size_t size);

  void * malloc(size_t size)
  {
    count_allocation();
    return __libc_malloc(size);
  }

  void * calloc(size_t count, size_t size)
  {
    count_allocation();
    return __libc_calloc(count, size);
  }

  void * realloc(void * ptr, size_t size)
  {
    count_allocation();
    return __libc_realloc(ptr, size);
  }

  void * memalign(size_t alignment, size_t size)
  {
    count_allocation();
    return __libc_memalign(alignment, size);
  }

  void * aligned_alloc(size_t alignment, size_t size)
  {
    count_allocation();
    return __libc_memalign(alignment, size);
  }

  int posix_memalign(void ** ptr, size_t alignment, size_t size)
  {
    count_allocation();
    *ptr = __libc_memalign(alignment, size);
    return *ptr != nullptr || size == 0 ? 0 : ENOMEM;
  }
}
#endif

namespace
{
constexpr size_t kObservationSize = 101;
constexpr size_t kActionSize = 14;

std::vector<std::vector<float>> random_observations(size_t count)
{
  std::mt19937 generator(42);
  std::normal_distribution<float> distribution(0.0f, 1.0f);
  std::vector<std::vector<float>> observations(count, std::vector<float>(kObservationSize));
  for (auto & observation : observations)
  {
    for (auto & value : observation)
    {
      value = distribution(generator);
    }
  }
  return observations;
}
}  // namespace

// First in this file, so no earlier test has started a thread pool that would hide a new one.
TEST(ExecuTorchPolicyTest, LoadStartsNoThreads)
{
#if !defined(__linux__)
  GTEST_SKIP() << "Thread counting reads /proc";
#else
  const auto thread_count = []
  {
    const std::filesystem::directory_iterator tasks("/proc/self/task");
    return std::distance(begin(tasks), end(tasks));
  };
  const auto threads_before = thread_count();
  ExecuTorchPolicy policy;
  policy.load(PTE_MODEL_PATH);
  EXPECT_EQ(thread_count(), threads_before);
#endif
}

TEST(ExecuTorchPolicyTest, LoadsBundledPolicy)
{
  ExecuTorchPolicy policy;
  EXPECT_FALSE(policy.is_loaded());
  ASSERT_NO_THROW(policy.load(PTE_MODEL_PATH));
  EXPECT_TRUE(policy.is_loaded());
  EXPECT_EQ(policy.input_size(), kObservationSize);
  EXPECT_EQ(policy.output_size(), kActionSize);
}

TEST(ExecuTorchPolicyTest, MissingFileThrows)
{
  ExecuTorchPolicy policy;
  EXPECT_THROW(policy.load("does_not_exist.pte"), std::runtime_error);
  EXPECT_FALSE(policy.is_loaded());
  std::vector<float> input(kObservationSize), output(kActionSize);
  EXPECT_FALSE(policy.run(input.data(), output.data()));
}

// A controller that is cleaned up and configured again loads the policy into the same object.
TEST(ExecuTorchPolicyTest, LoadingAgainGivesTheSameActions)
{
  ExecuTorchPolicy fresh, reloaded;
  fresh.load(PTE_MODEL_PATH);
  reloaded.load(PTE_MODEL_PATH);
  reloaded.load(PTE_MODEL_PATH);
  for (auto & observation : random_observations(10))
  {
    std::vector<float> expected(kActionSize), actions(kActionSize);
    ASSERT_TRUE(fresh.run(observation.data(), expected.data()));
    ASSERT_TRUE(reloaded.run(observation.data(), actions.data()));
    EXPECT_EQ(actions, expected);
  }
}

// The bundled program carries the weights of example_18's ONNX policy, so both must agree.
TEST(ExecuTorchPolicyTest, MatchesOnnxRuntimeOnSameWeights)
{
  ExecuTorchPolicy policy;
  policy.load(PTE_MODEL_PATH);

  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test_executorch_policy");
  Ort::Session session(env, ONNX_MODEL_PATH, Ort::SessionOptions{});
  const auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  const std::vector<int64_t> input_shape = {1, static_cast<int64_t>(kObservationSize)};
  const char * input_name = "obs";
  const char * output_name = "continuous_actions";

  for (auto & observation : random_observations(100))
  {
    std::vector<float> actions(kActionSize);
    ASSERT_TRUE(policy.run(observation.data(), actions.data()));

    Ort::Value input = Ort::Value::CreateTensor<float>(
      memory_info, observation.data(), observation.size(), input_shape.data(), input_shape.size());
    auto expected = session.Run(Ort::RunOptions{nullptr}, &input_name, &input, 1, &output_name, 1);
    const float * expected_actions = expected.front().GetTensorData<float>();
    for (size_t i = 0; i < kActionSize; ++i)
    {
      EXPECT_NEAR(actions[i], expected_actions[i], 1e-5f) << "action " << i;
    }
  }
}

TEST(ExecuTorchPolicyTest, RunDoesNotAllocate)
{
#if !defined(__GLIBC__)
  GTEST_SKIP() << "Allocation counting needs glibc";
#else
  // FileDataLoader::from() allocates inside the ExecuTorch shared library, which shows that the
  // counter sees allocations made there and not only in this test's own code.
  g_allocations = 0;
  g_count_allocations = true;
  const bool opened = executorch::extension::FileDataLoader::from(PTE_MODEL_PATH).ok();
  g_count_allocations = false;
  ASSERT_TRUE(opened);
  ASSERT_GT(g_allocations.load(), 0u);

  const auto observations = random_observations(100);
  std::vector<float> actions(kActionSize);
  ExecuTorchPolicy policy;
  policy.load(PTE_MODEL_PATH);

  // The controller loads the policy on one thread and runs it on the control loop thread.
  bool all_ok = true;
  std::thread control_loop(
    [&]
    {
      g_allocations = 0;
      g_count_allocations = true;
      for (const auto & observation : observations)
      {
        all_ok &= policy.run(observation.data(), actions.data());
      }
      g_count_allocations = false;
    });
  control_loop.join();

  EXPECT_TRUE(all_ok);
  EXPECT_EQ(g_allocations.load(), 0u);
#endif
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
