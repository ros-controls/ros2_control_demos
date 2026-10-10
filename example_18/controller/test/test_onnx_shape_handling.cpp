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

#include <string>
#include <vector>

// Test helper functions for shape handling
// These mirror the logic in LocomotionController for testing

std::string format_shape_string(const std::vector<int64_t> & shape)
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

std::vector<int64_t> resolve_dynamic_shape(
  const std::vector<int64_t> & model_shape, size_t actual_input_size)
{
  std::vector<int64_t> resolved_shape = model_shape;

  // Replace -1 (dynamic dimension) with actual size
  for (auto & dim : resolved_shape)
  {
    if (dim == -1)
    {
      dim = static_cast<int64_t>(actual_input_size);
    }
  }

  return resolved_shape;
}

class TestOnnxShapeHandling : public ::testing::Test
{
protected:
  void SetUp() override {}
};

// Test format_shape_string with various shapes
TEST_F(TestOnnxShapeHandling, FormatShapeString)
{
  EXPECT_EQ(format_shape_string({1, 46}), "[1, 46]");
  EXPECT_EQ(format_shape_string({-1, 46}), "[dynamic, 46]");
  EXPECT_EQ(format_shape_string({46}), "[46]");
  EXPECT_EQ(format_shape_string({-1, -1}), "[dynamic, dynamic]");
  EXPECT_EQ(format_shape_string({}), "[]");
}

// Test resolve_dynamic_shape with various model shapes
TEST_F(TestOnnxShapeHandling, ResolveDynamicShape)
{
  size_t actual_size = 46;

  // Static shape (no changes)
  auto result = resolve_dynamic_shape({46}, actual_size);
  EXPECT_EQ(result, std::vector<int64_t>({46}));

  // Dynamic dimension
  result = resolve_dynamic_shape({-1}, actual_size);
  EXPECT_EQ(result, std::vector<int64_t>({46}));

  // Multiple dimensions with dynamic
  result = resolve_dynamic_shape({1, -1}, actual_size);
  EXPECT_EQ(result, std::vector<int64_t>({1, 46}));

  result = resolve_dynamic_shape({-1, 46}, actual_size);
  EXPECT_EQ(result, std::vector<int64_t>({46, 46}));
}

// Test shape size calculation and dynamic dimension detection
TEST_F(TestOnnxShapeHandling, CalculateModelInputSize)
{
  auto calculate_size = [](const std::vector<int64_t> & shape)
  {
    size_t model_input_size = 1;
    bool has_dynamic_dim = false;
    for (auto dim : shape)
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
    return std::make_pair(model_input_size, has_dynamic_dim);
  };

  auto [size1, dynamic1] = calculate_size({1, 46});
  EXPECT_FALSE(dynamic1);
  EXPECT_EQ(size1, 46u);

  auto [size2, dynamic2] = calculate_size({-1, 46});
  EXPECT_TRUE(dynamic2);
  EXPECT_EQ(size2, 46u);

  auto [size3, dynamic3] = calculate_size({46});
  EXPECT_FALSE(dynamic3);
  EXPECT_EQ(size3, 46u);
}

// Test dimension matching validation
TEST_F(TestOnnxShapeHandling, DimensionMatching)
{
  size_t expected_input_size = 46;  // 10 + 3*12 joints

  // Match case
  size_t model_input_size = 46;
  bool has_dynamic_dim = false;
  bool matches = (!has_dynamic_dim && model_input_size == expected_input_size);
  EXPECT_TRUE(matches);

  // Mismatch case
  model_input_size = 48;
  has_dynamic_dim = false;
  matches = (!has_dynamic_dim && model_input_size == expected_input_size);
  EXPECT_FALSE(matches);

  // Dynamic dimension skips validation
  has_dynamic_dim = true;
  bool should_validate = !has_dynamic_dim;
  EXPECT_FALSE(should_validate);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
