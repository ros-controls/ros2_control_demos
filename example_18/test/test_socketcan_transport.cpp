// Copyright 2026 ros2_control Development Team
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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "ros2_control_demo_example_18/socketcan_transport.hpp"

using ros2_control_demo_example_18::CanTransport;
using ros2_control_demo_example_18::decode_wheel_feedback;
using ros2_control_demo_example_18::encode_wheel_command;
using ros2_control_demo_example_18::SocketCanTransport;
using ros2_control_demo_example_18::WheelCommand;
using ros2_control_demo_example_18::WheelFeedback;

class FakeTransport final : public CanTransport
{
public:
  bool open(const std::string & interface_name) override
  {
    opened = interface_name;
    return true;
  }
  ReceiveResult receive(WheelFeedback & value) override
  {
    value = feedback;
    return ReceiveResult::OK;
  }
  bool send(const WheelCommand & value) override
  {
    command = value;
    return true;
  }
  void close() noexcept override { opened.clear(); }
  std::string opened;
  WheelFeedback feedback{0.5, -0.25};
  WheelCommand command{};
};
TEST(SocketCanTransport, EncodesAndDecodesSignedMilliradians)
{
  const WheelCommand command{1.234, -2.5};
  const auto payload = encode_wheel_command(command);
  const auto feedback = decode_wheel_feedback(payload);
  EXPECT_DOUBLE_EQ(feedback.left_velocity, 1.234);
  EXPECT_DOUBLE_EQ(feedback.right_velocity, -2.5);
}

TEST(SocketCanTransport, SaturatesWireRangeAndMapsNanToZero)
{
  const auto payload =
    encode_wheel_command({std::numeric_limits<double>::infinity(), std::nan("")});
  const auto feedback = decode_wheel_feedback(payload);
  EXPECT_DOUBLE_EQ(feedback.left_velocity, 2147483.647);
  EXPECT_DOUBLE_EQ(feedback.right_velocity, 0.0);
}

TEST(SocketCanTransport, SendFailsBeforeOpen)
{
  SocketCanTransport transport;
  EXPECT_FALSE(transport.send({1.0, 1.0}));
}

TEST(SocketCanTransport, CanBeReplacedWithoutCanHardware)
{
  FakeTransport transport;
  ASSERT_TRUE(transport.open("vcan0"));
  ASSERT_TRUE(transport.send({1.0, -1.0}));
  WheelFeedback feedback;
  ASSERT_EQ(transport.receive(feedback), CanTransport::ReceiveResult::OK);
  EXPECT_EQ(transport.opened, "vcan0");
  EXPECT_DOUBLE_EQ(transport.command.right_velocity, -1.0);
  EXPECT_DOUBLE_EQ(feedback.left_velocity, 0.5);
}
