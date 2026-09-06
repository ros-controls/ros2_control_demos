#include <gtest/gtest.h>
#include "ros2_control_demo_example_18/socketcan_transport.hpp"
using namespace ros2_control_demo_example_18;

class FakeTransport final : public CanTransport
{
public:
  bool open(const std::string & interface_name) override { opened = interface_name; return true; }
  ReceiveResult receive(WheelFeedback & value) override { value = feedback; return ReceiveResult::OK; }
  bool send(const WheelCommand & value) override { command = value; return true; }
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
