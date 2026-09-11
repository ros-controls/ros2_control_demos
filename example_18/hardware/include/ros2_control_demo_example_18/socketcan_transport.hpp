#ifndef ROS2_CONTROL_DEMO_EXAMPLE_18__SOCKETCAN_TRANSPORT_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_18__SOCKETCAN_TRANSPORT_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace ros2_control_demo_example_18
{
struct WheelFeedback
{
  double left_velocity{};
  double right_velocity{};
};

struct WheelCommand
{
  double left_velocity{};
  double right_velocity{};
};

class CanTransport
{
public:
  enum class ReceiveResult
  {
    OK,
    NO_DATA,
    ERROR
  };
  virtual ~CanTransport() = default;
  virtual bool open(const std::string & interface_name) = 0;
  virtual ReceiveResult receive(WheelFeedback & feedback) = 0;
  virtual bool send(const WheelCommand & command) = 0;
  virtual void close() noexcept = 0;
};

// Wire format used by this example: two little-endian int32 values in millirad/s.
std::array<std::uint8_t, 8> encode_wheel_command(const WheelCommand & command);
WheelFeedback decode_wheel_feedback(const std::array<std::uint8_t, 8> & payload);

class SocketCanTransport final : public CanTransport
{
public:
  ~SocketCanTransport() override { close(); }
  bool open(const std::string & interface_name) override;
  ReceiveResult receive(WheelFeedback & feedback) override;
  bool send(const WheelCommand & command) override;
  void close() noexcept override;

private:
  int socket_{-1};
};
}  // namespace ros2_control_demo_example_18

#endif
