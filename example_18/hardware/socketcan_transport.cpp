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

#include "ros2_control_demo_example_18/socketcan_transport.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ros2_control_demo_example_18
{
namespace
{
constexpr canid_t kCommandId = 0x201;
constexpr canid_t kFeedbackId = 0x181;
constexpr double kScale = 1000.0;

std::int32_t decode_i32(const std::uint8_t * bytes)
{
  std::uint32_t value =
    static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8) |
    (static_cast<std::uint32_t>(bytes[2]) << 16) | (static_cast<std::uint32_t>(bytes[3]) << 24);
  return static_cast<std::int32_t>(value);
}
void encode_i32(std::uint8_t * bytes, std::int32_t value)
{
  const auto v = static_cast<std::uint32_t>(value);
  bytes[0] = static_cast<std::uint8_t>(v);
  bytes[1] = static_cast<std::uint8_t>(v >> 8);
  bytes[2] = static_cast<std::uint8_t>(v >> 16);
  bytes[3] = static_cast<std::uint8_t>(v >> 24);
}
}  // namespace

std::array<std::uint8_t, 8> encode_wheel_command(const WheelCommand & command)
{
  std::array<std::uint8_t, 8> result{};
  const auto to_wire = [](double value)
  {
    if (std::isnan(value))
    {
      return std::int32_t{0};
    }
    if (value == std::numeric_limits<double>::infinity())
    {
      return std::numeric_limits<std::int32_t>::max();
    }
    if (value == -std::numeric_limits<double>::infinity())
    {
      return std::numeric_limits<std::int32_t>::min();
    }
    const double scaled = std::round(value * kScale);
    if (scaled >= static_cast<double>(std::numeric_limits<std::int32_t>::max()))
    {
      return std::numeric_limits<std::int32_t>::max();
    }
    if (scaled <= static_cast<double>(std::numeric_limits<std::int32_t>::min()))
    {
      return std::numeric_limits<std::int32_t>::min();
    }
    return static_cast<std::int32_t>(scaled);
  };
  encode_i32(result.data(), to_wire(command.left_velocity));
  encode_i32(result.data() + 4, to_wire(command.right_velocity));
  return result;
}

WheelFeedback decode_wheel_feedback(const std::array<std::uint8_t, 8> & payload)
{
  return {decode_i32(payload.data()) / kScale, decode_i32(payload.data() + 4) / kScale};
}

bool SocketCanTransport::open(const std::string & interface_name)
{
  close();
  socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0)
  {
    return false;
  }
  const can_err_mask_t error_mask = CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_RESTARTED;
  if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask, sizeof(error_mask)) < 0)
  {
    close();
    return false;
  }
  const int flags = ::fcntl(socket_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK) < 0)
  {
    close();
    return false;
  }
  ifreq request{};
  std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_, SIOCGIFINDEX, &request) < 0)
  {
    close();
    return false;
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
  {
    close();
    return false;
  }
  return true;
}

CanTransport::ReceiveResult SocketCanTransport::receive(WheelFeedback & feedback)
{
  can_frame frame{};
  const auto received = ::recv(socket_, &frame, sizeof(frame), MSG_DONTWAIT);
  if (received < 0)
  {
    return errno == EAGAIN || errno == EWOULDBLOCK ? ReceiveResult::NO_DATA : ReceiveResult::ERROR;
  }
  if (received != sizeof(frame) || (frame.can_id & CAN_ERR_FLAG))
  {
    return ReceiveResult::ERROR;
  }
  if ((frame.can_id & CAN_EFF_MASK) != kFeedbackId || frame.can_dlc < 8)
  {
    return ReceiveResult::NO_DATA;
  }
  std::array<std::uint8_t, 8> payload{};
  std::memcpy(payload.data(), frame.data, payload.size());
  feedback = decode_wheel_feedback(payload);
  return ReceiveResult::OK;
}

bool SocketCanTransport::send(const WheelCommand & command)
{
  if (socket_ < 0)
  {
    return false;
  }
  can_frame frame{};
  frame.can_id = kCommandId;
  frame.can_dlc = 8;
  const auto payload = encode_wheel_command(command);
  std::memcpy(frame.data, payload.data(), payload.size());
  const auto sent = ::send(socket_, &frame, sizeof(frame), MSG_DONTWAIT);
  return sent == sizeof(frame);
}

void SocketCanTransport::close() noexcept
{
  if (socket_ >= 0)
  {
    ::close(socket_);
    socket_ = -1;
  }
}
}  // namespace ros2_control_demo_example_18
