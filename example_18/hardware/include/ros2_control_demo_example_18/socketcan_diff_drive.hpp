#ifndef ROS2_CONTROL_DEMO_EXAMPLE_18__SOCKETCAN_DIFF_DRIVE_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_18__SOCKETCAN_DIFF_DRIVE_HPP_

#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "ros2_control_demo_example_18/socketcan_transport.hpp"

namespace ros2_control_demo_example_18
{
class SocketCanDiffDriveHardware final : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::unique_ptr<CanTransport> transport_;
  std::string can_interface_;
};
}  // namespace ros2_control_demo_example_18

#endif
