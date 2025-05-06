#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rc_override_publisher");

  auto publisher = node->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
  rclcpp::Rate rate(10); // 10 Hz

  while (rclcpp::ok()) {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels.fill(0);         // Set all channels to "no override"
    msg.channels[0] = 1200;       // Channel 1 override
    msg.channels[2] = 1500;       // Channel 3 override

    RCLCPP_INFO(node->get_logger(), "Publishing RC override...");
    publisher->publish(msg);

    rclcpp::spin_some(node);
    rate.sleep(); // Maintain 10 Hz rate
  }

  rclcpp::shutdown();
  return 0;
}
