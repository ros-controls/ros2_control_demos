// This ROS2 node subscribes to the /cmd_vel topic and processes incoming Twist messages to control the movement
// of Asterius Mk2. It is designed to work with the hardware interface.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MavLinkSubscriberNode : public rclcpp::Node {
public:
  MavLinkSubscriberNode()
  : Node("mavlink_subscriber_node") {
    mavlink_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&MavLinkSubscriberNode::mavlink_vel_callback, this, std::placeholders::_1));
  }

private:
  void mavlink_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x = %f, angular.z = %f",
                msg->linear.x, msg->angular.z);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mavlink_vel_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavLinkSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
