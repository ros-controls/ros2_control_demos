#ifndef MAVLINK_SUBSCRIBER_NODE_HPP
#define MAVLINK_SUBSCRIBER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "mavlink.h"  // Include MAVLink header for message definitions
// #include "wddw.h"

class MavLinkSubscriberNode : public rclcpp::Node
{
public:
    MavLinkSubscriberNode();
private:
    void mavlink_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mavlink_vel_subscriber_;

    void send_empty_message() {
        // std::string response = 
        // Send a mavlink message
        mavlink_system_t mavlink_system_;
        mavlink_message_t mavlink_message_;
        int serial_fd_;  // File descriptor for serial communication, example if using serial
    }

};

#endif  // MAVLINK_SUBSCRIBER_NODE_HPP
