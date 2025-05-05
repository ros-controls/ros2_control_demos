#ifndef MAVLINK_SUBSCRIBER_NODE_HPP
#define MAVLINK_SUBSCRIBER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MavLinkSubscriberNode : public rclcpp::Node
{
public:
    MavLinkSubscriberNode();
private:
    void mavlink_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mavlink_vel_subscriber_;
};

#endif  // MAVLINK_SUBSCRIBER_NODE_HPP
