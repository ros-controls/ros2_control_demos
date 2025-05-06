// This ROS2 node subscribes to the /cmd_vel topic and processes incoming Twist messages to control the movement
// of Asterius Mk2. It is designed to work with the hardware interface.

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// #include "mavros_msgs/msg/mavlink.hpp"
extern "C" {
  #include <mavlink/v2.0/common/mavlink.h>
}
// #include "mavros/mavros_router.hpp"
// #include "mavros/mavros_uas.hpp"
// #include "include/ros2_control_demo_example_2/mavlink_subscriber_node.hpp"
// #include "include/libmavconn/interface.hpp"

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

    mavlink_message_t mavlink_msg;

    mavlink_set_position_target_local_ned_t target;

    // Fill the target message with received velocities
    target.time_boot_ms = 0; // Optional: time field (use 0 for now)
    target.target_system = 1; // System ID
    target.target_component = 1; // Component ID
    target.coordinate_frame = MAV_FRAME_LOCAL_NED;
    // target.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_T_MASK_VELOCITY;

    target.vx = msg->linear.x;  // Linear X velocity (m/s)
    target.vy = 0;              // Linear Y velocity (m/s)
    target.vz = 0;              // Linear Z velocity (m/s)
    target.afx = 0;             // Accelerations
    target.afy = 0;
    target.afz = 0;
    target.yaw = msg->angular.z;  // Angular velocity (yaw)
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
