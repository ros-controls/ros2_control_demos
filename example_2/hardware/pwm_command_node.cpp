// #include "rclcpp/rclcpp.hpp"
// #include "mavros_msgs/srv/param_set_v2.hpp"

// using namespace std::chrono_literals;

// class RcOverrideAndDisarmNode : public rclcpp::Node {
// public:
//   RcOverrideAndDisarmNode() : Node("rc_override_and_disarm_node") {
//     param_set_client_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");

//     // Wait for the service to be available
//     if (!param_set_client_->wait_for_service(2s)) {
//       RCLCPP_ERROR(this->get_logger(), "ParamSet service not available.");
//       return;
//     }

//     // Set a parameter (example)
//     set_param("SERVO3_TRIM", 1500);
//   }

// private:
//   void set_param(const std::string &param_id, double value) {
//     auto request = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
//     request->param_id = param_id;
    
//     // Set the value type to integer and assign the integer_value
//     request->value.type = 2; // integer value type
//     request->value.integer_value = static_cast<int32_t>(value);

//     auto result_future = param_set_client_->async_send_request(request);
//     if (result_future.wait_for(2s) == std::future_status::ready) {
//       auto response = result_future.get();
//       if (response->success) {
//         RCLCPP_INFO(this->get_logger(), "Successfully set parameter %s to %f", param_id.c_str(), value);
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set parameter %s", param_id.c_str());
//       }
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "ParamSet service call timed out.");
//     }
//   }

//   rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr param_set_client_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<RcOverrideAndDisarmNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }







#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/srv/param_set_v2.hpp"

using namespace std::chrono_literals;

class RcOverrideAndDisarmNode : public rclcpp::Node {
public:
  RcOverrideAndDisarmNode() : Node("rc_override_and_disarm_node") {
    param_set_client_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");

    if (!param_set_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "ParamSet service not available.");
      return;
    }

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&RcOverrideAndDisarmNode::cmd_vel_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Subscribed to /cmd_vel. Ready to convert Twist to param set.");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double trim_servo1 = 1500.0 + msg->linear.x * 100.0;  // Mapping for linear.x
    double trim_servo2 = 1500.0 + msg->angular.z * 100.0; // Mapping for angular.z

    trim_servo1 = std::clamp(trim_servo1, 1000.0, 2000.0);
    trim_servo2 = std::clamp(trim_servo2, 1000.0, 2000.0);

    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f → SERVO1_TRIM=%.2f, angular.z=%.2f → SERVO2_TRIM=%.2f",
                msg->linear.x, trim_servo1, msg->angular.z, trim_servo2);

    set_param("SERVO1_TRIM", trim_servo1);
    set_param("SERVO2_TRIM", trim_servo2);
  }

  void set_param(const std::string &param_id, double value) {
    auto request = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
    request->param_id = param_id;

    request->value.type = 2; // MAV_PARAM_TYPE_INT32
    request->value.integer_value = static_cast<int32_t>(value);

    auto future = param_set_client_->async_send_request(request);
    if (future.wait_for(2s) == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Set %s = %d", param_id.c_str(), static_cast<int>(value));
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set parameter %s", param_id.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Timeout setting parameter %s", param_id.c_str());
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr param_set_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RcOverrideAndDisarmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}























// Code to send PWM value over to the param topic
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "mavros_msgs/srv/param_set_v2.hpp"

// using namespace std::chrono_literals;

// class RcOverrideAndDisarmNode : public rclcpp::Node {
// public:
//   RcOverrideAndDisarmNode()
//   : Node("rc_override_and_disarm_node"), base_pwm_(1500.0) {

//     param_set_client_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");

//     if (!param_set_client_->wait_for_service(2s)) {
//       RCLCPP_ERROR(this->get_logger(), "ParamSet service not available.");
//       return;
//     }

//     cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//       "/cmd_vel", 10,
//       std::bind(&RcOverrideAndDisarmNode::cmd_vel_callback, this, std::placeholders::_1)
//     );

//     RCLCPP_INFO(this->get_logger(), "Node started. Listening to /cmd_vel...");
//   }

// private:
//   void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//     // Extract speed and turn from the message
//     double speed = msg->linear.x;
//     double turn = msg->angular.z;

//     // Map the speed and turn to appropriate PWM ranges with smaller intervals
//     double pwm = base_pwm_;

//     // Adjust PWM based on speed (linear.x) with a smaller multiplier
//     pwm += speed * 100.0;  // Smaller multiplier for smaller interval

//     // Calculate PWM adjustment for turning (angular.z) with a smaller range
//     double turn_pwm = 1500.0 + (turn * 100.0);  // Adjust the multiplier for turn

//     // Print the calculated PWM values for both speed and turn
//     RCLCPP_INFO(this->get_logger(), "Received cmd_vel: speed = %.2f, turn = %.2f", speed, turn);
//     RCLCPP_INFO(this->get_logger(), "Setting PWM: speed_pwm = %.2f, turn_pwm = %.2f", pwm, turn_pwm);

//     // Here you can add logic to send these PWM values to your hardware.
//   }

//   void set_param(const std::string &param_id, double value) {
//     auto request = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
//     request->param_id = param_id;
//     request->value.type = 3; // MAV_PARAM_TYPE_REAL32
//     request->value.double_value = value;

//     auto future = param_set_client_->async_send_request(request);
//     if (future.wait_for(30s) == std::future_status::ready) {
//       auto response = future.get();
//       if (response->success) {
//         RCLCPP_INFO(this->get_logger(), "Successfully set %s to %.2f", param_id.c_str(), value);
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set parameter %s", param_id.c_str());
//       }
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "ParamSet service call timed out.");
//     }
//   }

//   rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr param_set_client_;
//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
//   double base_pwm_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<RcOverrideAndDisarmNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


// Code to scales the values
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"

// using namespace std::chrono_literals;

// class TeleopSubscriberNode : public rclcpp::Node {
// public:
//   TeleopSubscriberNode()
//   : Node("teleop_subscriber_node"), base_pwm_(1500.0) {  // Starting PWM value
//     // Create a subscription to /cmd_vel
//     cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//       "/cmd_vel", 10,
//       std::bind(&TeleopSubscriberNode::cmd_vel_callback, this, std::placeholders::_1)
//     );

//     RCLCPP_INFO(this->get_logger(), "Node started. Listening to /cmd_vel...");
//   }

// private:
//   void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//     // Extract speed and turn from the message
//     double speed = msg->linear.x;
//     double turn = msg->angular.z;

//     // Map the speed and turn to appropriate PWM ranges with smaller intervals
//     double pwm = base_pwm_;

//     // Adjust PWM based on speed (linear.x) with a smaller multiplier
//     pwm += speed * 100.0;  // Smaller multiplier for smaller interval

//     // Calculate PWM adjustment for turning (angular.z) with a smaller range
//     double turn_pwm = 1500.0 + (turn * 100.0);  // Adjust the multiplier for turn

//     // Print the calculated PWM values for both speed and turn
//     RCLCPP_INFO(this->get_logger(), "Received cmd_vel: speed = %.2f, turn = %.2f", speed, turn);
//     RCLCPP_INFO(this->get_logger(), "Setting PWM: speed_pwm = %.2f, turn_pwm = %.2f", pwm, turn_pwm);

//     // Here you can add logic to send these PWM values to your hardware.
//   }



//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
//   double base_pwm_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<TeleopSubscriberNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }































// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "mavros_msgs/msg/override_rc_in.hpp"
// #include "mavros_msgs/srv/param_set_v2.hpp"

// using namespace std::chrono_literals;

// class RcOverrideAndDisarmNode : public rclcpp::Node {
// public:
//   RcOverrideAndDisarmNode()
//   : Node("rc_override_and_disarm_node"), base_pwm_(1500.0) {
    
//     // Publisher for RC override
//     rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);

//     // Service client for param setting (optional)
//     param_set_client_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");

//     if (!param_set_client_->wait_for_service(2s)) {
//       RCLCPP_WARN(this->get_logger(), "ParamSet service not available (optional). Continuing anyway.");
//     }

//     // Subscriber to /cmd_vel
//     cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//       "/cmd_vel", 10,
//       std::bind(&RcOverrideAndDisarmNode::cmd_vel_callback, this, std::placeholders::_1)
//     );

//     RCLCPP_INFO(this->get_logger(), "Node started. Listening to /cmd_vel...");
//   }

// private:
//   void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//     double speed = msg->linear.x;
//     double turn = msg->angular.z;

//     double speed_pwm = base_pwm_ + speed * 100.0;
//     double turn_pwm = base_pwm_ + turn * 100.0;

//     // Clip values to valid PWM range (usually 1000 to 2000 for RC)
//     speed_pwm = std::clamp(speed_pwm, 1000.0, 2000.0);
//     turn_pwm = std::clamp(turn_pwm, 1000.0, 2000.0);

//     // Create and populate RC override message
//     auto rc_msg = mavros_msgs::msg::OverrideRCIn();
//     rc_msg.channels.fill(0);  // Fill all channels with 0 (do not override)

//     // Example channel mapping:
//     rc_msg.channels[0] = static_cast<uint16_t>(turn_pwm);   // Roll (CH1)
//     rc_msg.channels[1] = static_cast<uint16_t>(speed_pwm);  // Pitch (CH2)

//     rc_override_pub_->publish(rc_msg);

//     RCLCPP_INFO(this->get_logger(), "cmd_vel: speed=%.2f, turn=%.2f | PWM: speed=%.2f, turn=%.2f",
//                 speed, turn, speed_pwm, turn_pwm);
//   }



//   void set_param(const std::string &param_id, double value) {
//     auto request = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
//     request->param_id = "SERVO1_TRIM";
//     request->value.type = 3; // MAV_PARAM_TYPE_REAL32
//     request->value.double_value = int(value);

//     auto future = param_set_client_->async_send_request(request);
//     if (future.wait_for(5s) == std::future_status::ready) {
//       auto response = future.get();
//       if (response->success) {
//         RCLCPP_INFO(this->get_logger(), "Successfully set param %s to %.2f", param_id.c_str(), value);
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set parameter %s", param_id.c_str());
//       }
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "ParamSet service call timed out.");
//     }
//   }

//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
//   rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
//   rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr param_set_client_;
//   double base_pwm_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<RcOverrideAndDisarmNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
