// #include "rclcpp/rclcpp.hpp"
// #include "mavros_msgs/msg/rc_out.hpp"
// #include "mavros_msgs/srv/command_bool.hpp"
// #include "mavros_msgs/srv/set_mode.hpp"
// #include "mavros_msgs/msg/param_event.hpp"

// #include <chrono>
// #include <thread>
// #include <mutex>
// #include <iostream>
// #include <atomic>
// #include <termios.h>
// #include <unistd.h>

// using namespace std::chrono_literals;

// class RcOverrideAndDisarmNode : public rclcpp::Node {
// public:
//   RcOverrideAndDisarmNode() : Node("rc_override_and_disarm_node") {
//     publisher_ = this->create_publisher<mavros_msgs::msg::RCOut>("/mavros/rc/out", 10);
//     client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
//     set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

//     // New: subscribe to ParamEvent
//     param_event_sub_ = this->create_subscription<mavros_msgs::msg::ParamEvent>(
//       "/mavros/param/event", 10,
//       std::bind(&RcOverrideAndDisarmNode::param_event_callback, this, std::placeholders::_1));

//     set_mode_to_manual();
//     RCLCPP_INFO(this->get_logger(), "Mode set to MANUAL");

//     timer_ = this->create_wall_timer(100ms, std::bind(&RcOverrideAndDisarmNode::publish_callback, this));
//     start_time_ = this->now();

//     // Launch thread to handle terminal input as an interrupt-like mechanism
//     input_thread_ = std::thread(&RcOverrideAndDisarmNode::handle_user_input, this);
//   }

//   ~RcOverrideAndDisarmNode() {
//     exit_flag_ = true;
//     if (input_thread_.joinable()) {
//       input_thread_.join();
//     }
//   }

// private:
//   void set_mode_to_manual() {
//     if (!set_mode_client_->wait_for_service(2s)) {
//       RCLCPP_ERROR(this->get_logger(), "Set mode service not available.");
//       return;
//     }

//     auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
//     request->base_mode = 0;
//     request->custom_mode = "MANUAL";

//     auto result_future = set_mode_client_->async_send_request(request);
//     if (result_future.wait_for(2s) == std::future_status::ready) {
//       auto response = result_future.get();
//       if (response->mode_sent) {
//         RCLCPP_INFO(this->get_logger(), "Mode set to MANUAL.");
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set mode to MANUAL.");
//       }
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "Set mode service call timed out.");
//     }
//   }

//   void publish_callback() {
//     mavros_msgs::msg::RCOut msg;
//     msg.channels.resize(18, 0);

//     std::lock_guard<std::mutex> lock(pwm_mutex_);
//     msg.channels[0] = channel0_pwm_;
//     msg.channels[2] = channel2_pwm_;

//     publisher_->publish(msg);
//     RCLCPP_INFO(this->get_logger(), "Publishing RC output: ch0=%d, ch2=%d", channel0_pwm_, channel2_pwm_);
//   }

//   void disarm() {
//     if (!client_->wait_for_service(2s)) {
//       RCLCPP_ERROR(this->get_logger(), "Disarm service not available.");
//       return;
//     }

//     auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
//     request->value = false;

//     auto result_future = client_->async_send_request(request);
//     if (result_future.wait_for(2s) == std::future_status::ready) {
//       auto response = result_future.get();
//       if (response->success) {
//         RCLCPP_INFO(this->get_logger(), "Successfully disarmed the vehicle.");
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "Disarm request failed.");
//       }
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "Disarm service call timed out.");
//     }
//   }

//   void arm() {
//     if (!client_->wait_for_service(2s)) {
//       RCLCPP_ERROR(this->get_logger(), "Arm service not available.");
//       return;
//     }

//     auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
//     request->value = true;

//     auto result_future = client_->async_send_request(request);
//     if (result_future.wait_for(2s) == std::future_status::ready) {
//       auto response = result_future.get();
//       if (response->success) {
//         RCLCPP_INFO(this->get_logger(), "Successfully armed the vehicle.");
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "Arm request failed.");
//       }
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "Arm service call timed out.");
//     }
//   }

//   // Function to capture keypress in a non-blocking manner (simulating an interrupt)
//   char get_keypress() {
//     struct termios oldt, newt;
//     char ch;
//     tcgetattr(STDIN_FILENO, &oldt);  // Get terminal settings
//     newt = oldt;
//     newt.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
//     tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // Apply new settings
//     ch = getchar();  // Get character
//     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Restore old terminal settings
//     return ch;
//   }

//   void handle_user_input() {
//     while (!exit_flag_) {
//       char key = get_keypress();  // Wait for a keypress (simulating an interrupt)

//       if (key == 'q') {  // Example: 'q' to disarm and exit
//         disarm();
//         break;
//       }
//       else if (key == 'u') {  // Example: 'u' to update PWM
//         int ch0, ch2;
//         std::cout << "\nEnter PWM for Channel 0 and 2 (e.g. 1500 1300): ";
//         std::cin >> ch0 >> ch2;

//         if (std::cin.fail()) {
//           std::cin.clear();
//           std::cin.ignore(1000, '\n');
//           std::cout << "Invalid input. Please enter two integers.\n";
//           continue;
//         }

//         {
//           std::lock_guard<std::mutex> lock(pwm_mutex_);
//           channel0_pwm_ = ch0;
//           channel2_pwm_ = ch2;
//         }

//         std::cout << "Updated: Channel 0 = " << ch0 << ", Channel 2 = " << ch2 << "\n";
//       }
//       else if (key == 'a') {  // Example: 'a' to arm
//         arm();
//       }
//     }
//   }

//   void param_event_callback(const mavros_msgs::msg::ParamEvent::SharedPtr msg) {
//     msg->param_id = "SERVO2_TRIM";
//     RCLCPP_INFO(this->get_logger(),
//                 "Param Event Received:\n"
//                 "  ID: %s\n"
//                 "  Index: %u / %u\n"
//                 "  Value Type: %u\n"
//                 "  Value (double): %f",
//                 msg->param_id.c_str(),
//                 msg->param_index,
//                 msg->param_count,
//                 msg->value.type,
//                 msg->value.double_value);
//   }

//   rclcpp::Publisher<mavros_msgs::msg::RCOut>::SharedPtr publisher_;
//   rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client_;
//   rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Time start_time_;
//   bool disarmed_ = false;

//   std::thread input_thread_;
//   std::mutex pwm_mutex_;
//   int channel0_pwm_ = 1589;
//   int channel2_pwm_ = 1580;
//   std::atomic<bool> exit_flag_{false};
//   rclcpp::Subscription<mavros_msgs::msg::ParamEvent>::SharedPtr param_event_sub_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<RcOverrideAndDisarmNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }


#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/param_set_v2.hpp"

using namespace std::chrono_literals;

class RcOverrideAndDisarmNode : public rclcpp::Node {
public:
  RcOverrideAndDisarmNode() : Node("rc_override_and_disarm_node") {
    param_set_client_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");

    // Wait for the service to be available
    if (!param_set_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "ParamSet service not available.");
      return;
    }

    // Set a parameter (example)
    set_param("SERVO1_TRIM", 1500);
  }

private:
  void set_param(const std::string &param_id, double value) {
    auto request = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
    request->param_id = param_id;
    
    // Set the value type to integer and assign the integer_value
    request->value.type = 2; // integer value type
    request->value.integer_value = static_cast<int32_t>(value);

    auto result_future = param_set_client_->async_send_request(request);
    if (result_future.wait_for(2s) == std::future_status::ready) {
      auto response = result_future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Successfully set parameter %s to %f", param_id.c_str(), value);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set parameter %s", param_id.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "ParamSet service call timed out.");
    }
  }

  rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr param_set_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RcOverrideAndDisarmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
