#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include <chrono>

using namespace std::chrono_literals;

class RcOverrideAndDisarmNode : public rclcpp::Node {
public:
  RcOverrideAndDisarmNode() : Node("rc_override_and_disarm_node") {
    // Publisher to publish RC output (RCOut)
    publisher_ = this->create_publisher<mavros_msgs::msg::RCOut>("/mavros/rc/out", 10);
    client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    timer_ = this->create_wall_timer(100ms, std::bind(&RcOverrideAndDisarmNode::publish_callback, this));
    start_time_ = this->now();
  }

private:
void publish_callback() {
  // Prepare RC output message (feedback from system)
  mavros_msgs::msg::RCOut msg;

  // Dynamically allocate memory for the channels array
  // msg.channels = (uint16_t*)malloc(18 * sizeof(uint16_t));  // Allocate memory for 18 channels

  // if (msg.channels == nullptr) {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for RC channels.");
  //   return;
  // }

  // Manually initialize all channels to 0 using a loop
  // for (int i = 0; i < 18; i++) {
  //   msg.channels[i] = 0;
  // }
  msg.channels.resize(18, 0);  // Resize the vector to 18 elements and initialize to 0

  // Set specific channels
  msg.channels[0] = 1769;  // Example: Set channel 0 to 1200
  msg.channels[2] = 1500;  // Example: Set channel 2 to 1500
  
  publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Publishing RC output...");
}



  void disarm() {
    if (!client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "Disarm service not available.");
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = false; // false = disarm

    auto result_future = client_->async_send_request(request);
    if (result_future.wait_for(2s) == std::future_status::ready) {
      auto response = result_future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Successfully disarmed the vehicle.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Disarm request failed.");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Disarm service call timed out.");
    }
  }

  rclcpp::Publisher<mavros_msgs::msg::RCOut>::SharedPtr publisher_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  bool disarmed_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RcOverrideAndDisarmNode>());
  rclcpp::shutdown();
  return 0;
}
