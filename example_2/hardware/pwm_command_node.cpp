#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/srv/param_set_v2.hpp"
// #include "mavros_msgs/msg/override_rc_in.hpp

using namespace std::chrono_literals;

class RcOverrideAndDisarmNode : public rclcpp::Node {
public:
  RcOverrideAndDisarmNode() : Node("rc_override_and_disarm_node") {
    param_set_client_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");

    if (!param_set_client_->wait_for_service(60s)) {
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
    double trim_servo2 = 1500.0 + msg->linear.x * 100.0; // Mapping for angular.z

    trim_servo1 = std::clamp(trim_servo1, 1000.0, 2000.0);
    trim_servo2 = std::clamp(trim_servo2, 1000.0, 2000.0);

    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f → SERVO1_TRIM=%.2f, angular.z=%.2f → SERVO2_TRIM=%.2f",
                msg->linear.x, trim_servo1, msg->angular.z, trim_servo2);

    set_param("SERVO1_TRIM", trim_servo1);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
