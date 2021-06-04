// Node for the Franka demo

#include <franka_controller.h>

int main(int argc, char* argv[])
{
    rclcpp::init(arc, argv);
    rclcpp::spin(std::make_shared<FrankaDemo>());
    rclcpp::shutdown();

    return 0;
}