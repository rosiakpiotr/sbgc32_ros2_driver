#include <rclcpp/rclcpp.hpp>

#include "sbgc32_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SBGC32Node>());
    rclcpp::shutdown();
    return 0;
}