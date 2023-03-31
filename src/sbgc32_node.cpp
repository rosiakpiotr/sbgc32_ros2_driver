#include "sbgc32_node.hpp"

SBGC32Node::SBGC32Node() : Node("sbgc32")
{
    gimbal = std::make_shared<FakeGimbal>();

    anglePublisher = this->create_publisher<gimbal_interfaces::msg::GimbalAngles>("/gimbal/angles", 10);
    anglePubTimer = this->create_wall_timer(8ms, std::bind(&SBGC32Node::publish_angles, this));

    targetAngleSubscriber = this->create_subscription<gimbal_interfaces::msg::GimbalAngles>(
        "/gimbal/target",
        10,
        std::bind(&SBGC32Node::target_angle_callback, this, std::placeholders::_1));
}

void SBGC32Node::publish_angles() const
{
    auto angles = gimbal->getCurrentAngles();

    auto message = gimbal_interfaces::msg::GimbalAngles();
    message.pitch = angles.pitch;
    message.yaw = angles.yaw;
    message.roll = angles.roll;

    anglePublisher->publish(message);
}

void SBGC32Node::target_angle_callback(const gimbal_interfaces::msg::GimbalAngles::SharedPtr msg)
{
    Angles target;
    target.pitch = msg->pitch;
    target.yaw = msg->yaw;
    target.roll = msg->roll;
    gimbal->moveToAngles(target);

    std::stringstream ss;
    ss << "New target angles set: " << target;
    RCLCPP_INFO(this->get_logger(), ss.str());
}
