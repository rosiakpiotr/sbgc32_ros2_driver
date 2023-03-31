#include "sbgc32_node.hpp"

SBGC32Node::SBGC32Node() : Node("sbgc32")
{
    gimbal = std::make_shared<FakeGimbal>();

    anglePublisher = this->create_publisher<geometry_msgs::msg::Vector3>("/gimbal/angles", 10);
    anglePubTimer = this->create_wall_timer(8ms, std::bind(&SBGC32Node::publish_angles, this));

    targetAngleSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/gimbal/target",
        10,
        std::bind(&SBGC32Node::target_angle_callback, this, std::placeholders::_1));
}

void SBGC32Node::publish_angles() const
{
    auto angles = gimbal->getCurrentAngles();

    auto message = geometry_msgs::msg::Vector3();
    message.x = angles.pitch;
    message.y = angles.yaw;
    message.z = angles.roll;

    anglePublisher->publish(message);
}

void SBGC32Node::target_angle_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    Angles target;
    target.pitch = msg->x;
    target.yaw = msg->y;
    target.roll = msg->z;
    gimbal->moveToAngles(target);

    std::stringstream ss;
    ss << "New target angles set: " << target;
    RCLCPP_INFO(this->get_logger(), ss.str());
}
