#ifndef SBGC32_NODE_HPP
#define SBGC32_NODE_HPP

#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/string.hpp>

#include "gimbal.hpp"
#include "gimbals/fake.hpp"

using namespace std::chrono_literals;

class SBGC32Node : public rclcpp::Node
{
public:
    SBGC32Node();

private:
    void publish_angles() const;
    void target_angle_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr anglePubTimer;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr anglePublisher;

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr targetAngleSubscriber;

    std::shared_ptr<Gimbal> gimbal;
};

#endif