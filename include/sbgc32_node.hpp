#ifndef SBGC32_NODE_HPP
#define SBGC32_NODE_HPP

#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_interfaces/msg/gimbal_angles.hpp"

#include "gimbal.hpp"
#include "gimbals/fake.hpp"

using namespace std::chrono_literals;

class SBGC32Node : public rclcpp::Node
{
public:
    SBGC32Node();

private:
    void publish_angles() const;
    void target_angle_callback(const gimbal_interfaces::msg::GimbalAngles::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr anglePubTimer;
    rclcpp::Publisher<gimbal_interfaces::msg::GimbalAngles>::SharedPtr anglePublisher;

    rclcpp::Subscription<gimbal_interfaces::msg::GimbalAngles>::SharedPtr targetAngleSubscriber;

    std::shared_ptr<Gimbal> gimbal;
};

#endif