#ifndef SBGC32_NODE_HPP
#define SBGC32_NODE_HPP

#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_interfaces/msg/gimbal_angles.hpp"
#include "gimbal_interfaces/srv/motors_state.hpp"

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

    void motorsStateSrvCallback(
        const gimbal_interfaces::srv::MotorsState::Request::SharedPtr req,
        gimbal_interfaces::srv::MotorsState::Response::SharedPtr resp);

    rclcpp::TimerBase::SharedPtr anglePubTimer;
    rclcpp::Publisher<gimbal_interfaces::msg::GimbalAngles>::SharedPtr anglePublisher;

    rclcpp::Subscription<gimbal_interfaces::msg::GimbalAngles>::SharedPtr targetAngleSubscriber;
    rclcpp::Service<gimbal_interfaces::srv::MotorsState>::SharedPtr motorsStateService;

    std::shared_ptr<Gimbal> gimbal;
};

#endif