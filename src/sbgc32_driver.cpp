#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "gimbal.hpp"
#include "gimbals/real.hpp"

class SBGC32Node : public rclcpp::Node
{

public:
    SBGC32Node() : Node("sbgc32") {
        std::cout << "Hello world" << std::endl;
        std::shared_ptr<Gimbal> gimbal = std::make_shared<RealGimbal>();
        gimbal->motorsOn();
    }


};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SBGC32Node>());
    rclcpp::shutdown();
    return 0;
}