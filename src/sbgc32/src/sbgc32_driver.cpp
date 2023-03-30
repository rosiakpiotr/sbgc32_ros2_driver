#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class SBGC32Node : public rclcpp::Node
{

public:
    SBGC32Node() : Node("sbgc32") {
        std::cout << "Hello world" << std::endl;
    }


};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SBGC32Node>());
    rclcpp::shutdown();
    return 0;
}