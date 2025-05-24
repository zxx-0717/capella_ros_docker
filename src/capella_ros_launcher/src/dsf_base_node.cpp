#include "rclcpp/rclcpp.hpp"
#include "dsfrobot/dsf_base.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DsfBase>("dsf_base_node");
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}