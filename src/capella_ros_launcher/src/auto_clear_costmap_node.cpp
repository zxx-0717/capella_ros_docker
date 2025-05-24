#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


class AutoClearCostmapNode:public rclcpp::Node
{
private:
        rclcpp::TimerBase::SharedPtr auto_clear_costmap_timer;
        rclcpp::Client<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedPtr auto_clear_costmap_localclient;
        rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr auto_clear_costmap_globalclient;
public:
        AutoClearCostmapNode(std::string name_of_the_node):Node(name_of_the_node){

                auto_clear_costmap_timer = this->create_wall_timer(std::chrono::seconds(5),std::bind(&AutoClearCostmapNode::auto_clear_costmap_callback,this));

                // auto_clear_costmap_localclient = this->create_client<nav2_msgs::srv::ClearCostmapExceptRegion>("/local_costmap/clear_except_local_costmap");

                // auto_clear_costmap_globalclient = this->create_client<nav2_msgs::srv::ClearCostmapExceptRegion>("/global_costmap/clear_except_global_costmap");
                auto_clear_costmap_globalclient = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");

};
        ~AutoClearCostmapNode(){};
        
        int auto_clear_costmap_callback(){
                auto auto_clear_costmap_request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
                // auto_clear_costmap_request->reset_distance = 5.0;
                while (!auto_clear_costmap_globalclient->wait_for_service(1s)) {
                        if (!rclcpp::ok()) {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                        return 0;
                        }
                        RCLCPP_INFO(this->get_logger(), "costmap service not available, waiting again...");
                }
                // auto_clear_costmap_localclient->async_send_request(auto_clear_costmap_request);
                auto_clear_costmap_globalclient->async_send_request(auto_clear_costmap_request);
                RCLCPP_INFO(this->get_logger(), "Costmap clear message sent !");
                return 0;
        };
};

int main(int argc, char **argv){
        rclcpp::init(argc, argv);
        auto node = std::make_shared<AutoClearCostmapNode>("AutoClearCostmapNode");
        rclcpp::spin(node);
        rclcpp::shutdown();
}




