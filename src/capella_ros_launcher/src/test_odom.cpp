#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "capella_ros_msg/msg/attitude_par.hpp"

#define POSITION_SIZE 6

using std::placeholders::_1;

class TestOdomNode : public rclcpp::Node
{
   public:
   TestOdomNode(std::string name) : Node(name)
   {
      RCLCPP_INFO(this->get_logger(), "大家好，我是 %s !", name.c_str());
      //create tf listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      sub_novel = this->create_subscription<nav_msgs::msg::Odometry>("odom", 
                        10, std::bind(&TestOdomNode::topic_callback, this, _1));

      pub_odom_att_ = this->create_publisher<capella_ros_msg::msg::AttitudePar>
                      ("odom_atti", 100);
      pub_odom_att1_ = this->create_publisher<capella_ros_msg::msg::AttitudePar>
                      ("odom_atti_1", 100);

   }

   private:
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_novel;
   rclcpp::Publisher<capella_ros_msg::msg::AttitudePar>::SharedPtr pub_odom_att_;
   rclcpp::Publisher<capella_ros_msg::msg::AttitudePar>::SharedPtr pub_odom_att1_;

   std::string fromFrameRel = "base_link";
   std::string toFrameRel   = "odom";
   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
   std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

   void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
   {
        nav_msgs::msg::Odometry rcv_odom;
        capella_ros_msg::msg::AttitudePar odom_at;
        double pitch = 0.0;
        double yaw = 0.0;
        double roll = 0.0;

        tf2::Quaternion orientation;
        tf2::fromMsg(msg->pose.pose.orientation, orientation);
        
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        odom_at.roll = roll;
        odom_at.pitch = pitch;
        odom_at.yaw = yaw;
        pub_odom_att_->publish(odom_at);

        geometry_msgs::msg::TransformStamped tfStamped;
        tf2::Quaternion qua;

        try 
        {
        tfStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        qua.setW(tfStamped.transform.rotation.w);
        qua.setX(tfStamped.transform.rotation.x);
        qua.setY(tfStamped.transform.rotation.y);
        qua.setZ(tfStamped.transform.rotation.z);
        tf2::Matrix3x3(qua).getRPY(roll, pitch, yaw);
        odom_at.roll = roll;
        odom_at.pitch = pitch;
        odom_at.yaw = yaw;
        pub_odom_att1_->publish(odom_at);
        } 
        catch (tf2::TransformException & ex) 
        {
            RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        }


        // rcv_odom.header.frame_id = msg->header.frame_id;
        // rcv_odom.child_frame_id = msg->child_frame_id;
        // rcv_odom.pose.pose.position.x = msg->pose.pose.position.x;
        // rcv_odom.pose.pose.position.y = msg->pose.pose.position.y;

        // RCLCPP_INFO(this->get_logger(),"Header_Frame_id: %s, Child_Frame_id: %s", rcv_odom.header.frame_id.c_str(), rcv_odom.child_frame_id.c_str());
        // RCLCPP_INFO(this->get_logger(),"X_odom: %lf, Y_odom: %lf", rcv_odom.pose.pose.position.x, rcv_odom.pose.pose.position.y);
   };

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TestOdomNode>("test_odom_node");
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}