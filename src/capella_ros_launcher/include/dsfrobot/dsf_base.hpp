#ifndef DSF_BASE_H
#define DSF_BASE_H

#include "rclcpp/rclcpp.hpp"
#include "capella_ros_msg/msg/velocities.hpp"
#include "capella_ros_msg/msg/battery.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <angles/angles.h>

class DsfBase : public rclcpp ::Node
{
public:
    DsfBase(std::string name);
    void velCallback(capella_ros_msg::msg::Velocities::SharedPtr vel);
    void batteryCallback(capella_ros_msg::msg::Battery::SharedPtr bat);
    void poseCalibrationCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose);

private:
    rclcpp::Subscription<capella_ros_msg::msg::Velocities>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<capella_ros_msg::msg::Battery>::SharedPtr battery_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_calib_sub_;//

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
//    tf2_ros::TransformBroadcaster odom_broadcaster_;

    float linear_scale_;
    float dsf_angle_scale_;
    float low_battery_;
    float steering_angle_;
    float linear_velocity_x_;
    float linear_velocity_y_;
    float angular_velocity_z_;
    rclcpp::Time last_vel_time_;
    rclcpp::Time last_pose_time_;
    float vel_dt_;
    double x_pos_;//X坐标
    double y_pos_;//Y坐标
    double heading_;//yaw
    bool isTimeInit = false;
    double covariance[5];
};

#endif
