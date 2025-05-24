#include "rclcpp/rclcpp.hpp"
#include "dsfrobot/dsf_base.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

DsfBase::DsfBase(std::string name) : Node(name), linear_scale_(1.042), low_battery_(21.0), linear_velocity_x_(0.0), 
    linear_velocity_y_(0.0), angular_velocity_z_(0.0), dsf_angle_scale_(1.0843),
    last_vel_time_(0.0), vel_dt_(0.0), x_pos_(0.0), y_pos_(0.0), heading_(0.0), covariance{0.0, 0.0, 0.0, 0.0, 0.0}
{
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("raw_odom", 100);//创建里程计话题
    velocity_subscriber_ = this->create_subscription<capella_ros_msg::msg::Velocities>("raw_vel", 100, std::bind(&DsfBase::velCallback, this, _1));//订阅速度信息话题
    battery_sub_ = this->create_subscription<capella_ros_msg::msg::Battery>("battery", 10, std::bind(&DsfBase::batteryCallback, this, _1));//订阅电池电压信息话题
    pose_calib_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
                               std::bind(&DsfBase::poseCalibrationCallback, this, _1));

    this->declare_parameter("linear_scale", linear_scale_);
    this->declare_parameter("low_battery", low_battery_);
    this->declare_parameter("dsf_angle_scale", dsf_angle_scale_);

    this->get_parameter("linear_scale", linear_scale_);
    RCLCPP_INFO(this->get_logger(), "linear_scale value: %f", linear_scale_);

    this->get_parameter("low_battery", low_battery_);
    RCLCPP_INFO(this->get_logger(), "low_battery value: %f", low_battery_);

    this->get_parameter("dsf_angle_scale", dsf_angle_scale_);
    RCLCPP_INFO(this->get_logger(), "dsf_angle_scale value: %f", dsf_angle_scale_);
}

void DsfBase::batteryCallback(capella_ros_msg::msg::Battery::SharedPtr bat)
{
	float cur_battery = bat->battery;

	if (cur_battery < low_battery_)
    {
		RCLCPP_WARN(this->get_logger(), "Current Voltage: %f ,Low Battery, Low Battery  Please Recharge!!", cur_battery);
	}
}

void DsfBase::velCallback(capella_ros_msg::msg::Velocities::SharedPtr vel)
{
    rclcpp::Time current_time = this->get_clock()->now();

    linear_velocity_x_ = vel->linear_x * linear_scale_;
    linear_velocity_y_ = vel->linear_y * linear_scale_;
    angular_velocity_z_ = vel->angular_z * dsf_angle_scale_;
    if((angular_velocity_z_ < 0.02) && (angular_velocity_z_ > -0.02))
        angular_velocity_z_ = 0.0;

    if(!isTimeInit)
    {
        vel_dt_ = 0.0001;
        isTimeInit = true;
    }
    else
    {
        vel_dt_ = current_time.seconds() - last_vel_time_.seconds();
    }
    
    last_vel_time_ = current_time;

    double delta_heading = angular_velocity_z_ * vel_dt_ ; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_ ; //m
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_ ; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    heading_ = angles::normalize_angle(heading_);

    // RCLCPP_INFO(this->get_logger(), "X_Pos: %lf, Y_Pos: %lf", x_pos_, y_pos_);
    // RCLCPP_INFO(this->get_logger(), "Heading Angle: %f", heading_);

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    tf2::Quaternion q_;
    
    q_.setRPY(0.0, 0.0, heading_);
    q_.normalize();

    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q_);
    /*odom_quat.x = q_.x();
    odom_quat.y = q_.y();
    odom_quat.z = q_.z();
    odom_quat.w = q_.w();*/

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    // odom_broadcaster_.sendTransform(odom_trans);

    nav_msgs::msg::Odometry odom;
    
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;
    if(covariance[0] != 0.0)
    {
        odom.pose.covariance[0] = covariance[0];
    }
    else
    {
        odom.pose.covariance[0] = 0.0001;
    }

    if(covariance[1] != 0.0)
    {
        odom.pose.covariance[1] = covariance[1];
    }
    else
    {
        odom.pose.covariance[1] = 0.00001;
    }

    if(covariance[2] != 0.0)
    {
        odom.pose.covariance[6] = covariance[2];
    }
    else
    {
        odom.pose.covariance[6] = 0.000001;
    }

    if(covariance[3] != 0.0)
    {
        odom.pose.covariance[7] = covariance[3];
    }
    else
    {
        odom.pose.covariance[7] = 0.0001;
    }

    if(covariance[4] != 0.0)
    {
        odom.pose.covariance[35] = covariance[4];
    }
    else
    {
        odom.pose.covariance[35] = 0.0001;
    }

    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_publisher_->publish(odom);
}

void DsfBase::poseCalibrationCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
{
    tf2::Quaternion q_;
    double roll = 0.0;
    double pitch = 0.0;
    double mid_x = 0.0;
    double mid_y = 0.0;
    double yaw = 0.0;
    float vel_dt_;

    rclcpp::Time current_time = this->get_clock()->now();
    vel_dt_ = current_time.seconds() - last_pose_time_.seconds();

    if(vel_dt_ > 10.0)
    {
        last_pose_time_ = current_time;
            //get X Y
        mid_x = pose->pose.pose.position.x;
        mid_y = pose->pose.pose.position.y;

        //get Yaw
        q_.setX(pose->pose.pose.orientation.x);
        q_.setY(pose->pose.pose.orientation.y);
        q_.setZ(pose->pose.pose.orientation.z);
        q_.setW(pose->pose.pose.orientation.w);
        tf2::Matrix3x3(q_).getRPY(roll, pitch, yaw);

    }

    //get covariance
 //   covariance[0] = pose->pose.covariance[0];
 //   covariance[1] = pose->pose.covariance[1];
//    covariance[2] = pose->pose.covariance[6];
 //   covariance[3] = pose->pose.covariance[7];
 //   covariance[4] = pose->pose.covariance[35];    
}
