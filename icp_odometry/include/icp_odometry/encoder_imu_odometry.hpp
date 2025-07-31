#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <robp_interfaces/msg/encoders.hpp>
#include <nav_msgs/msg/path.hpp>

class EncoderImuOdometry : public rclcpp::Node
{
public:
  EncoderImuOdometry();

private:
    void encoderCallback(const robp_interfaces::msg::Encoders::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publishPathAndTF(const rclcpp::Time& stamp);

    double x_, y_, yaw_;
    double omega_;
    double yaw_init_;
    double drift_;
    bool use_imu_;
    bool save_drift_;
    bool imu_initialized_;
    int ticks_per_revolution_;
    double wheel_radius_;
    double wheel_base_;

    rclcpp::Time last_imu_time_;

    rclcpp::Subscription<robp_interfaces::msg::Encoders>::SharedPtr encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
