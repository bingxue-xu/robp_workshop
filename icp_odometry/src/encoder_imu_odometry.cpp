#include "icp_odometry/encoder_imu_odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

EncoderImuOdometry::EncoderImuOdometry()
: Node("encoder_imu_odometry"),
  x_(0.0), y_(0.0), yaw_(0.0),
  omega_(0.0), yaw_init_(0.0), drift_(0.0),
  use_imu_(false), save_drift_(false), imu_initialized_(false)
  {

    this->declare_parameter("use_imu", true);
    this->declare_parameter("wheel_base", 0.311);
    this->declare_parameter("wheel_radius", 0.098425 / 2);
    this->declare_parameter("ticks_per_revolution", 48 * 64);

    use_imu_ = this->get_parameter("use_imu").as_bool();
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    ticks_per_revolution_ = this->get_parameter("ticks_per_revolution").as_int();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10,
        std::bind(&EncoderImuOdometry::imuCallback, this, std::placeholders::_1));
    encoder_sub_ = this->create_subscription<robp_interfaces::msg::Encoders>(
        "/motor/encoders", 10,
        std::bind(&EncoderImuOdometry::encoderCallback, this, std::placeholders::_1));
    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    path_.header.frame_id = "odom";

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    last_imu_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Encoder IMU Odometry Node Initialized");
  }

void EncoderImuOdometry::encoderCallback(const robp_interfaces::msg::Encoders::SharedPtr msg)
{
    int delta_l = msg->delta_encoder_left;
    int delta_r = msg->delta_encoder_right;

    bool is_stationary = (std::abs(delta_l) < 25 && std::abs(delta_r) < 25 && std::abs(delta_l -delta_r) < 3);
    if (is_stationary) {
        if (!save_drift_) {
            RCLCPP_INFO(this->get_logger(), "Stationary calibration started");
        }
        save_drift_ = true;
        } else {
            if (save_drift_) {
                RCLCPP_INFO(this->get_logger(), "Stationary calibration ended");
            }
            save_drift_ = false;
        }

    double K = 2 * M_PI / ticks_per_revolution_;
    double D = (wheel_radius_/2.0) * (K*(delta_r+delta_l));
    double delta_theta = (wheel_radius_ / wheel_base_)*(K*(delta_r - delta_l));

    x_ += D * std::cos(yaw_);
    y_ += D * std::sin(yaw_);

    if (use_imu_){
    } else{
        yaw_ += delta_theta;
    }

    publishPathAndTF(msg->header.stamp);
}

void EncoderImuOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // rclcpp::Time now_time = this->now();
    // double dt = (now_time - last_imu_time_).seconds();
    // last_imu_time_ = now_time;

    // omega_ = msg->angular_velocity.z;

    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if (!imu_initialized_) {
        yaw_init_ = yaw;
        imu_initialized_ = true;
        return;
    }
    
    double imu_yaw_change = yaw - yaw_init_;

    if (save_drift_) {
        drift_ = yaw_ - imu_yaw_change;
    }
    if (use_imu_) {
        yaw_ = imu_yaw_change + drift_;
    }
}

void EncoderImuOdometry::publishPathAndTF(const rclcpp::Time& stamp)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "odom";
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.position.z = 0.01;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    pose.pose.orientation = tf2::toMsg(q);

    path_.poses.push_back(pose);
    path_.header.stamp = stamp;
    path_pub_->publish(path_);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.01;
    transform.transform.rotation = pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
    RCLCPP_INFO(this->get_logger(), "Published path and TF");
}
