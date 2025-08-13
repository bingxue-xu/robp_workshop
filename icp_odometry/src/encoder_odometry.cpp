#include "icp_odometry/encoder_odometry.hpp"

namespace icp_odometry
{
EncoderOdometry::EncoderOdometry(): Node("encoder_odometry"),
    x_(0.0), y_(0.0), yaw_(0.0)
{

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    encoder_sub_ = this->create_subscription<robp_interfaces::msg::Encoders>("/motor/encoders", 10,
        std::bind(&EncoderOdometry::encoderCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/encoder_path", 10);
    path_.header.frame_id = "odom";
    path_.header.stamp = this->get_clock()->now();

    this->declare_parameter("frequency", 20);
    this->declare_parameter("wheel_base", 0.311);
    this->declare_parameter("wheel_radius", 0.098425/2);
    this->declare_parameter("ticks_per_revolution", 48*64);

    frequency_ = this->get_parameter("frequency").as_int();
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    ticks_per_revolution_ = this->get_parameter("ticks_per_revolution").as_int();

    RCLCPP_INFO(this->get_logger(), "EncoderOdometry initialized with frequency: %d, wheel_base: %.3f, wheel_radius: %.3f, ticks_per_revolution: %ld",
        frequency_, wheel_base_, wheel_radius_, ticks_per_revolution_);
}


void EncoderOdometry::encoderCallback(const robp_interfaces::msg::Encoders::SharedPtr msg)
{
    int64_t delta_ticks_left = msg->delta_encoder_left;
    int64_t delta_ticks_right = msg->delta_encoder_right;

    double K = 2.0 * M_PI / static_cast<double>(ticks_per_revolution_);
    double D = (wheel_radius_/2.0) * K * (delta_ticks_left + delta_ticks_right);
    double delta_theta = (wheel_radius_/wheel_base_) * K * (delta_ticks_right - delta_ticks_left);

    x_ += D * std::cos(yaw_);
    y_ += D * std::sin(yaw_);
    yaw_ += delta_theta;

    stamp_ = msg->header.stamp;

    publishPath(stamp_, x_, y_, yaw_);
    broadcastTransform(stamp_, x_, y_, yaw_);
}

void EncoderOdometry::broadcastTransform(const builtin_interfaces::msg::Time & stamp, double x, double y, double yaw)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}

void EncoderOdometry::publishPath(const builtin_interfaces::msg::Time & stamp, double x, double y, double yaw)
{
    path_.header.stamp = stamp;
    path_.header.frame_id = "odom";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_.header;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.01;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path_.poses.push_back(pose);
    path_pub_->publish(path_);
}

} // namespace icp_odometry
