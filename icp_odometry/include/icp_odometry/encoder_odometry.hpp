#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <memory>
#include <cmath>

#include  "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "robp_interfaces/msg/encoders.hpp"

namespace icp_odometry
{

class EncoderOdometry : public rclcpp::Node
{
public:
  EncoderOdometry();

private:
    void encoderCallback(const robp_interfaces::msg::Encoders::SharedPtr msg);
    void broadcastTransform(const builtin_interfaces::msg::Time & stamp, double x, double y, double yaw);
    void publishPath(const builtin_interfaces::msg::Time & stamp, double x, double y, double yaw);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<robp_interfaces::msg::Encoders>::SharedPtr encoder_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    double x_;
    double y_;
    double yaw_;
    builtin_interfaces::msg::Time stamp_;
    nav_msgs::msg::Path path_;

    int64_t ticks_per_revolution_;
    double wheel_radius_;
    double wheel_base_;
    int frequency_;
};

}
#endif // ODOMETRY_HPP
