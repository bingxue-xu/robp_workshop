// ROS
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

// STL
#include <cmath>
#include <memory>

class World : public rclcpp::Node
{
public:
    World() : Node("world")
    {
        // 初始化 TF 缓存与监听器（用于机器人重置）
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 发布圆形墙（LINE_STRIP）和重置机器人命令
        vis_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("wall_marker", 10);
        reset_pub_ = this->create_publisher<std_msgs::msg::Empty>("reset_distance", 10);
        reset_srv_ = this->create_service<std_srvs::srv::Empty>(
            "reset_world",
            std::bind(&World::reset, this, std::placeholders::_1, std::placeholders::_2)
        );

        // 配置 wall_marker 为 LINE_STRIP
        wall_marker_.header.frame_id = "odom";
        wall_marker_.ns     = "world";
        wall_marker_.id     = 0;
        wall_marker_.type   = visualization_msgs::msg::Marker::LINE_STRIP;
        wall_marker_.action = visualization_msgs::msg::Marker::ADD;
        // 线宽 = 0.05 米
        wall_marker_.scale.x = 0.05;
        // 颜色：black
        wall_marker_.color.a = 1.0;
        wall_marker_.color.r = 0.0;
        wall_marker_.color.g = 0.0;
        wall_marker_.color.b = 0.0;

        // 构建经过原点的圆环点集合，点高度 z=0.01
        const double R = 5.0;
        const double cx = 0.0, cy = R;
        const int segments = 360;
        for (int i = 0; i <= segments; ++i) {
            double theta = 2 * M_PI * i / segments;
            geometry_msgs::msg::Point p;
            p.x = cx + R * std::cos(theta);
            p.y = cy + R * std::sin(theta);
            p.z = 0.01;  // 墙高度 0.01 米
            wall_marker_.points.push_back(p);
        }

        // 初始设置完成后通过定时器发布
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(100ms, std::bind(&World::publish, this));
    }

    void publish()
    {
        // 更新时间戳并发布墙体
        wall_marker_.header.stamp = this->now();
        vis_pub_->publish(wall_marker_);
    }

private:
    // 重置服务：仅发布机器人重置命令
    void reset(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        const std::shared_ptr<std_srvs::srv::Empty::Response>
    )
    {
        reset_pub_->publish(std_msgs::msg::Empty());
    }

    // 成员变量
    rclcpp::TimerBase::SharedPtr timer_;
    visualization_msgs::msg::Marker                           wall_marker_;
    std::shared_ptr<tf2_ros::TransformListener>               tf_listener_;
    std::unique_ptr<tf2_ros::Buffer>                         tf_buffer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr           reset_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr            reset_srv_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<World>());
    rclcpp::shutdown();
    return 0;
}
