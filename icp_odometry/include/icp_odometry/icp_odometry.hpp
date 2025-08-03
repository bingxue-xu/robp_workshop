#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <laser_geometry/laser_geometry.hpp>
#include <Eigen/Dense>

class ICPOdometry : public rclcpp::Node {
    public:
        ICPOdometry();

    private:
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
        void publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& delta,
                             pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp);
        void setICPCovariance(nav_msgs::msg::Odometry& odom, pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp);
        
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_;
        
        bool publish_tf_;
        bool first_cloud_;
        bool has_last_stamp_;
        Eigen::Matrix4f current_pose_;
        
        rclcpp::Time last_stamp_;
        nav_msgs::msg::Path path_msg_;
        laser_geometry::LaserProjection projector_;
};
