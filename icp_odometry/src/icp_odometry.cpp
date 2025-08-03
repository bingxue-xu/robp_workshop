#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"  
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "pcl/filters/voxel_grid.h"
#include "laser_geometry/laser_geometry.hpp"
#include "icp_odometry/icp_odometry.hpp"

using std::placeholders::_1;

ICPOdometry::ICPOdometry()
    : Node("icp_odometry"), publish_tf_(true), first_cloud_(true), has_last_stamp_(false)
    {
        this->declare_parameter("publish_tf", true);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ICPOdometry::scan_callback, this, _1));

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom/icp", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path/icp", 10);
        path_msg_.header.frame_id = "odom";
        
        last_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        current_pose_.setIdentity();
        RCLCPP_INFO(this->get_logger(), "ICPOdometry Node Initialized");
    }
    
void ICPOdometry::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received laser scan with %zu points", scan_msg->ranges.size());
        sensor_msgs::msg::PointCloud2 cloud;
        projector_.projectLaser(*scan_msg, cloud);
        cloud.header = scan_msg->header; 
        pointcloud_pub_->publish(cloud);

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud, *current_cloud);

        // Downsample
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(current_cloud);
        voxel.setLeafSize(0.2f, 0.2f, 0.2f);
        voxel.filter(*current_cloud);

        if (first_cloud_) {
            *last_cloud_ = *current_cloud;
            first_cloud_ = false;
            return;
        }

        if (!has_last_stamp_) {
            last_stamp_ = scan_msg->header.stamp;
            has_last_stamp_ = true;
            *last_cloud_ = *current_cloud;
            return;
        }

        // ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations(50);
        icp.setInputSource(current_cloud);
        icp.setInputTarget(last_cloud_);
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);

        if (!icp.hasConverged()) {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge");
            return;
        }

        Eigen::Matrix4f delta = icp.getFinalTransformation();
        current_pose_ = current_pose_ * delta;
        *last_cloud_ = *current_cloud;

        publish_odometry(scan_msg->header.stamp, delta, icp);
    }

void ICPOdometry::publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& delta,
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = current_pose_(0, 3);
    odom.pose.pose.position.y = current_pose_(1, 3);
    odom.pose.pose.position.z = current_pose_(2, 3);

    Eigen::Matrix3f rot = current_pose_.block<3,3>(0,0);
    Eigen::Quaternionf q(rot);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    double dt = (stamp - last_stamp_).seconds();
    last_stamp_ = stamp;

    float dx = delta(0, 3);
    float dy = delta(1, 3);
    float dz = delta(2, 3);
    odom.twist.twist.linear.x = dx / dt;
    odom.twist.twist.linear.y = dy / dt;
    odom.twist.twist.linear.z = dz / dt;

    Eigen::AngleAxisf rot_vec(delta.block<3,3>(0,0));
    float angular_velocity = rot_vec.angle() / dt; 
    odom.twist.twist.angular.z = angular_velocity;

    setICPCovariance(odom, icp);

    odom_pub_->publish(odom);
    // RCLCPP_INFO(this->get_logger(), "Publishing odom to /odom/icp");

    // Broadcast TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = odom.pose.pose.position.x;
    t.transform.translation.y = odom.pose.pose.position.y;
    t.transform.translation.z = odom.pose.pose.position.z;
    t.transform.rotation = odom.pose.pose.orientation;
    if (publish_tf_) {
        tf_broadcaster_->sendTransform(t);
        RCLCPP_INFO(this->get_logger(), "Publishing TF from odom to base_link");
    } else {
        RCLCPP_INFO(this->get_logger(), "Skipping TF publishing");
        return;
    }

    // Path 
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose = odom.pose.pose;
    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(pose_stamped);
    path_pub_->publish(path_msg_);
}

void ICPOdometry::setICPCovariance(nav_msgs::msg::Odometry& odom, 
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp) {

    std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
    std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);

    if (icp.hasConverged()) {
        double fitness_score = icp.getFitnessScore();

        double base_pos_var = 0.001 + fitness_score * 0.1;
        double base_ang_var = 0.0001 + fitness_score * 0.01;

        odom.pose.covariance[0] = base_pos_var; // x
        odom.pose.covariance[7] = base_pos_var; // y
        odom.pose.covariance[14] = 0.1; // z
        odom.pose.covariance[21] = 0.1; // roll
        odom.pose.covariance[28] = 0.1; // pitch
        odom.pose.covariance[35] = base_ang_var; // yaw

        odom.twist.covariance[0] = 0.1; // linear x
        odom.twist.covariance[7] = 0.1; // linear y
        odom.twist.covariance[14] = 0.5; // linear z
        odom.twist.covariance[21] = 0.5; // angular x
        odom.twist.covariance[28] = 0.5; // angular y
        odom.twist.covariance[35] = 0.1;

        RCLCPP_DEBUG(this->get_logger(), "ICPCovariance set with fitness score: %.4f", fitness_score);
    } else {
        odom.pose.covariance[0] = 1.0; // x
        odom.pose.covariance[7] = 1.0; // y
        odom.pose.covariance[14] = 0.1; // z
        odom.twist.covariance[0] = 1.0; // roll
        odom.twist.covariance[7] = 1.0; // pitch
        odom.twist.covariance[35] = 1.0; // yaw
        RCLCPP_WARN(this->get_logger(), "ICP did not converge, setting default covariance");
    }
}


