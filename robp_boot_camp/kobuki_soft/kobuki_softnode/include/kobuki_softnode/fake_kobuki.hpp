/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef KOBUKI_SOFTNODE_FAKE_KOBUKI_HPP
#define KOBUKI_SOFTNODE_FAKE_KOBUKI_HPP

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kobuki_ros_interfaces/msg/motor_power.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// std
#include <memory>
#include <string>

namespace kobuki
{
enum { LEFT = 0, RIGHT = 1 };

class FakeKobuki : public rclcpp::Node
{
 public:
	FakeKobuki();

	~FakeKobuki();

 private:
	void update();

	void subscribeVelocityCommand(geometry_msgs::msg::Twist const &msg);

	void subscribeMotorPowerCommand(kobuki_ros_interfaces::msg::MotorPower const &msg);

	void updateJoint(unsigned int index, double &w, rclcpp::Duration step_time);

	void updateOdometry(double w_left, double w_right, rclcpp::Duration step_time);

	void updateTF(geometry_msgs::msg::TransformStamped &odom_tf);

 private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Time last_cmd_vel_time_;
	rclcpp::Time prev_update_time_;

	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr              vel_sub_;
	rclcpp::Subscription<kobuki_ros_interfaces::msg::MotorPower>::SharedPtr mp_sub_;

	sensor_msgs::msg::JointState joint_states_;
	nav_msgs::msg::Odometry      odom_;
	float                        odom_pose_[3];
	float                        odom_vel_[3];
	double                       pose_cov_[36];

	std::string wheel_joint_name_[2];
	float       wheel_speed_cmd_[2];
	float       wheel_separation_;
	float       wheel_diameter_;

	bool   motor_enabled_;
	double cmd_vel_timeout_;
};
}  // namespace kobuki
#endif  // KOBUKI_SOFTNODE_FAKE_KOBUKI_HPP
