#include <kobuki_softnode/fake_kobuki.hpp>

// ros
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// std
#include <chrono>

namespace kobuki
{
FakeKobuki::FakeKobuki() : Node("fake_kobuki")
{
	// something
	this->wheel_speed_cmd_[LEFT]  = 0.0;
	this->wheel_speed_cmd_[RIGHT] = 0.0;

	// using the same values as in kobuki_node
	double pcov[36] = {0.1, 0, 0, 0,   0, 0, 0, 0.1, 0, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
	                   0,   0, 0, 1e6, 0, 0, 0, 0,   0, 0, 1e6, 0, 0, 0, 0,   0, 0, 0.2};
	memcpy(&(this->odom_.pose.covariance), pcov, sizeof(double) * 36);
	memcpy(&(this->odom_.twist.covariance), pcov, sizeof(double) * 36);

	// wheel information from kobuki_gazebo
	this->wheel_separation_ = 0.23;
	this->wheel_diameter_   = 0.070;

	// parameters
	this->declare_parameter("wheel_left_joint_name", std::string("wheel_left_joint"));
	this->declare_parameter("wheel_right_joint_name", std::string("wheel_right_joint"));
	this->declare_parameter("cmd_vel_timeout", double{0.6});
	this->declare_parameter("odom_frame", std::string("odom"));
	this->declare_parameter("base_frame", std::string("base_footprint"));

	// joint states
	this->wheel_joint_name_[LEFT] =
	    this->get_parameter("wheel_left_joint_name").as_string();
	this->wheel_joint_name_[RIGHT] =
	    this->get_parameter("wheel_right_joint_name").as_string();
	this->cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
	this->cmd_vel_timeout_ = 1.0;

	this->motor_enabled_ = true;

	this->joint_states_.header.frame_id = "Joint States";
	this->joint_states_.name.push_back(wheel_joint_name_[LEFT]);
	this->joint_states_.name.push_back(wheel_joint_name_[RIGHT]);
	this->joint_states_.position.resize(2, 0.0);
	this->joint_states_.velocity.resize(2, 0.0);
	this->joint_states_.effort.resize(2, 0.0);

	// odometry
	this->odom_.header.frame_id = this->get_parameter("odom_frame").as_string();
	this->odom_.child_frame_id  = this->get_parameter("base_frame").as_string();

	this->odom_pose_[0] = 0;
	this->odom_pose_[1] = 0;
	this->odom_pose_[2] = 0;

	// initialize publishers
	this->js_pub_ =
	    this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);
	this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);

	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	// initialize subscribers
	this->vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
	    "/mobile_base/commands/velocity", 10,
	    std::bind(&FakeKobuki::subscribeVelocityCommand, this, std::placeholders::_1));
	this->mp_sub_ = this->create_subscription<kobuki_ros_interfaces::msg::MotorPower>(
	    "/mobile_base/commands/motor_power", 10,
	    std::bind(&FakeKobuki::subscribeMotorPowerCommand, this, std::placeholders::_1));

	this->prev_update_time_  = this->now();
	this->last_cmd_vel_time_ = this->now();

	// timer called 30 times a second
	using namespace std::chrono_literals;
	timer_ = this->create_wall_timer(1s / 30, std::bind(&FakeKobuki::update, this));
}

FakeKobuki::~FakeKobuki() {}

void FakeKobuki::subscribeVelocityCommand(geometry_msgs::msg::Twist const& msg)
{
	this->last_cmd_vel_time_ = this->now();
	this->wheel_speed_cmd_[LEFT] =
	    msg.linear.x - msg.angular.z * this->wheel_separation_ / 2;
	this->wheel_speed_cmd_[RIGHT] =
	    msg.linear.x + msg.angular.z * this->wheel_separation_ / 2;
}

void FakeKobuki::subscribeMotorPowerCommand(
    kobuki_ros_interfaces::msg::MotorPower const& msg)
{
	if ((kobuki_ros_interfaces::msg::MotorPower::ON == msg.state) &&
	    (!this->motor_enabled_)) {
		this->motor_enabled_ = true;
		RCLCPP_INFO(this->get_logger(), "Motors fire up.");
	} else if ((kobuki_ros_interfaces::msg::MotorPower::OFF == msg.state) &&
	           (this->motor_enabled_)) {
		this->motor_enabled_ = false;
		RCLCPP_INFO(this->get_logger(), "Motors take a break.");
	}
}

void FakeKobuki::updateJoint(unsigned int index, double& w, rclcpp::Duration step_time)
{
	double v                            = this->wheel_speed_cmd_[index];
	w                                   = v / (this->wheel_diameter_ / 2);
	this->joint_states_.velocity[index] = w;
	this->joint_states_.position[index] += w * step_time.seconds();
}

void FakeKobuki::updateOdometry(double w_left, double w_right, rclcpp::Duration step_time)
{
	auto createQuaternionMsgFromYaw = [](double yaw) {
		tf2::Quaternion q;
		q.setRPY(0, 0, yaw);
		return tf2::toMsg(q);
	};

	double d1, d2;
	double dr, da;
	d1 = d2 = 0;
	dr = da = 0;

	d1 = step_time.seconds() * (this->wheel_diameter_ / 2) * w_left;
	d2 = step_time.seconds() * (this->wheel_diameter_ / 2) * w_right;

	if (std::isnan(d1)) {
		d1 = 0;
	}
	if (std::isnan(d2)) {
		d2 = 0;
	}

	dr = (d1 + d2) / 2;
	da = (d2 - d1) / this->wheel_separation_;

	// compute odometric pose
	this->odom_pose_[0] += dr * cos(this->odom_pose_[2]);
	this->odom_pose_[1] += dr * sin(this->odom_pose_[2]);
	this->odom_pose_[2] += da;

	// compute odometric instantaneouse velocity
	this->odom_vel_[0] = dr / step_time.seconds();
	this->odom_vel_[1] = 0.0;
	this->odom_vel_[2] = da / step_time.seconds();

	this->odom_.pose.pose.position.x  = this->odom_pose_[0];
	this->odom_.pose.pose.position.y  = this->odom_pose_[1];
	this->odom_.pose.pose.position.z  = 0;
	this->odom_.pose.pose.orientation = createQuaternionMsgFromYaw(this->odom_pose_[2]);

	// We should update the twist of the odometry
	this->odom_.twist.twist.linear.x  = this->odom_vel_[0];
	this->odom_.twist.twist.angular.z = this->odom_vel_[2];
}

void FakeKobuki::updateTF(geometry_msgs::msg::TransformStamped& odom_tf)
{
	odom_tf.header                  = this->odom_.header;
	odom_tf.child_frame_id          = this->odom_.child_frame_id;
	odom_tf.transform.translation.x = this->odom_.pose.pose.position.x;
	odom_tf.transform.translation.y = this->odom_.pose.pose.position.y;
	odom_tf.transform.translation.z = this->odom_.pose.pose.position.z;
	odom_tf.transform.rotation      = this->odom_.pose.pose.orientation;
}

void FakeKobuki::update()
{
	rclcpp::Time     time_now  = this->now();
	rclcpp::Duration step_time = time_now - this->prev_update_time_;
	this->prev_update_time_    = time_now;

	// zero-ing after timeout
	if (((time_now - this->last_cmd_vel_time_).seconds() > this->cmd_vel_timeout_) ||
	    !this->motor_enabled_) {
		this->wheel_speed_cmd_[LEFT]  = 0.0;
		this->wheel_speed_cmd_[RIGHT] = 0.0;
	}

	// joint_states
	double w_left, w_right;
	updateJoint(LEFT, w_left, step_time);
	updateJoint(RIGHT, w_right, step_time);
	this->joint_states_.header.stamp = time_now;
	this->js_pub_->publish(this->joint_states_);

	// odom
	updateOdometry(w_left, w_right, step_time);
	this->odom_.header.stamp = time_now;
	this->odom_pub_->publish(this->odom_);

	// tf
	geometry_msgs::msg::TransformStamped odom_tf;
	updateTF(odom_tf);
	this->tf_broadcaster_->sendTransform(odom_tf);
}
}  // namespace kobuki

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<kobuki::FakeKobuki>());
	rclcpp::shutdown();
	return 0;
}