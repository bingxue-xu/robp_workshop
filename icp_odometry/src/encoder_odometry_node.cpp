#include "icp_odometry/encoder_odometry.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<icp_odometry::EncoderOdometry>());
    rclcpp::shutdown();
    return 0;
}
