cmake_minimum_required(VERSION 3.8)
project(icp_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(robp_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

# ICP main executable
add_executable(icp_odometry
  src/icp_odometry.cpp)
ament_target_dependencies(icp_odometry
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros pcl_conversions laser_geometry robp_interfaces tf2_geometry_msgs
)
target_include_directories(icp_odometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(icp_odometry ${PCL_LIBRARIES})

# Encoder IMU 
add_executable(encoder_imu_odometry_node
  src/encoder_imu_odometry.cpp
  src/encoder_imu_odometry_node.cpp)

ament_target_dependencies(encoder_imu_odometry_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros robp_interfaces tf2_geometry_msgs
)
target_include_directories(encoder_imu_odometry_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS icp_odometry encoder_imu_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)
add_compile_options(-Wall -Wextra -Wpedantic -g)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
