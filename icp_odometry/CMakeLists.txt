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

include_directories(${PCL_INCLUDE_DIRS})

add_executable(icp_odometry src/icp_odometry.cpp)
ament_target_dependencies(icp_odometry
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros pcl_conversions laser_geometry
)
target_link_libraries(icp_odometry ${PCL_LIBRARIES})
install(TARGETS icp_odometry
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
