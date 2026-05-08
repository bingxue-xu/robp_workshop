cmake_minimum_required(VERSION 3.5)
project(kobuki_softnode)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(fake_kobuki src/fake_kobuki.cpp)
ament_target_dependencies(fake_kobuki geometry_msgs kobuki_ros_interfaces nav_msgs rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS
	fake_kobuki
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()