cmake_minimum_required(VERSION 3.8)
project(ros2_nav2_docking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(opennav_docking REQUIRED)

install(
  DIRECTORY config launch src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()