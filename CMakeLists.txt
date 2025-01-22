cmake_minimum_required(VERSION 3.5)
project(ros2_stretch_docking)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(image_proc REQUIRED)

# potentially replace with nav2_common, nav2_package()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  opennav_docking_core
  cv_bridge
  image_transport
  OpenCV
  tf2
  apriltag_ros
  apriltag_msgs
  image_proc
)

add_executable(dock_pose_publisher src/dock_pose_publisher.cpp)
ament_target_dependencies(dock_pose_publisher ${dependencies})

add_executable(rotate_camera_node src/rotate_camera_node.cpp)
ament_target_dependencies(rotate_camera_node ${dependencies})

add_executable(detected_dock_pose_publisher src/detected_dock_pose_publisher.cpp)
ament_target_dependencies(detected_dock_pose_publisher ${dependencies})

install(TARGETS
  dock_pose_publisher
  rotate_camera_node
  detected_dock_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
