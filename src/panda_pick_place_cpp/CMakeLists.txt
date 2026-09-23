cmake_minimum_required(VERSION 3.8)
project(panda_pick_place_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(panda_vision_msgs REQUIRED)

add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(pick_and_place
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  panda_vision_msgs
)

install(TARGETS pick_and_place
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
