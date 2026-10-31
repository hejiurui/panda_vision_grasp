cmake_minimum_required(VERSION 3.8)
project(panda_vision_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(panda_vision_msgs REQUIRED)

add_executable(color_detector src/color_detector.cpp)
ament_target_dependencies(color_detector
  rclcpp
  sensor_msgs
  cv_bridge
  tf2
  tf2_ros
  geometry_msgs
  std_msgs
  panda_vision_msgs
)
target_link_libraries(color_detector ${OpenCV_LIBS})

install(TARGETS color_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
