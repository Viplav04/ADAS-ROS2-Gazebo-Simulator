cmake_minimum_required(VERSION 3.5)

project(lane_assist)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


set (OpenCV_DIR = /home/rosteam/opencv/build)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lane_node 
src/lane_assist.cpp
src/main.cpp
)

ament_target_dependencies(lane_node
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "image_transport"
  "cv_bridge"
  "OpenCV"
)

install(TARGETS
  lane_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
