cmake_minimum_required(VERSION 3.5)

project(laser_obstacle_avoidance)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(obstacle_node 
src/obstacle_avoidance.cpp
src/main.cpp
)

ament_target_dependencies(obstacle_node
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS
  obstacle_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

