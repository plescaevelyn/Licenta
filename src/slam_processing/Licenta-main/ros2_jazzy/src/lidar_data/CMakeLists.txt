cmake_minimum_required(VERSION 3.8)
project(lidar_data)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

install(PROGRAMS
  src/lidar_data_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
