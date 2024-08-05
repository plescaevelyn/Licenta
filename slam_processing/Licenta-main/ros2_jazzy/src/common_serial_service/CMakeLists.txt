cmake_minimum_required(VERSION 3.5)
project(common_serial_service)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)

install(PROGRAMS
  src/serial_reader_service.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
