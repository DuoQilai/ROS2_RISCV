cmake_minimum_required(VERSION 3.10)
project(speed_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(speed_interfaces REQUIRED)

add_executable(speed_server src/speed_server.cpp)
add_executable(speed_client src/speed_client.cpp)
ament_target_dependencies(speed_server geometry_msgs rclcpp speed_interfaces)
ament_target_dependencies(speed_client rclcpp speed_interfaces)
install(TARGETS speed_server speed_client DESTINATION lib/${PROJECT_NAME})
ament_package()
