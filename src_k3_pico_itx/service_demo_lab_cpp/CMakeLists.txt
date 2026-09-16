cmake_minimum_required(VERSION 3.10)
project(service_demo_lab_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(server src/server.cpp)
add_executable(client src/client.cpp)
add_executable(client_wait src/client_wait.cpp)
ament_target_dependencies(server example_interfaces rclcpp)
ament_target_dependencies(client example_interfaces rclcpp)
ament_target_dependencies(client_wait example_interfaces rclcpp)

install(TARGETS server client client_wait DESTINATION lib/${PROJECT_NAME})
ament_package()
