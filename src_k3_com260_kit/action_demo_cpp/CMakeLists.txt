cmake_minimum_required(VERSION 3.8)
project(action_demo_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_demo_interfaces REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(dishes_server src/dishes_server.cpp)
target_compile_features(dishes_server PRIVATE cxx_std_17)
ament_target_dependencies(dishes_server rclcpp rclcpp_action action_demo_interfaces)

add_executable(dishes_client src/dishes_client.cpp)
target_compile_features(dishes_client PRIVATE cxx_std_17)
ament_target_dependencies(dishes_client rclcpp rclcpp_action action_demo_interfaces)

install(TARGETS dishes_server dishes_client DESTINATION lib/${PROJECT_NAME})

ament_package()
