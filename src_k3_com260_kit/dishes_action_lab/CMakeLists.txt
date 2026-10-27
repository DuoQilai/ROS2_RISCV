cmake_minimum_required(VERSION 3.8)
project(dishes_action_lab)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dishes_action_interfaces REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(server src/server.cpp)
target_compile_features(server PRIVATE cxx_std_17)
ament_target_dependencies(server rclcpp rclcpp_action dishes_action_interfaces)

add_executable(client src/client.cpp)
target_compile_features(client PRIVATE cxx_std_17)
ament_target_dependencies(client rclcpp rclcpp_action dishes_action_interfaces)

install(TARGETS server client DESTINATION lib/${PROJECT_NAME})

ament_package()
