cmake_minimum_required(VERSION 3.8)
project(tracking_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tracking_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(server src/server.cpp)
target_compile_features(server PRIVATE cxx_std_17)
ament_target_dependencies(server rclcpp rclcpp_action tracking_interfaces geometry_msgs nav_msgs)

add_executable(client src/client.cpp)
target_compile_features(client PRIVATE cxx_std_17)
ament_target_dependencies(client rclcpp rclcpp_action tracking_interfaces geometry_msgs nav_msgs)

add_executable(server_gazebo src/server_gazebo.cpp)
target_compile_features(server_gazebo PRIVATE cxx_std_17)
ament_target_dependencies(server_gazebo rclcpp rclcpp_action tracking_interfaces geometry_msgs nav_msgs)

install(TARGETS server client server_gazebo DESTINATION lib/${PROJECT_NAME})

ament_package()
