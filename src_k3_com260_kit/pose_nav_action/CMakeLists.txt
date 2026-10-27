cmake_minimum_required(VERSION 3.8)
project(pose_nav_action)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pose_nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(nav_server src/nav_server.cpp)
target_compile_features(nav_server PRIVATE cxx_std_17)
ament_target_dependencies(nav_server rclcpp rclcpp_action pose_nav_interfaces geometry_msgs nav_msgs)

add_executable(nav_client src/nav_client.cpp)
target_compile_features(nav_client PRIVATE cxx_std_17)
ament_target_dependencies(nav_client rclcpp rclcpp_action pose_nav_interfaces geometry_msgs nav_msgs)

install(TARGETS nav_server nav_client DESTINATION lib/${PROJECT_NAME})

ament_package()
