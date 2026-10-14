cmake_minimum_required(VERSION 3.8)
project(slam_sim_demo_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(slam_map_runner src/slam_map_runner.cpp)
target_compile_features(slam_map_runner PRIVATE cxx_std_17)
ament_target_dependencies(slam_map_runner rclcpp geometry_msgs nav_msgs sensor_msgs)

install(TARGETS slam_map_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params rviz DESTINATION share/${PROJECT_NAME})

ament_package()
