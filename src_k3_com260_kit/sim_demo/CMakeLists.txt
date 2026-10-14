cmake_minimum_required(VERSION 3.8)
project(sim_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(sensor_reader src/sensor_reader.cpp)
target_compile_features(sensor_reader PRIVATE cxx_std_17)
ament_target_dependencies(sensor_reader rclcpp sensor_msgs)

install(TARGETS sensor_reader DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config gui models urdf worlds rviz DESTINATION share/${PROJECT_NAME})

ament_package()
