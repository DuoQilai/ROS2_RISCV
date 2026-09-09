cmake_minimum_required(VERSION 3.10)
project(lifecycle_demo_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(lifecycle_demo src/lifecycle_demo.cpp)
ament_target_dependencies(lifecycle_demo geometry_msgs lifecycle_msgs rclcpp rclcpp_lifecycle)

install(TARGETS lifecycle_demo DESTINATION lib/${PROJECT_NAME})
ament_package()
