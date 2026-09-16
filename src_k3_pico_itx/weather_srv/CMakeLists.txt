cmake_minimum_required(VERSION 3.10)
project(weather_srv)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(weather_interfaces REQUIRED)

add_executable(weather_server src/weather_server.cpp)
ament_target_dependencies(weather_server rclcpp weather_interfaces)
install(TARGETS weather_server DESTINATION lib/${PROJECT_NAME})
ament_package()
