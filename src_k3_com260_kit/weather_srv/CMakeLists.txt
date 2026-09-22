cmake_minimum_required(VERSION 3.10)
project(weather_srv)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(weather_interfaces REQUIRED)

add_executable(weather_server src/weather_server.cpp)
target_compile_features(weather_server PUBLIC cxx_std_17)
ament_target_dependencies(weather_server rclcpp weather_interfaces)
install(TARGETS weather_server DESTINATION lib/${PROJECT_NAME})
ament_package()
