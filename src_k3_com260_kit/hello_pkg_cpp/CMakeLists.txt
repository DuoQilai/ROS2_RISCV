cmake_minimum_required(VERSION 3.10)
project(hello_pkg_cpp)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hello_node src/hello_node.cpp)
add_executable(logger_demo src/logger_demo.cpp)
add_executable(odom_monitor src/odom_monitor.cpp)
target_compile_features(hello_node PUBLIC cxx_std_17)
target_compile_features(logger_demo PUBLIC cxx_std_17)
target_compile_features(odom_monitor PUBLIC cxx_std_17)
ament_target_dependencies(hello_node rclcpp)
ament_target_dependencies(logger_demo rclcpp)
ament_target_dependencies(odom_monitor nav_msgs rclcpp)

install(TARGETS hello_node logger_demo odom_monitor DESTINATION lib/${PROJECT_NAME})
ament_package()
