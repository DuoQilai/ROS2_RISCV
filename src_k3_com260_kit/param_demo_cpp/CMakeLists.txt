cmake_minimum_required(VERSION 3.8)
project(param_demo_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(param_demo src/param_demo.cpp)
target_compile_features(param_demo PRIVATE cxx_std_17)
ament_target_dependencies(param_demo rclcpp rcl_interfaces geometry_msgs)

add_executable(param_node src/param_node.cpp)
target_compile_features(param_node PRIVATE cxx_std_17)
ament_target_dependencies(param_node rclcpp rcl_interfaces geometry_msgs)

add_executable(speed_ctrl src/speed_controller.cpp)
target_compile_features(speed_ctrl PRIVATE cxx_std_17)
ament_target_dependencies(speed_ctrl rclcpp rcl_interfaces geometry_msgs)

add_executable(patrol_driver src/patrol_driver.cpp)
target_compile_features(patrol_driver PRIVATE cxx_std_17)
ament_target_dependencies(patrol_driver rclcpp rcl_interfaces geometry_msgs)

install(TARGETS param_demo param_node speed_ctrl patrol_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config maps rviz DESTINATION share/${PROJECT_NAME})

ament_package()
