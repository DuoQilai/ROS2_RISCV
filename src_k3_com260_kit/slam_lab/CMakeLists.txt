cmake_minimum_required(VERSION 3.8)
project(slam_lab)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(slam_monitor src/slam_monitor.cpp)
target_compile_features(slam_monitor PRIVATE cxx_std_17)
ament_target_dependencies(slam_monitor rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cartographer_ros_msgs)

add_executable(set_initial_pose src/initial_pose_setter.cpp)
target_compile_features(set_initial_pose PRIVATE cxx_std_17)
ament_target_dependencies(set_initial_pose rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cartographer_ros_msgs)

add_executable(amcl_evaluator src/amcl_evaluator.cpp)
target_compile_features(amcl_evaluator PRIVATE cxx_std_17)
ament_target_dependencies(amcl_evaluator rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cartographer_ros_msgs)

add_executable(save_cartographer_state src/cartographer_state_saver.cpp)
target_compile_features(save_cartographer_state PRIVATE cxx_std_17)
ament_target_dependencies(save_cartographer_state rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs cartographer_ros_msgs)

install(TARGETS slam_monitor set_initial_pose amcl_evaluator save_cartographer_state DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# The board runs clients; the x86 backend image also installs Cartographer Lua resources.
if(EXISTS "/opt/ros/humble/share/cartographer/configuration_files")
  install(DIRECTORY /opt/ros/humble/share/cartographer/configuration_files/
    DESTINATION share/${PROJECT_NAME}/config/cartographer FILES_MATCHING PATTERN "*.lua")
endif()

ament_package()
