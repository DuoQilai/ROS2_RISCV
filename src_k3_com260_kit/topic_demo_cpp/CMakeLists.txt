cmake_minimum_required(VERSION 3.10)
project(topic_demo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(topic_demo_interfaces REQUIRED)

add_executable(talker src/talker.cpp)
target_compile_features(talker PUBLIC cxx_std_17)
ament_target_dependencies(talker rclcpp topic_demo_interfaces)

add_executable(listener src/listener.cpp)
target_compile_features(listener PUBLIC cxx_std_17)
ament_target_dependencies(listener rclcpp std_msgs topic_demo_interfaces)

add_executable(publisher src/publisher.cpp)
target_compile_features(publisher PUBLIC cxx_std_17)
ament_target_dependencies(publisher geometry_msgs rclcpp)

add_executable(gps_subscriber src/gps_subscriber.cpp)
target_compile_features(gps_subscriber PUBLIC cxx_std_17)
ament_target_dependencies(gps_subscriber geometry_msgs rclcpp)

add_executable(qos_publisher src/qos_publisher.cpp)
target_compile_features(qos_publisher PUBLIC cxx_std_17)
ament_target_dependencies(qos_publisher rclcpp std_msgs)

add_executable(square_driver src/square_driver.cpp)
target_compile_features(square_driver PUBLIC cxx_std_17)
ament_target_dependencies(square_driver geometry_msgs rclcpp)

add_executable(executor_single src/multi_topic_demo.cpp)
target_compile_features(executor_single PUBLIC cxx_std_17)
ament_target_dependencies(executor_single rclcpp std_msgs)
add_executable(executor_multi src/multi_topic_demo.cpp)
target_compile_features(executor_multi PUBLIC cxx_std_17)
target_compile_definitions(executor_multi PRIVATE USE_MULTI_EXECUTOR)
ament_target_dependencies(executor_multi rclcpp std_msgs)

add_executable(teacher_talker src/teacher_talker.cpp)
target_compile_features(teacher_talker PUBLIC cxx_std_17)
ament_target_dependencies(teacher_talker rclcpp std_msgs)

add_executable(teacher_listener src/teacher_listener.cpp)
target_compile_features(teacher_listener PUBLIC cxx_std_17)
ament_target_dependencies(teacher_listener rclcpp std_msgs)

add_executable(cmd_vel_subscriber src/cmd_vel_subscriber.cpp)
target_compile_features(cmd_vel_subscriber PUBLIC cxx_std_17)
ament_target_dependencies(cmd_vel_subscriber rclcpp geometry_msgs)

add_executable(scan_subscriber src/scan_subscriber.cpp)
target_compile_features(scan_subscriber PUBLIC cxx_std_17)
ament_target_dependencies(scan_subscriber rclcpp sensor_msgs)

install(TARGETS
  talker
  listener
  publisher
  gps_subscriber
  qos_publisher
  square_driver
  executor_single
  executor_multi
  teacher_talker
  teacher_listener
  cmd_vel_subscriber
  scan_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
