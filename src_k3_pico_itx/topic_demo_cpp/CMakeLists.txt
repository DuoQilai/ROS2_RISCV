cmake_minimum_required(VERSION 3.10)
project(topic_demo_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(topic_demo_interfaces REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp topic_demo_interfaces)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs topic_demo_interfaces)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher geometry_msgs rclcpp)

add_executable(gps_subscriber src/gps_subscriber.cpp)
ament_target_dependencies(gps_subscriber geometry_msgs rclcpp)

add_executable(qos_publisher src/qos_publisher.cpp)
ament_target_dependencies(qos_publisher rclcpp std_msgs)

add_executable(square_driver src/square_driver.cpp)
ament_target_dependencies(square_driver geometry_msgs rclcpp)

add_executable(executor_single src/multi_topic_demo.cpp)
ament_target_dependencies(executor_single rclcpp std_msgs)
add_executable(executor_multi src/multi_topic_demo.cpp)
target_compile_definitions(executor_multi PRIVATE USE_MULTI_EXECUTOR)
ament_target_dependencies(executor_multi rclcpp std_msgs)

install(TARGETS
  talker
  listener
  publisher
  gps_subscriber
  qos_publisher
  square_driver
  executor_single
  executor_multi
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
