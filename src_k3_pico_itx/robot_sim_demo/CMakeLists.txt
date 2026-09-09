cmake_minimum_required(VERSION 3.8)
project(robot_sim_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
target_compile_features(camera_info_publisher PUBLIC cxx_std_17)
ament_target_dependencies(camera_info_publisher rclcpp rosgraph_msgs sensor_msgs)
install(TARGETS camera_info_publisher DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/circle_driver scripts/patrol_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config gui rviz urdf worlds models DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*:Zone.Identifier" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_assets test/test_assets.py)
endif()

ament_package()
