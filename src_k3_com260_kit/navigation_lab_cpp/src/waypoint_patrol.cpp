#include "navigation_lab_cpp/navigator.hpp"

int main(int argc, char ** argv) {
  init_navigation(argc, argv);
  auto nav = std::make_shared<Navigator>();
  bool passed = nav->ready();
  for (int lap = 1; passed && lap <= 2 && !stop_navigation; ++lap) {
    RCLCPP_INFO(nav->get_logger(), "=== Patrol lap %d/2 ===", lap);
    passed = nav->follow_waypoints();
    RCLCPP_INFO(nav->get_logger(), "Lap %d %s", lap, passed ? "complete" : "interrupted");
  }
  rclcpp::shutdown();
  return passed && !stop_navigation ? 0 : 1;
}
