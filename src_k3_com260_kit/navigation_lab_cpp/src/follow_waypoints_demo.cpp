#include "navigation_lab_cpp/navigator.hpp"

int main(int argc, char ** argv) {
  init_navigation(argc, argv);
  auto nav = std::make_shared<Navigator>();
  const bool passed = nav->ready() && nav->follow_waypoints();
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
