#include "navigation_client_cpp/navigator.hpp"

int main(int argc, char ** argv) {
  init_navigation(argc, argv);
  auto node = std::make_shared<BTNavigatorClient>();
  const double x = node->declare_parameter("x", 3.0);
  const double y = node->declare_parameter("y", 2.0);
  const double yaw = node->declare_parameter("yaw", 0.0);
  const auto tree = node->declare_parameter<std::string>("behavior_tree", "");
  const double cancel = node->declare_parameter("cancel_after", 0.0);
  const bool success = node->navigate(x, y, yaw, tree, cancel);
  rclcpp::shutdown();
  return success ? 0 : 1;
}
