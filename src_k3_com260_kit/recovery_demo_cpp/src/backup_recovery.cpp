#include "recovery_demo_cpp/recovery.hpp"
int main(int argc, char ** argv) {
  init_actions(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("backup_recovery");
  const bool ok = backup(node, node->declare_parameter("distance", 0.15),
    node->declare_parameter("speed", 0.05));
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
