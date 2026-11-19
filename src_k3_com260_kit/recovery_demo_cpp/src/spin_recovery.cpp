#include "recovery_demo_cpp/recovery.hpp"
int main(int argc, char ** argv) {
  init_actions(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("spin_recovery");
  const bool ok = spin(node, node->declare_parameter("target_yaw", 1.57),
    node->declare_parameter("time_allowance", 5.0));
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
