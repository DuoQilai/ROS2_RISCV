#include <array>
#include <functional>
#include "recovery_demo_cpp/recovery.hpp"
int main(int argc, char ** argv) {
  init_actions(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("recovery_strategy");
  const std::array<std::function<bool()>, 6> steps = {{
    [&] { return spin(node, 1.57); }, [&] { return wait(node, 1); },
    [&] { return backup(node, 0.15); }, [&] { return spin(node, 3.14); },
    [&] { return backup(node, 0.3); }, [&] { return wait(node, 2); }
  }};
  bool ok = false;
  for (size_t i = 0; i < steps.size() && !stop_requested; ++i) {
    RCLCPP_INFO(node->get_logger(), "执行恢复步骤 %zu", i + 1);
    if (steps[i]()) { ok = true; break; }
  }
  RCLCPP_INFO(node->get_logger(), "%s", ok ? "恢复成功" : "恢复未成功");
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
