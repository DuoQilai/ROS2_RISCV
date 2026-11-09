#include <array>
#include "navigation_client_cpp/navigator.hpp"

int main(int argc, char ** argv) {
  init_navigation(argc, argv);
  auto node = std::make_shared<BTNavigatorClient>();
  // Five Museum waypoints; each successful visit is followed by a three-second pause.
  const std::array<std::array<double, 3>, 5> goals{{
    {1.0, 0.0, 0.0}, {1.0, -1.0, -1.57}, {2.0, -1.0, 0.0},
    {0.0, -1.0, 3.14}, {0.0, 0.0, 0.0}}};
  for (std::size_t i = 0; i < goals.size(); ++i) {
    const auto & goal = goals[i];
    if (!node->navigate(goal[0], goal[1], goal[2])) { rclcpp::shutdown(); return 1; }
    RCLCPP_INFO(node->get_logger(), "到达目标 %zu/5，暂停 3 秒", i + 1);
    rclcpp::sleep_for(3s);
  }
  rclcpp::shutdown();
}
