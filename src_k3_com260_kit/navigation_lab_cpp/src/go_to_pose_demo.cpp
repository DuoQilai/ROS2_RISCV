#include <array>
#include "navigation_lab_cpp/navigator.hpp"

int main(int argc, char ** argv) {
  init_navigation(argc, argv);
  auto nav = std::make_shared<Navigator>();
  bool passed = nav->ready();
  const double pi = std::acos(-1.0);
  const std::array<std::array<double, 3>, 4> goals{{
    {3, 0, 0}, {3, 2, pi / 2}, {0, 2, pi}, {0, 0, -pi / 2}}};
  if (passed) {
    for (std::size_t i = 0; i < goals.size() && !stop_navigation; ++i) {
      RCLCPP_INFO(nav->get_logger(), "--- Goal %zu/4 ---", i + 1);
      const auto & p = goals[i];
      if (!nav->navigate(p[0], p[1], p[2])) { passed = false; }
    }
  }
  rclcpp::shutdown();
  return passed && !stop_navigation ? 0 : 1;
}
