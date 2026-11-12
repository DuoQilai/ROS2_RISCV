#include <cassert>
#include <iostream>
#include "rpp_controller/pursuit.hpp"
#include "rpp_controller/tuning.hpp"

int main() {
  const Path path{{0, 0}, {1, 0}, {2, 0}};
  auto straight = pure_pursuit({0, 0, 0}, path);
  assert(std::abs(straight.v - 0.3) < 1e-9 && straight.w == 0);
  auto arrived = pure_pursuit({2, 0, 0}, path);
  assert(arrived.v == 0 && arrived.w == 0);
  auto turning = pure_pursuit({0, 0, 0}, {{0, 0}, {1, 0.5}, {2, 1}});
  assert(turning.w > 0 && turning.v < straight.v);
  assert(std::abs(turning.w - turning.v * turning.curvature) < 1e-9);
  auto progressed = pure_pursuit({1.5, 0, 0}, {{-2, 0}, {0, 0}, {1.5, 0}, {2, 0}});
  assert(progressed.target.first == 2 && progressed.v > 0);
  assert(pure_pursuit({0, 0, 0}, {}).v == 0);
  assert(pure_pursuit({NAN, 0, 0}, path).v == 0);
  RPPConfig invalid;
  invalid.min_lookahead = invalid.max_lookahead + 1;
  assert(pure_pursuit({0, 0, 0}, path, nullptr, 0.3, invalid).v == 0);
  Costmap2D map(5, 5, 0.05);
  std::fill(map.data.begin(), map.data.end(), 0);
  map.set_cost(0.2, 0, 254);
  auto blocked = pure_pursuit({0, 0, 0}, path, &map);
  assert(blocked.v == 0 && blocked.w == 0);
  const auto narrow = recommend_speed("differential", "narrow");
  assert(std::abs(narrow.linear - 0.3) < 1e-9 && std::abs(narrow.acceleration - 1.75) < 1e-9);
  assert(std::abs(recommend_lookahead(0.3, 10, 0.22) - 0.67) < 1e-9);
  std::cout << "RPP straight, turn, arrival, path progress and collision checks passed\n";
}
