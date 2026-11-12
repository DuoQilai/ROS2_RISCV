#include <iostream>
#include "hybrid_astar_planner/hybrid.hpp"

int main() {
  Costmap2D map(5, 5, 0.05);
  std::fill(map.data.begin(), map.data.end(), 0);
  const auto path = hybrid_plan(map, {-1, -1, 0}, {1, 1, 0});
  std::cout << "Hybrid-A* points=" << path.size() << '\n';
  if (path.empty()) { return 1; }
  for (const auto & pose : path) {
    std::cout << pose[0] << ',' << pose[1] << ',' << pose[2] << '\n';
  }
}
