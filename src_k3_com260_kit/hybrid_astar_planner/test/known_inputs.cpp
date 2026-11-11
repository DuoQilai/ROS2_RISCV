#include <cassert>
#include <iostream>
#include "hybrid_astar_planner/hybrid.hpp"

int main() {
  Costmap2D map(5, 5, 0.05);
  std::fill(map.data.begin(), map.data.end(), 0);
  const Pose goal{1, 1, 0};
  auto path = hybrid_plan(map, {-1, -1, 0}, goal);
  assert(!path.empty());
  assert(std::hypot(path.back()[0] - goal[0], path.back()[1] - goal[1]) < 0.5);
  assert(std::abs(wrapped_angle(path.back()[2])) <= 3.14159265358979323846 / 12);
  for (auto pose : path) { assert(footprint_clear(map, pose)); }
  assert(!footprint_clear(map, {2.4, 0, 0}));
  assert(hybrid_plan(map, {-1, -1, 0}, goal, 0).empty());
  assert(hybrid_plan(map, {NAN, 0, 0}, goal).empty());
  map.set_cost(goal[0], goal[1], 254);
  assert(hybrid_plan(map, {-1, -1, 0}, goal).empty());
  std::cout << "Hybrid-A* endpoint, heading, footprint and invalid-input checks passed\n";
}
