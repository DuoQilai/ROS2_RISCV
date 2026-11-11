#include <cassert>
#include <iostream>
#include "grid_path_planner/search.hpp"

int main() {
  Costmap2D map(0.5, 0.5, 0.1);
  std::fill(map.data.begin(), map.data.end(), 0);
  const auto start = map.grid_to_world(0, 0), goal = map.grid_to_world(4, 4);
  auto d = search_grid(map, start, goal, false), a = search_grid(map, start, goal, true);
  assert(std::abs(d.cost - 4 * std::sqrt(2.0)) < 1e-9);
  assert(std::abs(a.cost - d.cost) < 1e-9 && a.expanded <= d.expanded);
  assert(search_grid(map, start, start, true).path.size() == 1);
  map.data[1] = map.data[map.width] = 254;
  assert(!clear_segment(map, start, goal));
  assert(!clear_segment(map, {-0.249, -0.151}, {-0.051, -0.149}));
  assert(search_grid(map, start, goal, true).path.empty());
  map.data[0] = 255;
  assert(search_grid(map, start, goal, false).path.empty());
  assert(search_grid(map, {-1, 0}, goal, true).path.empty());
  assert(search_grid(map, {NAN, 0}, goal, true).path.empty());
  std::fill(map.data.begin(), map.data.end(), 0);
  map.data[2 * map.width + 2] = 200;
  a = search_grid(map, start, goal, true);
  d = search_grid(map, start, goal, false);
  assert(std::abs(a.cost - d.cost) < 1e-9);
  assert(a.cost < 8 && a.path.size() > 5);
  std::cout << "Grid shortest paths, corners, segment crossings and invalid inputs passed\n";
}
