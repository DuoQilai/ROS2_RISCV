#include <chrono>
#include <fstream>
#include <iostream>
#include "grid_path_planner/search.hpp"

int main() {
  Costmap2D map(5, 5, 0.1);
  std::fill(map.data.begin(), map.data.end(), 0);
  for (int y = 5; y < 40; ++y) { map.data[y * map.width + 25] = 254; }
  for (bool astar : {false, true}) {
    const auto begin = std::chrono::steady_clock::now();
    const auto result = search_grid(map, {-2, -2}, {2, 2}, astar);
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
    std::cout << (astar ? "A*" : "Dijkstra") << " points=" << result.path.size()
      << " cost=" << result.cost << " expanded=" << result.expanded << " ms=" << ms << '\n';
    if (result.path.empty()) { return 1; }
    std::ofstream svg(astar ? "astar.svg" : "dijkstra.svg");
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 500 500\">"
      << "<rect width=\"500\" height=\"500\" fill=\"white\"/>"
      << "<rect x=\"250\" y=\"100\" width=\"10\" height=\"350\" fill=\"black\"/>"
      << "<polyline fill=\"none\" stroke=\"blue\" stroke-width=\"3\" points=\"";
    for (auto [x, y] : result.path) { svg << 100 * (x + 2.5) << ',' << 100 * (2.5 - y) << ' '; }
    svg << "\"/></svg>\n";
  }
}
