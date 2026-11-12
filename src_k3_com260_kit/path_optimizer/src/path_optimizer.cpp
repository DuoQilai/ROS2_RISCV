#include <iostream>
#include "path_optimizer/optimize.hpp"

int main() {
  Costmap2D map(5, 5, 0.05);
  std::fill(map.data.begin(), map.data.end(), 0);
  const Path path{{-1, -1}, {-0.5, -0.2}, {0, 0}, {0.5, 0.8}, {1, 1}};
  const auto smooth = smooth_path(path, map);
  const auto spline = bspline_path(path, map);
  const auto pruned = prune_path(path, map);
  const auto sampled = resample_path(pruned);
  const auto safe = optimize_cost(path, map);
  std::cout << "original=" << path.size() << " smooth=" << smooth.size()
    << " spline=" << spline.size() << " pruned=" << pruned.size()
    << " resampled=" << sampled.size() << " cost_optimized=" << safe.size() << '\n';
  for (auto [x, y] : smooth) { std::cout << x << ',' << y << '\n'; }
}
