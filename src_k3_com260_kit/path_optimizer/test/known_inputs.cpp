#include <cassert>
#include <iostream>
#include "path_optimizer/optimize.hpp"

int main() {
  Costmap2D map(5, 5, 0.05);
  std::fill(map.data.begin(), map.data.end(), 0);
  const Path straight{{-1, 0}, {0, 0}, {1, 0}};
  assert(smooth_path(straight, map) == straight);
  assert(prune_path(straight, map).size() == 2);
  const auto sampled = resample_path({{0, 0}, {0, 0}, {0.15, 0}, {0.3, 0}}, 0.1);
  assert(sampled.size() == 4 && distance(sampled.back(), {0.3, 0}) < 1e-9);
  for (std::size_t i = 1; i < sampled.size(); ++i) { assert(std::abs(distance(sampled[i-1], sampled[i]) - 0.1) < 1e-9); }
  const Path curve{{-1, -1}, {-0.5, 0}, {0.5, 0}, {1, 1}};
  auto spline = bspline_path(curve, map);
  assert(spline.size() == 100 && spline.front() == curve.front() && spline.back() == curve.back());
  map.set_cost(0, 0, 254);
  const Path detour{{-1, 0}, {-1, 1}, {1, 1}, {1, 0}};
  assert(path_clear(map, smooth_path(detour, map)));
  assert(prune_path(detour, map, 10).size() > 2);
  assert(resample_path({}).empty());
  bool rejected = false;
  try { resample_path(straight, 0); } catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  std::cout << "Path endpoints, spacing, spline and obstacle checks passed\n";
}
