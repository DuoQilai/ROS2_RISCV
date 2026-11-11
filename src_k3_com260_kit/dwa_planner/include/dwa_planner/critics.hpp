#pragma once
#include "dwa_planner/dwa.hpp"

// Small scoring examples; Nav2 DWB loads its own critic plugins.
inline double obstacle_cost(const std::vector<Motion> & trajectory, const Costmap2D & map) {
  double total = 0;
  if (trajectory.empty()) { return std::numeric_limits<double>::infinity(); }
  for (auto state : trajectory) {
    if (!inside_map(map, {state.x, state.y}) || map.get_cost(state.x, state.y) >= 253) {
      return std::numeric_limits<double>::infinity();
    }
    total += map.get_cost(state.x, state.y);
  }
  return total / trajectory.size();
}

inline double path_distance(const std::vector<Motion> & trajectory, const Path & path) {
  if (trajectory.empty() || path.empty()) { return std::numeric_limits<double>::infinity(); }
  double total = 0;
  for (auto state : trajectory) {
    total += point_path_distance({state.x, state.y}, path);
  }
  return total / trajectory.size();
}

inline double oscillation_cost(const std::vector<Velocity> & history) {
  if (history.size() < 5) { return 0; }
  double cost = 0;
  for (std::size_t i = history.size() - 4; i < history.size(); ++i) {
    if (history[i - 1].v * history[i].v < 0) { cost += 100; }
  }
  return cost;
}
