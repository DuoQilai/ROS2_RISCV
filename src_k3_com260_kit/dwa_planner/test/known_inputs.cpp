#include <cassert>
#include <iostream>
#include "dwa_planner/dwa.hpp"
#include "dwa_planner/critics.hpp"

int main() {
  const DWAConfig config;
  const Motion state{0, 0, 0, 0, 0};
  const auto window = dynamic_window(state, config);
  assert(window[0] == 0 && window[1] == 0.2);
  assert(std::abs(window[2] + 0.3) < 1e-9 && std::abs(window[3] - 0.3) < 1e-9);
  auto result = dwa_plan(state, {2, 0}, {});
  assert(result.command.v > 0 && result.command.v <= 0.2);
  assert(std::abs(result.command.w) <= 0.3 + 1e-9);
  assert(result.command.w == 0);
  assert(result.trajectory.size() == 21);
  const Path sparse_path{{0, 0}, {0.5, 0}, {1, 0}, {2, 0}};
  const auto sparse = dwa_plan(state, {2, 0}, {}, sparse_path);
  assert(sparse.command.v > 0 && sparse.command.w == 0);
  assert(point_path_distance({0.25, 0}, sparse_path) == 0);
  assert(point_path_distance({0.25, 0.3}, sparse_path) == 0.3);
  assert(point_path_distance({1, 0}, {{0, 0}, {0, 0}}) == 1);
  auto blocked = dwa_plan(state, {2, 0}, {{0, 0}});
  assert(blocked.trajectory.empty() && blocked.command.v == 0 && blocked.command.w == 0);
  assert(dwa_plan(state, {0, 0}, {}).command.v == 0);
  assert(dwa_plan({NAN, 0, 0, 0, 0}, {2, 0}, {}).trajectory.empty());
  assert(dwa_plan(state, {2, 0}, {{NAN, 0}}).trajectory.empty());
  auto detour = dwa_plan(state, {1.5, 0}, {{0.65, -0.15}, {0.65, 0}, {0.65, 0.15}},
    {{0, 0}, {0.2, -0.3}, {0.7, -0.5}, {1.2, -0.3}, {1.5, 0}});
  assert(detour.command.w < 0);
  DWAConfig slow;
  slow.max_speed = 0.15;
  auto approach = dwa_plan({1.447158, 0.087147, -1.000353, 0, 0}, {1.5, 0}, {}, {}, slow);
  assert(approach.command.v > 0 && approach.command.v < 0.05);
  TrajectoryScore scores;
  auto near = predict_trajectory({0, 0, 0, 0.5, 0}, {0.5, 0}, config);
  assert(!std::isfinite(score_trajectory(near, {2, 0}, {{1.24, 0}}, {}, config, scores)));
  Costmap2D map(5, 5, 0.1);
  std::fill(map.data.begin(), map.data.end(), 0);
  assert(obstacle_cost(near, map) == 0);
  map.set_cost(0.2, 0, 254);
  assert(!std::isfinite(obstacle_cost(near, map)));
  assert(path_distance({state}, {{0, 1}}) == 1);
  assert(oscillation_cost({{1, 0}, {-1, 0}, {1, 0}, {-1, 0}, {1, 0}}) == 400);
  std::cout << "DWA acceleration bounds, endpoint samples, collision, braking and arrival checks passed\n";
}
