#pragma once
#include <array>
#include <grid_path_planner/search.hpp>

struct RPPConfig {
  double desired_speed = 0.3, max_speed = 0.5, min_speed = 0.05, max_yaw_rate = 1;
  double lookahead = 1, min_lookahead = 0.3, max_lookahead = 2;
  double max_curvature = 0.5, curvature_weight = 0.3, cost_gain = 1;
};
struct RPPCommand { double v = 0, w = 0, curvature = 0; Point target{}; };

inline RPPCommand pure_pursuit(std::array<double, 3> pose, const Path & path,
                               const Costmap2D * map = nullptr,
                               double current_speed = 0.3, const RPPConfig & c = {}) {
  RPPCommand command;
  for (double value : {pose[0], pose[1], pose[2], current_speed, c.desired_speed, c.max_speed,
      c.min_speed, c.max_yaw_rate, c.lookahead, c.min_lookahead, c.max_lookahead,
      c.max_curvature, c.curvature_weight, c.cost_gain}) {
    if (!std::isfinite(value)) { return command; }
  }
  for (auto [x, y] : path) { if (!std::isfinite(x) || !std::isfinite(y)) { return command; } }
  if (path.size() < 2 || c.desired_speed <= 0 || c.max_speed <= 0 || c.max_yaw_rate <= 0 ||
      c.min_speed < 0 || c.min_speed > c.max_speed || c.min_lookahead <= 0 ||
      c.max_lookahead < c.min_lookahead || c.max_curvature <= 0) { return command; }
  if (map && !inside_map(*map, {pose[0], pose[1]})) { return command; }
  const auto goal = path.back();
  if (std::hypot(goal.first - pose[0], goal.second - pose[1]) <= 0.1) { return command; }
  if (map && map->get_cost(pose[0], pose[1]) >= 253) { return command; }
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    double distance = std::hypot(path[i].first - pose[0], path[i].second - pose[1]);
    if (distance < best) { best = distance; nearest = i; }
  }
  const double lookahead = std::clamp(c.lookahead + std::abs(current_speed) * 0.5,
    c.min_lookahead, c.max_lookahead);
  command.target = goal;
  for (std::size_t i = nearest; i < path.size(); ++i) {
    if (std::hypot(path[i].first - pose[0], path[i].second - pose[1]) >= lookahead) {
      command.target = path[i]; break;
    }
  }
  const double dx = command.target.first - pose[0], dy = command.target.second - pose[1];
  const double angle = std::atan2(std::sin(std::atan2(dy, dx) - pose[2]),
    std::cos(std::atan2(dy, dx) - pose[2]));
  if (std::abs(angle) > std::acos(-1) / 4) {
    command.w = std::clamp(angle, -c.max_yaw_rate, c.max_yaw_rate);
    return command;
  }
  command.curvature = std::clamp(2 * std::sin(angle) / std::hypot(dx, dy),
    -c.max_curvature, c.max_curvature);
  double speed = c.desired_speed * std::max(0.3, 1 - std::abs(command.curvature) * c.curvature_weight);
  if (map) { speed *= std::max(0.1, 1 - map->get_cost(pose[0], pose[1]) / 254.0 * c.cost_gain); }
  command.v = std::clamp(speed, c.min_speed, c.max_speed);
  if (command.curvature != 0) { command.v = std::min(command.v, c.max_yaw_rate / std::abs(command.curvature)); }
  command.w = command.v * command.curvature;
  if (map) {
    auto predicted = pose;
    Point previous{pose[0], pose[1]};
    for (int i = 0; i < 10; ++i) {
      predicted[0] += command.v * std::cos(predicted[2]) * 0.1;
      predicted[1] += command.v * std::sin(predicted[2]) * 0.1;
      predicted[2] += command.w * 0.1;
      Point next{predicted[0], predicted[1]};
      if (!clear_segment(*map, previous, next)) { return {}; }
      previous = next;
    }
  }
  return command;
}
