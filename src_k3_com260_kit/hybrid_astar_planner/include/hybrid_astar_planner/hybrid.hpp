#pragma once
#include <array>
#include <grid_path_planner/search.hpp>

using Pose = std::array<double, 3>;
inline double wrapped_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

inline bool footprint_clear(const Costmap2D & map, Pose pose, double radius = 0.22) {
  if (pose[0] - radius < map.origin_x || pose[1] - radius < map.origin_y ||
      pose[0] + radius >= map.origin_x + map.width * map.resolution ||
      pose[1] + radius >= map.origin_y + map.height * map.resolution) { return false; }
  auto [x, y] = map.world_to_grid(pose[0], pose[1]);
  const int cells = static_cast<int>(std::ceil(radius / map.resolution)) + 1;
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      const auto center = map.grid_to_world(x + dx, y + dy);
      const double gap_x = std::max(0.0, std::abs(center.first - pose[0]) - map.resolution / 2);
      const double gap_y = std::max(0.0, std::abs(center.second - pose[1]) - map.resolution / 2);
      if (std::hypot(gap_x, gap_y) <= radius &&
          (!map.inside(x + dx, y + dy) || map.data[(y + dy) * map.width + x + dx] >= 254)) {
        return false;
      }
    }
  }
  return true;
}

inline std::vector<Pose> hybrid_plan(const Costmap2D & map, Pose start, Pose goal,
                                    double turning_radius = 0.5, double reverse_penalty = 2.0) {
  for (double value : {start[0], start[1], start[2], goal[0], goal[1], goal[2],
      turning_radius, reverse_penalty}) {
    if (!std::isfinite(value)) { return {}; }
  }
  if (turning_radius <= 0 || reverse_penalty < 1 ||
      !footprint_clear(map, start) || !footprint_clear(map, goal)) { return {}; }
  constexpr double pi = 3.14159265358979323846;
  auto key = [&](Pose pose) {
    auto [x, y] = map.world_to_grid(pose[0], pose[1]);
    int heading = static_cast<int>(std::floor((wrapped_angle(pose[2]) + pi) / (pi / 12))) % 24;
    return (y * map.width + x) * 24 + heading;
  };
  struct Node { Pose pose; double cost; int parent; };
  std::vector<Node> nodes{{start, 0, -1}};
  std::vector<double> best(map.width * map.height * 24, std::numeric_limits<double>::infinity());
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  best[key(start)] = 0;
  open.push({std::hypot(start[0] - goal[0], start[1] - goal[1]), 0});
  while (!open.empty()) {
    const int index = open.top().second;
    open.pop();
    const Node current = nodes[index];
    if (current.cost > best[key(current.pose)] + 1e-9) { continue; }
    if (std::hypot(current.pose[0] - goal[0], current.pose[1] - goal[1]) < 0.5 &&
        std::abs(wrapped_angle(current.pose[2] - goal[2])) <= pi / 12) {
      std::vector<Pose> path;
      for (int i = index; i != -1; i = nodes[i].parent) { path.push_back(nodes[i].pose); }
      std::reverse(path.begin(), path.end());
      return path;
    }
    for (int gear : {1, -1}) {
      for (int steer = -2; steer <= 2; ++steer) {
        if (gear < 0 && std::abs(steer) == 2) { continue; }
        const double length = gear > 0 ? 0.15 : -0.06;
        const double curvature = steer / (2.0 * turning_radius);
        const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(length) / (map.resolution / 2))));
        Pose next = current.pose;
        bool clear = true;
        for (int i = 0; i < steps && clear; ++i) {
          const double ds = length / steps;
          next[0] += ds * std::cos(next[2] + ds * curvature / 2);
          next[1] += ds * std::sin(next[2] + ds * curvature / 2);
          next[2] = wrapped_angle(next[2] + ds * curvature);
          clear = footprint_clear(map, next);
        }
        if (!clear) { continue; }
        const double cost = current.cost + std::abs(length) * (gear < 0 ? reverse_penalty : 1) +
          std::abs(curvature) * 0.05;
        const int cell = key(next);
        if (cost + 1e-9 >= best[cell]) { continue; }
        best[cell] = cost;
        nodes.push_back({next, cost, index});
        open.push({cost + std::hypot(next[0] - goal[0], next[1] - goal[1]),
          static_cast<int>(nodes.size() - 1)});
      }
    }
  }
  return {};
}
