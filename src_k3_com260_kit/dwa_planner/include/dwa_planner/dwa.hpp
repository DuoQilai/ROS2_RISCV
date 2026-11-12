#pragma once
#include <array>
#include <grid_path_planner/search.hpp>

struct Motion { double x, y, yaw, v, w; };
struct Velocity { double v = 0, w = 0; };
struct DWAConfig {
  double max_speed = 0.5, min_speed = 0, max_yaw_rate = 1;
  double max_accel = 2, max_yaw_accel = 3;
  double dt = 0.1, predict_time = 2, v_step = 0.05, w_step = 0.1;
  double radius = 0.22;
};
struct TrajectoryScore { double heading = 0, obstacle = 0, velocity = 0, path = 0; };
struct DWAResult {
  Velocity command;
  std::vector<Motion> trajectory;
  double score = -std::numeric_limits<double>::infinity();
};

inline std::array<double, 4> dynamic_window(Motion state, const DWAConfig & c) {
  return {std::max(c.min_speed, state.v - c.max_accel * c.dt),
    std::min(c.max_speed, state.v + c.max_accel * c.dt),
    std::max(-c.max_yaw_rate, state.w - c.max_yaw_accel * c.dt),
    std::min(c.max_yaw_rate, state.w + c.max_yaw_accel * c.dt)};
}

inline std::vector<Motion> predict_trajectory(Motion state, Velocity command, const DWAConfig & c) {
  std::vector<Motion> trajectory{state};
  for (int step = 0; step < static_cast<int>(std::ceil(c.predict_time / c.dt)); ++step) {
    state.x += command.v * std::cos(state.yaw) * c.dt;
    state.y += command.v * std::sin(state.yaw) * c.dt;
    state.yaw += command.w * c.dt;
    state.v = command.v; state.w = command.w;
    trajectory.push_back(state);
  }
  return trajectory;
}

inline double point_path_distance(Point point, const Path & path) {
  double nearest = std::hypot(point.first - path.front().first, point.second - path.front().second);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto [ax, ay] = path[i - 1];
    const double dx = path[i].first - ax, dy = path[i].second - ay;
    const double squared = dx * dx + dy * dy;
    const double t = squared == 0 ? 0 : std::clamp(
      ((point.first - ax) * dx + (point.second - ay) * dy) / squared, 0.0, 1.0);
    nearest = std::min(nearest, std::hypot(point.first - ax - t * dx, point.second - ay - t * dy));
  }
  return nearest;
}

inline double score_trajectory(const std::vector<Motion> & trajectory, Point goal,
                              const Path & obstacles, const Path & path,
                              const DWAConfig & c, TrajectoryScore & scores) {
  const auto end = trajectory.back();
  const double angle = end.yaw - std::atan2(goal.second - end.y, goal.first - end.x);
  scores.heading = 1 - std::abs(std::atan2(std::sin(angle), std::cos(angle))) / std::acos(-1);
  double clearance = std::numeric_limits<double>::infinity();
  scores.path = 0;
  for (auto state : trajectory) {
    for (auto [x, y] : obstacles) { clearance = std::min(clearance, std::hypot(state.x - x, state.y - y) - c.radius); }
    if (!path.empty()) {
      scores.path += 1 / (1 + point_path_distance({state.x, state.y}, path));
    }
  }
  const double braking = end.v * end.v / (2 * c.max_accel);
  if (clearance <= braking) { return -std::numeric_limits<double>::infinity(); }
  scores.obstacle = std::min(1.0, clearance / 0.5);
  scores.velocity = end.v / c.max_speed;
  scores.path /= trajectory.size();
  return scores.heading + 0.5 * scores.obstacle + 0.3 * scores.velocity + 2 * scores.path;
}

inline DWAResult dwa_plan(Motion state, Point goal, const Path & obstacles,
                          const Path & path = {}, const DWAConfig & c = {}) {
  DWAResult best;
  for (double value : {state.x, state.y, state.yaw, state.v, state.w, goal.first, goal.second,
      c.max_speed, c.min_speed, c.max_yaw_rate, c.max_accel, c.max_yaw_accel,
      c.dt, c.predict_time, c.v_step, c.w_step, c.radius}) {
    if (!std::isfinite(value)) { return best; }
  }
  for (const auto * points : {&obstacles, &path}) {
    for (auto [x, y] : *points) { if (!std::isfinite(x) || !std::isfinite(y)) { return best; } }
  }
  if (c.max_speed <= 0 || c.max_accel <= 0 || c.dt <= 0 || c.predict_time <= 0 ||
      c.v_step <= 0 || c.w_step <= 0 || c.radius <= 0 || c.max_yaw_rate <= 0 ||
      c.max_yaw_accel <= 0 || c.min_speed < 0 || c.min_speed > c.max_speed) { return best; }
  const double remaining = std::hypot(state.x - goal.first, state.y - goal.second);
  if (remaining <= 0.1) { return best; }
  if (!path.empty()) {
    auto nearest = std::min_element(path.begin(), path.end(), [&](Point a, Point b) {
      return std::hypot(a.first - state.x, a.second - state.y) <
        std::hypot(b.first - state.x, b.second - state.y);
    });
    double ahead = 0;
    goal = *nearest;
    for (auto next = nearest + 1; next != path.end() && ahead < 0.6; ++next) {
      ahead += std::hypot(next->first - goal.first, next->second - goal.second);
      goal = *next;
    }
  }
  auto window = dynamic_window(state, c);
  window[1] = std::min(window[1], remaining / (2 * c.predict_time));
  if (window[1] < window[0] || window[3] < window[2]) { return best; }
  const int nv = std::max(1, static_cast<int>(std::ceil((window[1] - window[0]) / c.v_step)));
  const int nw = std::max(1, static_cast<int>(std::ceil((window[3] - window[2]) / c.w_step)));
  for (int i = 0; i <= nv; ++i) {
    for (int j = -1; j <= nw; ++j) {
      const double yaw_rate = j < 0 ? 0 : window[2] + (window[3] - window[2]) * j / nw;
      if (yaw_rate < window[2] || yaw_rate > window[3]) { continue; }
      Velocity command{window[0] + (window[1] - window[0]) * i / nv,
        yaw_rate};
      const auto trajectory = predict_trajectory(state, command, c);
      TrajectoryScore scores;
      const double score = score_trajectory(trajectory, goal, obstacles, path, c, scores);
      if (score > best.score) { best = {command, trajectory, score}; }
    }
  }
  return best;
}
