#pragma once
#include <grid_path_planner/search.hpp>

inline double distance(Point a, Point b) { return std::hypot(a.first - b.first, a.second - b.second); }
inline Point blend(Point a, Point b, double t) {
  return {a.first + t * (b.first - a.first), a.second + t * (b.second - a.second)};
}
inline bool path_clear(const Costmap2D & map, const Path & path) {
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!clear_segment(map, path[i - 1], path[i])) { return false; }
  }
  return !path.empty() && inside_map(map, path[0]) && map.get_cost(path[0].first, path[0].second) < 253;
}

inline Path smooth_path(const Path & path, const Costmap2D & map, int iterations = 100) {
  auto result = path;
  for (int n = 0; n < iterations; ++n) {
    for (std::size_t i = 1; i + 1 < result.size(); ++i) {
      Point candidate{
        result[i].first + 0.2 * (0.5 * (path[i].first - result[i].first) +
          (0.3 + 0.2) * (result[i - 1].first + result[i + 1].first - 2 * result[i].first)),
        result[i].second + 0.2 * (0.5 * (path[i].second - result[i].second) +
          (0.3 + 0.2) * (result[i - 1].second + result[i + 1].second - 2 * result[i].second))};
      if (clear_segment(map, result[i - 1], candidate) && clear_segment(map, candidate, result[i + 1])) {
        result[i] = candidate;
      }
    }
  }
  return result;
}

inline Path bspline_path(const Path & path, const Costmap2D & map, int samples = 100) {
  if (path.size() < 4 || samples < 2) { return path; }
  const int count = static_cast<int>(path.size());
  std::vector<double> knots(count + 4);
  for (int i = 0; i < count + 4; ++i) {
    knots[i] = std::clamp((i - 3.0) / (count - 3), 0.0, 1.0);
  }
  Path result;
  for (int sample = 0; sample < samples; ++sample) {
    const double t = static_cast<double>(sample) / (samples - 1);
    const int span = t == 1 ? count - 1 : static_cast<int>(
      std::upper_bound(knots.begin(), knots.end(), t) - knots.begin()) - 1;
    Point points[4];
    for (int j = 0; j <= 3; ++j) { points[j] = path[span - 3 + j]; }
    for (int level = 1; level <= 3; ++level) {
      for (int j = 3; j >= level; --j) {
        const int i = span - 3 + j;
        const double alpha = (t - knots[i]) / (knots[i + 4 - level] - knots[i]);
        points[j] = blend(points[j - 1], points[j], alpha);
      }
    }
    result.push_back(points[3]);
  }
  return path_clear(map, result) ? result : path;
}

inline Path prune_path(const Path & path, const Costmap2D & map, double epsilon = 0.1) {
  if (path.size() < 3) { return path; }
  const Point first = path.front(), last = path.back();
  const double squared = std::pow(distance(first, last), 2);
  double farthest = -1;
  std::size_t split = 1;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const double t = squared == 0 ? 0 : std::clamp(
      ((path[i].first - first.first) * (last.first - first.first) +
       (path[i].second - first.second) * (last.second - first.second)) / squared, 0.0, 1.0);
    const double error = distance(path[i], blend(first, last, t));
    if (error > farthest) { farthest = error; split = i; }
  }
  if (farthest <= epsilon && clear_segment(map, first, last)) { return {first, last}; }
  auto left = prune_path(Path(path.begin(), path.begin() + split + 1), map, epsilon);
  auto right = prune_path(Path(path.begin() + split, path.end()), map, epsilon);
  left.insert(left.end(), right.begin() + 1, right.end());
  return left;
}

inline Path resample_path(const Path & path, double step = 0.1) {
  if (!std::isfinite(step) || step <= 0) { throw std::invalid_argument("step must be positive"); }
  if (path.size() < 2) { return path; }
  Path result{path.front()};
  double remaining = step;
  for (std::size_t i = 1; i < path.size(); ++i) {
    Point from = path[i - 1];
    double length = distance(from, path[i]);
    while (length >= remaining && length > 0) {
      from = blend(from, path[i], remaining / length);
      result.push_back(from);
      length -= remaining;
      remaining = step;
    }
    remaining -= length;
  }
  if (distance(result.back(), path.back()) > 1e-9) { result.push_back(path.back()); }
  return result;
}

inline Path optimize_cost(const Path & path, const Costmap2D & map, double max_shift = 0.3) {
  auto result = path;
  for (int n = 0; n < 50; ++n) {
    for (std::size_t i = 1; i + 1 < result.size(); ++i) {
      const double dx = result[i + 1].first - result[i - 1].first;
      const double dy = result[i + 1].second - result[i - 1].second;
      const double length = std::hypot(dx, dy);
      if (length == 0 || map.get_cost(result[i].first, result[i].second) <= 50) { continue; }
      for (double sign : {-1.0, 1.0}) {
        Point next{result[i].first - sign * dy / length * map.resolution,
          result[i].second + sign * dx / length * map.resolution};
        if (distance(next, path[i]) <= max_shift &&
            map.get_cost(next.first, next.second) < map.get_cost(result[i].first, result[i].second) &&
            clear_segment(map, result[i - 1], next) && clear_segment(map, next, result[i + 1])) {
          result[i] = next;
        }
      }
    }
  }
  return result;
}
