#pragma once
#include <limits>
#include <queue>
#include <utility>
#include <costmap_demo/costmap.hpp>

using Point = std::pair<double, double>;
using Path = std::vector<Point>;

struct SearchResult {
  Path path;
  double cost = std::numeric_limits<double>::infinity();
  int expanded = 0;
};

inline bool inside_map(const Costmap2D & map, Point point) {
  return std::isfinite(point.first) && std::isfinite(point.second) &&
    point.first >= map.origin_x && point.first < map.origin_x + map.width * map.resolution &&
    point.second >= map.origin_y && point.second < map.origin_y + map.height * map.resolution;
}

inline bool clear_segment(const Costmap2D & map, Point a, Point b) {
  if (!inside_map(map, a) || !inside_map(map, b)) { return false; }
  auto [x, y] = map.world_to_grid(a.first, a.second);
  const auto [end_x, end_y] = map.world_to_grid(b.first, b.second);
  const double dx = b.first - a.first, dy = b.second - a.second;
  const int sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1;
  const double infinity = std::numeric_limits<double>::infinity();
  const double step_x = dx == 0 ? infinity : map.resolution / std::abs(dx);
  const double step_y = dy == 0 ? infinity : map.resolution / std::abs(dy);
  double tx = dx == 0 ? infinity : (map.origin_x + (x + (sx > 0)) * map.resolution - a.first) / dx;
  double ty = dy == 0 ? infinity : (map.origin_y + (y + (sy > 0)) * map.resolution - a.second) / dy;
  auto free = [&](int gx, int gy) { return map.inside(gx, gy) && map.data[gy * map.width + gx] < 253; };
  while (free(x, y)) {
    if (x == end_x && y == end_y) { return true; }
    // Check both neighbours when crossing a grid corner.
    if (std::abs(tx - ty) < 1e-12) {
      if (!free(x + sx, y) || !free(x, y + sy)) { return false; }
      x += sx; y += sy; tx += step_x; ty += step_y;
    } else if (tx < ty) { x += sx; tx += step_x; }
    else { y += sy; ty += step_y; }
  }
  return false;
}

inline SearchResult search_grid(const Costmap2D & map, Point start, Point goal, bool astar) {
  SearchResult result;
  if (!inside_map(map, start) || !inside_map(map, goal)) { return result; }
  auto [sx, sy] = map.world_to_grid(start.first, start.second);
  auto [gx, gy] = map.world_to_grid(goal.first, goal.second);
  auto free = [&](int x, int y) {
    return map.inside(x, y) && map.data[y * map.width + x] < 253;
  };
  if (!free(sx, sy) || !free(gx, gy)) { return result; }
  const int first = sy * map.width + sx, last = gy * map.width + gx;
  const int size = map.width * map.height;
  std::vector<double> cost(size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(size, -1);
  std::vector<bool> closed(size, false);
  auto heuristic = [&](int x, int y) {
    const int dx = std::abs(x - gx), dy = std::abs(y - gy);
    return astar ? dx + dy + (std::sqrt(2.0) - 2) * std::min(dx, dy) : 0.0;
  };
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  cost[first] = 0;
  open.push({heuristic(sx, sy), first});
  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();
    if (closed[current]) { continue; }
    closed[current] = true;
    ++result.expanded;
    if (current == last) {
      result.cost = cost[last];
      for (int cell = last; cell != -1; cell = parent[cell]) {
        result.path.push_back(map.grid_to_world(cell % map.width, cell / map.width));
      }
      std::reverse(result.path.begin(), result.path.end());
      return result;
    }
    const int x = current % map.width, y = current / map.width;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx == 0 && dy == 0) || !free(x + dx, y + dy)) { continue; }
        // A diagonal move must not cut between blocked adjacent cells.
        if (dx && dy && (!free(x + dx, y) || !free(x, y + dy))) { continue; }
        const int next = (y + dy) * map.width + x + dx;
        const double candidate = cost[current] + std::hypot(dx, dy) *
          (1.0 + map.data[next] / 50.0);
        if (candidate < cost[next]) {
          cost[next] = candidate;
          parent[next] = current;
          open.push({candidate + heuristic(x + dx, y + dy), next});
        }
      }
    }
  }
  return result;
}
