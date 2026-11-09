#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>

inline std::int8_t occupancy(std::uint8_t cost) {
  if (cost == 255) { return -1; }
  if (cost == 254) { return 100; }
  if (cost == 253) { return 99; }
  return cost == 0 ? 0 : 1 + 97 * (cost - 1) / 251;
}

class Costmap2D {
public:
  Costmap2D(double width_m, double height_m, double cell_size = 0.05)
  : resolution(cell_size), width(static_cast<int>(width_m / cell_size)),
    height(static_cast<int>(height_m / cell_size)), origin_x(-width_m / 2), origin_y(-height_m / 2),
    data(width * height, 255) {}
  std::pair<int, int> world_to_grid(double x, double y) const {
    return {static_cast<int>(std::floor((x - origin_x) / resolution)),
      static_cast<int>(std::floor((y - origin_y) / resolution))};
  }
  std::pair<double, double> grid_to_world(int x, int y) const {
    return {origin_x + (x + 0.5) * resolution, origin_y + (y + 0.5) * resolution};
  }
  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
  void set_cost(double x, double y, int cost) {
    auto [gx, gy] = world_to_grid(x, y);
    if (inside(gx, gy)) { data[gy * width + gx] = static_cast<std::uint8_t>(std::clamp(cost, 0, 255)); }
  }
  std::uint8_t get_cost(double x, double y) const {
    auto [gx, gy] = world_to_grid(x, y);
    return inside(gx, gy) ? data[gy * width + gx] : 255;
  }
  nav_msgs::msg::OccupancyGrid to_occupancy_grid() const {
    nav_msgs::msg::OccupancyGrid message;
    message.header.frame_id = "map";
    message.info.resolution = resolution;
    message.info.width = width;
    message.info.height = height;
    message.info.origin.position.x = origin_x;
    message.info.origin.position.y = origin_y;
    message.info.origin.orientation.w = 1;
    for (auto cost : data) { message.data.push_back(occupancy(cost)); }
    return message;
  }
  void reset() { std::fill(data.begin(), data.end(), 255); }
  double resolution;
  int width, height;
  double origin_x, origin_y;
  std::vector<std::uint8_t> data;
};

struct InflationLayer {
  double inflation_radius = 0.55;
  double cost_scaling_factor = 3.0;
  double inscribed_radius = 0.22;
  std::uint8_t compute_cost(double distance) const {
    if (distance == 0) { return 254; }
    if (distance > inflation_radius) { return 0; }
    if (distance <= inscribed_radius) { return 253; }
    return static_cast<std::uint8_t>(252 * std::exp(-cost_scaling_factor * (distance - inscribed_radius)));
  }
  Costmap2D inflate(const Costmap2D & input) const {
    auto output = input;
    const int radius = static_cast<int>(std::ceil(inflation_radius / input.resolution));
    for (int y = 0; y < input.height; ++y) {
      for (int x = 0; x < input.width; ++x) {
        if (input.data[y * input.width + x] != 254) { continue; }
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            if (!input.inside(x + dx, y + dy)) { continue; }
            auto & cell = output.data[(y + dy) * input.width + x + dx];
            const auto cost = compute_cost(std::hypot(dx, dy) * input.resolution);
            // Preserve unknown cells except where an inscribed obstacle establishes collision.
            if (cell == 255) { if (cost >= 253) { cell = cost; } }
            else { cell = std::max(cell, cost); }
          }
        }
      }
    }
    return output;
  }
};

inline double recommend_inflation_radius(double radius, const std::string & environment) {
  if (environment == "narrow_corridor") { return radius * 1.5; }
  if (environment == "open_space") { return radius * 3.0; }
  if (environment == "office") { return radius * 2.5; }
  return radius * 2.0;
}

inline Costmap2D merge_layers(const std::vector<Costmap2D> & layers) {
  if (layers.empty()) { throw std::invalid_argument("至少需要一个图层"); }
  auto output = layers.front();
  for (const auto & layer : layers) {
    if (layer.width != output.width || layer.height != output.height ||
        layer.resolution != output.resolution || layer.origin_x != output.origin_x ||
        layer.origin_y != output.origin_y) { throw std::invalid_argument("图层几何必须一致"); }
    for (std::size_t i = 0; i < output.data.size(); ++i) {
      if (layer.data[i] == 255) { continue; }
      output.data[i] = output.data[i] == 255 ? layer.data[i] : std::max(output.data[i], layer.data[i]);
    }
  }
  return output;
}

inline std::uint8_t footprint_cost(const Costmap2D & map, double x, double y, double radius) {
  const int cells = static_cast<int>(std::ceil(radius / map.resolution));
  std::uint8_t maximum = 0;
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      if (std::hypot(dx, dy) * map.resolution <= radius) {
        maximum = std::max(maximum, map.get_cost(x + dx * map.resolution, y + dy * map.resolution));
      }
    }
  }
  return maximum;
}
