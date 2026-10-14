#pragma once
#include "slam_teaching_cpp/scan_matching.hpp"
#include <cstdint>
#include <iostream>
namespace slam_teaching {
// BEGIN HectorScanMatcher
class HectorScanMatcher {
public:
  int max_iterations; double convergence_delta;
  explicit HectorScanMatcher(int iterations = 15, double delta = 1e-6)
  : max_iterations(iterations), convergence_delta(delta) {}
  static double interpolate(const MatrixXd & map, const Vector2d & p, double outside = 0.5) {
    int x = static_cast<int>(std::floor(p.x())), y = static_cast<int>(std::floor(p.y()));
    if (x < 0 || y < 0 || x + 1 >= map.cols() || y + 1 >= map.rows()) { return outside; }
    double dx = p.x() - x, dy = p.y() - y;
    return map(y, x) * (1-dx) * (1-dy) + map(y, x+1) * dx * (1-dy) +
      map(y+1, x) * (1-dx) * dy + map(y+1, x+1) * dx * dy;
  }
  Vector3d match(const Points & scan, const MatrixXd & map, double resolution,
    Vector3d pose, const Vector2d & origin = Vector2d::Zero()) const {
    if (resolution <= 0 || map.rows() < 2 || map.cols() < 2) {
      throw std::invalid_argument("地图分辨率必须为正，地图至少为2×2");
    }
    MatrixXd gx(map.rows(), map.cols()), gy(map.rows(), map.cols());
    for (int y = 0; y < map.rows(); ++y) { for (int x = 0; x < map.cols(); ++x) {
      const int x0 = std::max(0, x-1), x1 = std::min<int>(map.cols()-1, x+1);
      const int y0 = std::max(0, y-1), y1 = std::min<int>(map.rows()-1, y+1);
      gx(y, x) = (map(y, x1)-map(y, x0)) / ((x1-x0) * resolution);
      gy(y, x) = (map(y1, x)-map(y0, x)) / ((y1-y0) * resolution);
    }}
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
      const Eigen::Matrix2d R = rotation(pose.z());
      Eigen::Matrix2d dR; dR << -std::sin(pose.z()), -std::cos(pose.z()), std::cos(pose.z()), -std::sin(pose.z());
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero(); Vector3d b = Vector3d::Zero();
      for (const auto & point : scan) {
        Vector2d grid = (R * point + pose.head<2>() - origin) / resolution;
        const double residual = 1.0 - interpolate(map, grid);
        Vector2d gradient(interpolate(gx, grid, 0.0), interpolate(gy, grid, 0.0));
        Vector3d J(-gradient.x(), -gradient.y(), -gradient.dot(dR * point));
        H += J * J.transpose(); b += J * residual;
      }
      Vector3d delta = (H + 1e-6 * Eigen::Matrix3d::Identity()).ldlt().solve(-b);
      pose += delta;
      if (delta.norm() < convergence_delta) { break; }
    }
    return pose;
  }
};
// END HectorScanMatcher

// BEGIN MultiResolutionMap
class MultiResolutionMap {
public:
  std::vector<double> resolutions{0.20, 0.10, 0.05};
  std::map<double, MatrixXd> maps;
  void build_pyramid(const MatrixXd & base_map) {
    for (double resolution : resolutions) {
      const int scale = static_cast<int>(std::round(resolution / resolutions.back()));
      MatrixXd downsampled(base_map.rows() / scale, base_map.cols() / scale);
      for (int y = 0; y < downsampled.rows(); ++y) { for (int x = 0; x < downsampled.cols(); ++x) {
        downsampled(y, x) = base_map.block(y * scale, x * scale, scale, scale).mean();
      }}
      maps[resolution] = downsampled;
    }
  }
  const MatrixXd & get_map(double resolution) const {
    auto closest = std::min_element(resolutions.begin(), resolutions.end(),
      [resolution](double a, double b) { return std::abs(a-resolution) < std::abs(b-resolution); });
    return maps.at(*closest);
  }
};
// END MultiResolutionMap

// BEGIN CoarseToFineMatcher
class CoarseToFineMatcher {
public:
  Vector3d match(const Points & scan, const MultiResolutionMap & pyramid, Vector3d pose,
    const Vector2d & origin = Vector2d::Zero()) const {
    for (double resolution : pyramid.resolutions) {
      pose = HectorScanMatcher().match(scan, pyramid.maps.at(resolution), resolution, pose, origin);
      std::cout << "分辨率 " << resolution << "m: 位姿=" << pose.transpose() << '\n';
    }
    return pose;
  }
};
// END CoarseToFineMatcher

// BEGIN OccupancyGridMap
class OccupancyGridMap {
public:
  double resolution; int width_pixels, height_pixels; Vector2d origin;
  MatrixXd log_odds;
  OccupancyGridMap(double width, double height, double res)
  : resolution(res), width_pixels(static_cast<int>(width/res)), height_pixels(static_cast<int>(height/res)),
    origin(-width/2, -height/2), log_odds(MatrixXd::Zero(height_pixels, width_pixels)) {}
  Eigen::Vector2i world_to_grid(const Vector2d & point) const {
    return ((point - origin) / resolution).array().floor().cast<int>().matrix();
  }
  Vector2d grid_to_world(const Eigen::Vector2i & cell) const { return cell.cast<double>() * resolution + origin; }
  bool inside(const Eigen::Vector2i & cell) const {
    return cell.x() >= 0 && cell.y() >= 0 && cell.x() < width_pixels && cell.y() < height_pixels;
  }
  static std::vector<Eigen::Vector2i> bresenham_line(Eigen::Vector2i from, const Eigen::Vector2i & to) {
    std::vector<Eigen::Vector2i> cells;
    const int dx = std::abs(to.x()-from.x()), dy = -std::abs(to.y()-from.y());
    const int sx = from.x() < to.x() ? 1 : -1, sy = from.y() < to.y() ? 1 : -1;
    int error = dx + dy;
    while (true) {
      cells.push_back(from); if (from == to) { break; }
      int e2 = 2 * error;
      if (e2 >= dy) { error += dy; from.x() += sx; }
      if (e2 <= dx) { error += dx; from.y() += sy; }
    }
    return cells;
  }
  void update_with_scan(const Points & scan, const Vector3d & pose) {
    for (const auto & point : scan) {
      Eigen::Vector2i end = world_to_grid(rotation(pose.z()) * point + pose.head<2>());
      if (!inside(end)) { continue; }
      auto cells = bresenham_line(world_to_grid(pose.head<2>()), end);
      for (std::size_t i = 0; i + 1 < cells.size(); ++i) {
        if (inside(cells[i])) { log_odds(cells[i].y(), cells[i].x()) -= 0.3; }
      }
      log_odds(end.y(), end.x()) += 1.0;
    }
    log_odds = log_odds.array().max(-5.0).min(5.0).matrix();
  }
  MatrixXd get_probability_map() const { return (1.0 / (1.0 + (-log_odds.array()).exp())).matrix(); }
  std::vector<int8_t> get_occupancy_grid() const {
    const auto probability = get_probability_map(); std::vector<int8_t> data;
    for (int y = 0; y < height_pixels; ++y) { for (int x = 0; x < width_pixels; ++x) {
      double p = probability(y, x); data.push_back(p < 0.2 ? 0 : p > 0.8 ? 100 : -1);
    }}
    return data;
  }
};
// END OccupancyGridMap
}  // namespace slam_teaching
