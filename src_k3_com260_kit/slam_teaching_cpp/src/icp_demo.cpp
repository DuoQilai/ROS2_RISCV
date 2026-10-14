#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include "slam_teaching_cpp/scan_matching.hpp"

int main() {
  using namespace slam_teaching;
  std::mt19937 random(42); std::uniform_real_distribution<double> uniform(0.0, 10.0);
  std::normal_distribution<double> noise(0.0, 0.1);
  Points target, source;
  const Eigen::Matrix2d true_R = rotation(0.3); const Vector2d true_t(1.0, 0.5);
  for (int i = 0; i < 100; ++i) {
    const double x = uniform(random), y = uniform(random); target.emplace_back(x, y);
  }
  for (const auto & point : target) {
    const double nx = noise(random), ny = noise(random);
    source.push_back(true_R * point + true_t + Vector2d(nx, ny));
  }
  auto result = ICP().align(source, target);
  Vector3d expected; expected << -true_R.transpose() * true_t, -0.3;
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "source_to_target expected=" << expected.transpose() << '\n';
  std::cout << "estimated=" << result.pose.transpose() << " iterations=" << result.errors.size()
            << " nearest_error=" << result.errors.back() << '\n';
  // 原章的结构化墙面比较：水平各100点、垂直各60点。
  Points walls, transformed;
  for (int i = 0; i < 100; ++i) { double x = -5.0 + 10.0 * i / 99.0; walls.emplace_back(x, -3.0); walls.emplace_back(x, 3.0); }
  for (int i = 0; i < 60; ++i) { double y = -3.0 + 6.0 * i / 59.0; walls.emplace_back(-5.0, y); walls.emplace_back(5.0, y); }
  noise = std::normal_distribution<double>(0.0, 0.05);
  for (const auto & point : walls) {
    const double nx = noise(random), ny = noise(random);
    transformed.push_back(rotation(0.15) * point + Vector2d(0.5, 0.3) + Vector2d(nx, ny));
  }
  auto start = std::chrono::steady_clock::now(); auto icp = ICP(50).align(transformed, walls);
  double icp_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  start = std::chrono::steady_clock::now(); auto plicp = PointToLineICP(20).align(transformed, walls);
  double plicp_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  Vector3d wall_expected; wall_expected << -rotation(0.15).transpose() * Vector2d(0.5, 0.3), -0.15;
  std::cout << "ICP pose=" << icp.pose.transpose() << " iterations=" << icp.errors.size() << " error=" << icp.errors.back() << " ms=" << icp_ms << '\n';
  std::cout << "PLICP pose=" << plicp.pose.transpose() << " iterations=" << plicp.errors.size() << " squared_line_error=" << plicp.errors.back() << " ms=" << plicp_ms << '\n';
  std::cout << "PLICP angle_error_deg=" << std::abs(wrap(plicp.pose.z() - wall_expected.z())) * 180.0 / std::acos(-1.0)
            << " translation_error=" << (plicp.pose.head<2>() - wall_expected.head<2>()).norm() << '\n';
  // 无噪声已知输入作为实现断言；带噪声实验结果单独报告。
  Points exact;
  for (const auto & point : target) { exact.push_back(rotation(0.03) * point + Vector2d(0.1, 0.05)); }
  const auto known = ICP().align(exact, target);
  Vector3d known_expected; known_expected << -rotation(0.03).transpose() * Vector2d(0.1, 0.05), -0.03;
  bool pass = (known.pose - known_expected).norm() < 1e-6 && plicp.pose.allFinite();
  std::cout << "known_input_error=" << (known.pose - known_expected).norm() << " pass=" << std::boolalpha << pass << '\n';
  return pass ? 0 : 1;
}
