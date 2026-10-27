#pragma once
#include "slam_teaching_cpp/bayes.hpp"
#include <algorithm>
#include <limits>
#include <numeric>
#include <utility>
namespace slam_teaching {
using Points = std::vector<Vector2d>;
inline Eigen::Matrix2d rotation(double theta) {
  Eigen::Matrix2d R; R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta); return R;
}
inline std::size_t nearest(const Vector2d & point, const Points & cloud) {
  if (cloud.empty()) { throw std::invalid_argument("目标点云为空"); }
  return static_cast<std::size_t>(std::min_element(cloud.begin(), cloud.end(),
    [&point](const Vector2d & a, const Vector2d & b) {
      return (a - point).squaredNorm() < (b - point).squaredNorm();
    }) - cloud.begin());
}
struct MatchResult { Vector3d pose = Vector3d::Zero(); std::vector<double> errors; };

// BEGIN ICP
class ICP {
public:
  int max_iterations; double tolerance;
  explicit ICP(int iterations = 50, double tol = 1e-6) : max_iterations(iterations), tolerance(tol) {}
  MatchResult align(const Points & source, const Points & target,
    const Vector3d & initial = Vector3d::Zero()) const {
    if (source.empty() || target.empty()) { throw std::invalid_argument("ICP要求非空点云"); }
    Eigen::Matrix2d R = rotation(initial.z()); Vector2d t = initial.head<2>();
    MatchResult result;
    double previous = std::numeric_limits<double>::infinity();
    for (int step = 0; step < max_iterations; ++step) {
      Points transformed, matched;
      Vector2d mu_source = Vector2d::Zero(), mu_target = Vector2d::Zero();
      double error = 0.0;
      for (const auto & point : source) {
        const Vector2d p = R * point + t;
        const Vector2d q = target[nearest(p, target)];
        transformed.push_back(p); matched.push_back(q);
        mu_source += p; mu_target += q; error += (p - q).norm();
      }
      mu_source /= static_cast<double>(source.size()); mu_target /= static_cast<double>(source.size());
      Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
      for (std::size_t i = 0; i < source.size(); ++i) {
        W += (transformed[i] - mu_source) * (matched[i] - mu_target).transpose();
      }
      Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix2d V = svd.matrixV();
      Eigen::Matrix2d increment = V * svd.matrixU().transpose();
      if (increment.determinant() < 0) {
        V.col(1) *= -1; increment = V * svd.matrixU().transpose();
      }
      t = increment * t + mu_target - increment * mu_source;
      R = increment * R;
      error /= static_cast<double>(source.size()); result.errors.push_back(error);
      if (std::abs(previous - error) < tolerance) { break; }
      previous = error;
    }
    result.pose << t, std::atan2(R(1, 0), R(0, 0)); return result;
  }
};
// END ICP

// BEGIN PointToLineICP
class PointToLineICP {
public:
  int max_iterations; double tolerance;
  explicit PointToLineICP(int iterations = 20, double tol = 1e-6)
  : max_iterations(iterations), tolerance(tol) {}
  Points estimate_normals(const Points & points, int k = 5) const {
    if (points.size() < static_cast<std::size_t>(k + 1)) {
      throw std::invalid_argument("法向量估计要求至少 k+1 个点");
    }
    Points normals;
    for (std::size_t i = 0; i < points.size(); ++i) {
      std::vector<std::size_t> ids(points.size()); std::iota(ids.begin(), ids.end(), 0);
      std::stable_sort(ids.begin(), ids.end(), [&](std::size_t a, std::size_t b) {
        if (a == i) { return b != i; } if (b == i) { return false; }
        return (points[a] - points[i]).squaredNorm() < (points[b] - points[i]).squaredNorm();
      });
      Vector2d center = Vector2d::Zero();
      for (int j = 1; j <= k; ++j) { center += points[ids[j]]; } center /= k;
      Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
      for (int j = 1; j <= k; ++j) {
        Vector2d d = points[ids[j]] - center; covariance += d * d.transpose();
      }
      covariance /= k - 1;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
      Vector2d normal = solver.eigenvectors().col(0).normalized();
      if (normal.dot(points[i]) < 0) { normal *= -1; } normals.push_back(normal);
    }
    return normals;
  }
  MatchResult align(const Points & source, const Points & target,
    const Vector3d & initial = Vector3d::Zero()) const {
    if (source.empty()) { throw std::invalid_argument("PLICP要求非空源点云"); }
    MatchResult result; result.pose = initial;
    const auto normals = estimate_normals(target);
    double previous = std::numeric_limits<double>::infinity();
    for (int step = 0; step < max_iterations; ++step) {
      const double theta = result.pose.z();
      const Eigen::Matrix2d R = rotation(theta);
      Eigen::Matrix2d derivative; derivative << -std::sin(theta), -std::cos(theta), std::cos(theta), -std::sin(theta);
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero(); Vector3d b = Vector3d::Zero(); double error = 0.0;
      for (const auto & point : source) {
        const Vector2d transformed = R * point + result.pose.head<2>();
        const auto idx = nearest(transformed, target); const Vector2d & normal = normals[idx];
        const double residual = normal.dot(transformed - target[idx]);
        Vector3d J(normal.x(), normal.y(), normal.dot(derivative * point));
        H += J * J.transpose(); b += J * residual; error += residual * residual;
      }
      result.pose += (H + 1e-6 * Eigen::Matrix3d::Identity()).ldlt().solve(-b);
      result.pose.z() = wrap(result.pose.z());
      error /= static_cast<double>(source.size()); result.errors.push_back(error);
      if (std::abs(previous - error) < tolerance) { break; } previous = error;
    }
    return result;
  }
};
// END PointToLineICP
}  // namespace slam_teaching
