#pragma once
#include <algorithm>
#include <vector>
#include <Eigen/Dense>

using PointCloud = std::vector<Eigen::Vector3d>;
struct LidarFeatures { PointCloud edge, flat; };

inline LidarFeatures extract_features(const PointCloud & points) {
  if (points.empty()) { return {}; }
  const int count = points.size();
  std::vector<double> curvature(count);
  for (int i = 0; i < count; ++i) {
    const int begin = std::max(0, i - 5), end = std::min(count, i + 5);
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int j = begin; j < end; ++j) { mean += points[j]; }
    curvature[i] = (points[i] - mean / (end - begin)).norm();
  }
  auto sorted = curvature;
  std::sort(sorted.begin(), sorted.end());
  const auto percentile = [&](double fraction) {
    const double index = fraction * (count - 1);
    const int lower = index;
    return sorted[lower] + (index - lower) *
      (sorted[std::min(lower + 1, count - 1)] - sorted[lower]);
  };
  const double edge_threshold = percentile(0.8), flat_threshold = percentile(0.3);
  LidarFeatures features;
  for (int i = 0; i < count; ++i) {
    if (curvature[i] > edge_threshold) { features.edge.push_back(points[i]); }
    if (curvature[i] < flat_threshold) { features.flat.push_back(points[i]); }
  }
  return features;
}
