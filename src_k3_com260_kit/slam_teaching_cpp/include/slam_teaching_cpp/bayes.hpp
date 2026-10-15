#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace slam_teaching {
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
inline double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// BEGIN BayesianFilter
class BayesianFilter {
public:
  Vector3d belief_mean = Vector3d::Zero();
  Eigen::Matrix3d belief_cov = Eigen::Matrix3d::Identity() * 0.1;
  void predict(const Vector2d & control, const Vector3d & motion_noise) {
    const double theta = belief_mean.z();
    belief_mean += Vector3d(control.x() * std::cos(theta) * 0.1,
      control.x() * std::sin(theta) * 0.1, control.y() * 0.1);
    belief_cov += motion_noise.asDiagonal();
  }
  void update(const Vector3d & observation, const Vector3d & obs_noise) {
    Eigen::Matrix3d S = belief_cov + obs_noise.asDiagonal().toDenseMatrix();
    Eigen::Matrix3d K = belief_cov * S.inverse();
    belief_mean += K * (observation - belief_mean);
    belief_cov = (Eigen::Matrix3d::Identity() - K) * belief_cov;
  }
};
// END BayesianFilter

// BEGIN SLAMBayesianFilter
class SLAMBayesianFilter {
public:
  VectorXd mean;
  MatrixXd cov;
  std::vector<bool> landmarks_seen;
  explicit SLAMBayesianFilter(int num_landmarks = 100)
  : mean(VectorXd::Zero(3 + 2 * num_landmarks)),
    cov(MatrixXd::Identity(mean.size(), mean.size()) * 0.1),
    landmarks_seen(num_landmarks, false) {}
  void predict(const Vector2d & odom, const Vector3d & noise) {
    const double v = odom.x(), theta = mean[2], dt = 0.1;
    mean.head<3>() += Vector3d(v * std::cos(theta) * dt,
      v * std::sin(theta) * dt, odom.y() * dt);
    MatrixXd G = MatrixXd::Identity(mean.size(), mean.size());
    G(0, 2) = -v * std::sin(theta) * dt;
    G(1, 2) = v * std::cos(theta) * dt;
    cov = G * cov * G.transpose();
    cov.topLeftCorner<3, 3>() += noise.asDiagonal();
  }
  void update_with_landmark(const Vector2d & z, int id, const Vector2d & noise) {
    const int idx = 3 + 2 * id;
    if (!landmarks_seen.at(id)) {
      mean.segment<2>(idx) = mean.head<2>() + z.x() *
        Vector2d(std::cos(mean[2] + z.y()), std::sin(mean[2] + z.y()));
      landmarks_seen[id] = true;
      return;
    }
    Vector2d d = mean.segment<2>(idx) - mean.head<2>();
    const double q = d.squaredNorm(), r = std::sqrt(q);
    if (q == 0.0) { throw std::invalid_argument("路标与机器人重合，观测雅可比未定义"); }
    MatrixXd H = MatrixXd::Zero(2, mean.size());
    H(0, 0) = -d.x() / r; H(0, 1) = -d.y() / r;
    H(0, idx) = d.x() / r; H(0, idx + 1) = d.y() / r;
    H(1, 0) = d.y() / q; H(1, 1) = -d.x() / q; H(1, 2) = -1.0;
    H(1, idx) = -d.y() / q; H(1, idx + 1) = d.x() / q;
    Vector2d innovation(z.x() - r, wrap(z.y() - std::atan2(d.y(), d.x()) + mean[2]));
    Eigen::Matrix2d S = H * cov * H.transpose() + noise.asDiagonal().toDenseMatrix();
    MatrixXd K = cov * H.transpose() * S.inverse();
    mean += K * innovation;
    cov = (MatrixXd::Identity(mean.size(), mean.size()) - K * H) * cov;
  }
};
// END SLAMBayesianFilter

// BEGIN GraphSLAM
class GraphSLAM {
public:
  struct Constraint { int i, j; Vector3d measurement; Eigen::Matrix3d information; };
  std::vector<Vector3d> poses;
  std::vector<Constraint> constraints;
  void add_odometry_constraint(int i, int j, const Vector3d & measurement,
    const Eigen::Matrix3d & information) { constraints.push_back({i, j, measurement, information}); }
  void optimize() {
    if (poses.empty()) { return; }
    const int n = static_cast<int>(3 * poses.size());
    MatrixXd H = MatrixXd::Zero(n, n);
    VectorXd b = VectorXd::Zero(n);
    for (const auto & edge : constraints) {
      // 简化的全局坐标加性约束，不是完整 SE(2) 相对位姿模型。
      const Vector3d error = edge.measurement - (poses.at(edge.j) - poses.at(edge.i));
      const int i = 3 * edge.i, j = 3 * edge.j;
      H.block<3, 3>(i, i) += edge.information;
      H.block<3, 3>(j, j) += edge.information;
      H.block<3, 3>(i, j) -= edge.information;
      H.block<3, 3>(j, i) -= edge.information;
      b.segment<3>(i) += edge.information * error;
      b.segment<3>(j) -= edge.information * error;
    }
    H.topRows(3).setZero(); H.leftCols(3).setZero();
    H.topLeftCorner<3, 3>().setIdentity(); b.head<3>().setZero();
    VectorXd delta = (H + 1e-6 * MatrixXd::Identity(n, n)).ldlt().solve(-b);
    for (std::size_t i = 0; i < poses.size(); ++i) { poses[i] += delta.segment<3>(3 * i); }
  }
};
// END GraphSLAM

// BEGIN suggest_slam_algorithm
inline std::string suggest_slam_algorithm(const std::string & environment = "indoor",
  bool has_odom = true, bool has_imu = false, const std::string & sensor_type = "lidar2d",
  const std::string & precision_requirement = "high", bool realtime = true) {
  (void)precision_requirement; (void)realtime;  // 原示例未使用这两个选型因素。
  if (sensor_type == "lidar2d") {
    if (environment == "indoor" && !has_odom) { return "Hector-SLAM (不依赖里程计)"; }
    if (environment == "indoor" && has_odom) { return "slam_toolbox (ROS2推荐) / gmapping"; }
    if (environment == "large_scale") { return "Cartographer (大场景图优化)"; }
  }
  if (sensor_type == "lidar3d") { return "Cartographer 3D / A-LOAM"; }
  if (sensor_type == "camera") { return has_imu ? "VINS-Fusion / ORB-SLAM3" : "ORB-SLAM2 / DSO"; }
  if (sensor_type == "mixed") { return "LVI-SAM / R3LIVE"; }
  return "slam_toolbox (默认推荐)";
}
// END suggest_slam_algorithm

// BEGIN InformationFormSLAM
class InformationFormSLAM {
public:
  MatrixXd Omega;
  VectorXd xi;
  explicit InformationFormSLAM(int num_landmarks = 100)
  : Omega(MatrixXd::Identity(3 + 2 * num_landmarks, 3 + 2 * num_landmarks) * 0.01),
    xi(VectorXd::Zero(3 + 2 * num_landmarks)) {}
  // 原例 predict 是 pass；本例只演示测量的信息贡献，不提供运动预测。
  void update_with_measurement(const Vector2d & z, int id, const Eigen::Matrix2d & R) {
    const int idx = 3 + 2 * id;
    MatrixXd H = MatrixXd::Zero(2, xi.size());
    H(0, 0) = -1.0; H(0, idx) = 1.0; H(1, 1) = -1.0; H(1, idx + 1) = 1.0;
    // 线性观测 z=H*x 的信息更新；信息向量累加 H^T R^-1 z。
    Omega += H.transpose() * R.inverse() * H;
    xi += H.transpose() * R.inverse() * z;
  }
  VectorXd get_mean() const { return Omega.ldlt().solve(xi); }
  MatrixXd get_covariance() const { return Omega.inverse(); }
};
// END InformationFormSLAM
}  // namespace slam_teaching
