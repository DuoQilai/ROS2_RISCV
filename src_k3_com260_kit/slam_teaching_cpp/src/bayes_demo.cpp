#include <iomanip>
#include <iostream>
#include "slam_teaching_cpp/bayes.hpp"
int main() {
  using namespace slam_teaching;
  std::cout << std::fixed << std::setprecision(6);
  BayesianFilter filter;
  filter.predict(Vector2d(1.0, 0.2), Vector3d(0.01, 0.01, 0.01));
  filter.update(Vector3d(0.12, 0.0, 0.03), Vector3d(0.1, 0.1, 0.1));
  std::cout << "bayes mean=" << filter.belief_mean.transpose()
            << " cov=" << filter.belief_cov.diagonal().transpose() << '\n';
  SLAMBayesianFilter slam(1);
  slam.predict(Vector2d(1.0, 0.0), Vector3d(0.01, 0.01, 0.01));
  slam.update_with_landmark(Vector2d(2.0, 0.0), 0, Vector2d(0.1, 0.1));
  slam.update_with_landmark(Vector2d(2.0, 0.0), 0, Vector2d(0.1, 0.1));
  std::cout << "ekf mean=" << slam.mean.transpose() << '\n';
  GraphSLAM graph;
  graph.poses = {Vector3d::Zero(), Vector3d(1.2, 0.0, 0.0)};
  graph.add_odometry_constraint(0, 1, Vector3d(1.0, 0.0, 0.0), Eigen::Matrix3d::Identity());
  graph.optimize(); std::cout << "graph pose1=" << graph.poses[1].transpose() << '\n';
  InformationFormSLAM info(1);
  info.update_with_measurement(Vector2d(2.0, 0.0), 0, Eigen::Matrix2d::Identity() * 0.1);
  VectorXd mean = info.get_mean();
  std::cout << "information relative=" << (mean.segment<2>(3) - mean.head<2>()).transpose() << '\n';
  std::cout << "suggestion=" << suggest_slam_algorithm() << '\n';
  bool pass = std::abs(filter.belief_mean.x() - 0.110476190476) < 1e-9 &&
    std::abs(filter.belief_mean.z() - 0.025238095238) < 1e-9 &&
    std::abs(slam.mean[3] - 2.1) < 1e-9 && std::abs(graph.poses[1].x() - 1.0) < 1e-6 &&
    std::abs(mean[3] - mean[0] - 2.0 / 1.0005) < 1e-9;
  std::cout << "pass=" << std::boolalpha << pass << '\n';
  return pass ? 0 : 1;
}
