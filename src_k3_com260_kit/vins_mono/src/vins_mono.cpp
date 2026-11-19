#include <cassert>
#include <iostream>
#include "vins_mono/vins.hpp"
int main() {
  VINSMono vins;
  for (int i = 0; i < 10; ++i) { vins.process_imu({1, 0, 0}, {0, 0, 0}, 0.1); }
  assert((vins.state.position - Eigen::Vector3d(0.5, 0, 0)).norm() < 1e-9);
  assert((vins.state.velocity - Eigen::Vector3d(1, 0, 0)).norm() < 1e-9);
  assert(vins.imu_buffer.size() == 10);
  std::cout << "IMU example: position=" << vins.state.position.transpose()
            << " velocity=" << vins.state.velocity.transpose() << '\n';
  std::cout << suggest_fusion_strategy("indoor", true, true, true, "high") << '\n';
}
