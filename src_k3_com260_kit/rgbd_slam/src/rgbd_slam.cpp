#include <algorithm>
#include <cassert>
#include <iostream>
#include "rgbd_slam/rgbd.hpp"
int main() {
  RGBDSLAM slam(open3d::camera::PinholeCameraIntrinsic(64, 48, 60, 60, 32, 24));
  RGBDSLAM::Image rgb, depth;
  rgb.Prepare(64, 48, 3, 1);
  depth.Prepare(64, 48, 1, 2);
  std::fill(rgb.data_.begin(), rgb.data_.end(), 180);
  for (int v = 0; v < 48; ++v) {
    for (int u = 0; u < 64; ++u) { *depth.PointerAt<uint16_t>(u, v) = 1000; }
  }
  assert((slam.process_frame(rgb, depth) - Eigen::Matrix4d::Identity()).norm() < 1e-9);
  assert(slam.keyframes.size() == 1 && !slam.point_cloud->points_.empty());
  RGBDSLAM::Cloud source, target;
  for (int i = 0; i < 100; ++i) {
    const Eigen::Vector3d p(std::sin(i * 2.3), std::cos(i * 1.7), std::sin(i * 0.7));
    source.points_.push_back(p);
    target.points_.push_back(p + Eigen::Vector3d(0.01, -0.02, 0.03));
  }
  const auto transform = slam.register_pointcloud(source, target);
  const double error = (transform.block<3, 1>(0, 3) - Eigen::Vector3d(0.01, -0.02, 0.03)).norm();
  assert(error < 1e-6);
  std::cout << "Known RGB-D input: keyframes=" << slam.keyframes.size()
            << " TSDF_points=" << slam.point_cloud->points_.size() << " ICP_translation_error=" << error << '\n';
}
