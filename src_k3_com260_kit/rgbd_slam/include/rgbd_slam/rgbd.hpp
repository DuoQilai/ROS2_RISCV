#pragma once
#include <algorithm>
#include <open3d/Open3D.h>

class RGBDSLAM {
public:
  using Image = open3d::geometry::Image;
  using Cloud = open3d::geometry::PointCloud;
  struct Keyframe { Eigen::Matrix4d pose; std::shared_ptr<Cloud> cloud; Image rgb, depth; };
  explicit RGBDSLAM(const open3d::camera::PinholeCameraIntrinsic & camera) : K(camera) {}
  Eigen::Matrix4d process_frame(const Image & rgb, const Image & depth,
                               const Eigen::Matrix4d & initial = Eigen::Matrix4d::Identity()) {
    auto cloud = rgbd_to_pointcloud(rgb, depth);
    Eigen::Matrix4d pose = keyframes.empty() ? initial : register_pointcloud(*cloud, *keyframes.back().cloud, initial);
    poses.push_back(pose);
    if (is_keyframe(pose)) {
      keyframes.push_back({pose, cloud, rgb, depth});
      fusion_map();
    }
    return pose;
  }
  std::shared_ptr<Cloud> rgbd_to_pointcloud(const Image & rgb, const Image & depth) const {
    auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(rgb, depth, 1000, 5, false);
    return Cloud::CreateFromRGBDImage(*rgbd, K)->VoxelDownSample(0.01);
  }
  Eigen::Matrix4d register_pointcloud(const Cloud & source, const Cloud & target,
      const Eigen::Matrix4d & initial = Eigen::Matrix4d::Identity()) const {
    using namespace open3d::pipelines::registration;
    return RegistrationICP(source, target, 0.1, initial, TransformationEstimationPointToPoint(),
      ICPConvergenceCriteria(1e-6, 1e-6, 50)).transformation_;
  }
  bool is_keyframe(const Eigen::Matrix4d & pose) const {
    if (poses.size() < 2) { return true; }
    const Eigen::Matrix4d delta = keyframes.back().pose.inverse() * pose;
    const double cosine = std::clamp((delta.block<3, 3>(0, 0).trace() - 1) / 2, -1.0, 1.0);
    return delta.block<3, 1>(0, 3).norm() > 0.1 || std::abs(std::acos(cosine)) > 0.3;
  }
  void fusion_map() {
    using namespace open3d::pipelines::integration;
    ScalableTSDFVolume volume(0.01, 0.03, TSDFVolumeColorType::RGB8);
    for (const auto & frame : keyframes) {
      auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(frame.rgb, frame.depth, 1000, 4, false);
      volume.Integrate(*rgbd, K, frame.pose.inverse());
    }
    point_cloud = volume.ExtractPointCloud();
  }
  std::vector<Keyframe> keyframes;
  std::vector<Eigen::Matrix4d> poses;
  std::shared_ptr<Cloud> point_cloud;
private:
  open3d::camera::PinholeCameraIntrinsic K;
};
