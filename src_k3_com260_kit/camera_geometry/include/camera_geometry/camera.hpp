#pragma once
#include <opencv2/calib3d.hpp>

class PinholeCamera {
public:
  cv::Matx33d K;
  int width, height;
  PinholeCamera(double fx, double fy, double cx, double cy, int w = 640, int h = 480)
  : K(fx, 0, cx, 0, fy, cy, 0, 0, 1), width(w), height(h) {}
  cv::Point2d project(const cv::Vec3d & point) const {
    const auto pixel = K * point;
    return {pixel[0] / pixel[2], pixel[1] / pixel[2]};
  }
  cv::Vec3d back_project(const cv::Point2d & pixel, double depth) const {
    return {(pixel.x - K(0, 2)) * depth / K(0, 0),
      (pixel.y - K(1, 2)) * depth / K(1, 1), depth};
  }
};

class BundleAdjustment {
public:
  explicit BundleAdjustment(const cv::Matx33d & camera) : K(camera) {}
  double compute_reprojection_error(const cv::Vec<double, 6> & pose,
      const cv::Vec3d & point, const cv::Point2d & observation) const {
    cv::Matx33d rotation;
    cv::Rodrigues(cv::Vec3d(pose[0], pose[1], pose[2]), rotation);
    const auto camera_point = rotation * point + cv::Vec3d(pose[3], pose[4], pose[5]);
    const auto pixel = K * camera_point;
    return cv::norm(cv::Point2d(pixel[0] / pixel[2], pixel[1] / pixel[2]) - observation);
  }
private:
  cv::Matx33d K;
};
