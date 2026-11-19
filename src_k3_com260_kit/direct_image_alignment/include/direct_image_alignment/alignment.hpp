#pragma once
#include <algorithm>
#include <cmath>
#include <random>
#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

class DirectImageAlignment {
public:
  explicit DirectImageAlignment(int levels = 3) : num_levels(levels) {}
  std::vector<cv::Mat> build_pyramid(const cv::Mat & image) const {
    std::vector<cv::Mat> pyramid{image};
    for (int i = 1; i < num_levels; ++i) {
      cv::Mat smaller;
      cv::resize(pyramid.back(), smaller, {}, 0.5, 0.5, cv::INTER_LINEAR);
      pyramid.push_back(smaller);
    }
    return pyramid;
  }
  std::vector<double> photometric_error(const cv::Mat & ref, const cv::Mat & cur,
      const Eigen::Matrix4d & pose, double inv_depth) const {
    std::vector<cv::Point> pixels;
    for (int y = 0; y < ref.rows; ++y) {
      for (int x = 0; x < ref.cols; ++x) {
        int left = std::max(0, x - 1), right = std::min(ref.cols - 1, x + 1);
        int top = std::max(0, y - 1), bottom = std::min(ref.rows - 1, y + 1);
        double dx = (ref.at<double>(y, right) - ref.at<double>(y, left)) / (right - left);
        double dy = (ref.at<double>(bottom, x) - ref.at<double>(top, x)) / (bottom - top);
        if (std::hypot(dx, dy) > 10) { pixels.emplace_back(x, y); }
      }
    }
    if (pixels.size() > 1000) {
      std::mt19937 random(std::random_device{}());
      std::shuffle(pixels.begin(), pixels.end(), random);
      pixels.resize(1000);
    }
    std::vector<double> residuals;
    for (const auto & pixel : pixels) {
      double z = 1 / inv_depth;
      Eigen::Vector4d point((pixel.x - ref.cols / 2.0) * z / 500, (pixel.y - ref.rows / 2.0) * z / 500, z, 1);
      const Eigen::Vector4d p = pose * point;
      const double u = p.x() / p.z() * 500 + ref.cols / 2.0;
      const double v = p.y() / p.z() * 500 + ref.rows / 2.0;
      if (u >= 0 && u < ref.cols - 1 && v >= 0 && v < ref.rows - 1) {
        residuals.push_back(ref.at<double>(pixel.y, pixel.x) - cur.at<double>(int(v), int(u)));
      }
    }
    return residuals;
  }
  Eigen::Matrix4d align(const cv::Mat & ref, const cv::Mat & cur,
                       const Eigen::Matrix4d & initial_pose, double inv_depth) const {
    Eigen::Matrix4d pose = initial_pose;
    const auto reference = build_pyramid(ref), current = build_pyramid(cur);
    for (int level = num_levels - 1; level >= 0; --level) {
      for (int iteration = 0; iteration < 10; ++iteration) {
        const auto errors = photometric_error(reference[level], current[level], pose, inv_depth / (1 << level));
        Eigen::Map<const Eigen::VectorXd> residuals(errors.data(), errors.size());
        Eigen::MatrixXd J(0, 6);  // The lesson leaves the photometric Jacobian unimplemented.
        if (J.rows() != residuals.size()) {
          throw std::logic_error("原示例未实现光度雅可比 J；需完成雅可比后才能进行 Gauss-Newton 位姿优化");
        }
        const Eigen::Matrix<double, 6, 6> H = J.transpose() * J + Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
        const Eigen::Matrix<double, 6, 1> delta = H.ldlt().solve(-J.transpose() * residuals);
        pose = se3_exp(delta) * pose;
        if (delta.norm() < 0.001) { break; }
      }
    }
    return pose;
  }
  Eigen::Matrix4d se3_exp(const Eigen::Matrix<double, 6, 1> & xi) const {
    const Eigen::Vector3d w = xi.head<3>(), v = xi.tail<3>();
    double theta = w.norm();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity(), A = R, wx;
    wx << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
    if (theta >= 1e-6) {
      R += std::sin(theta) / theta * wx + (1 - std::cos(theta)) / (theta * theta) * wx * wx;
      A += (1 - std::cos(theta)) / (theta * theta) * wx +
        (theta - std::sin(theta)) / (theta * theta * theta) * wx * wx;
    }
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = A * v;
    return T;
  }
private:
  int num_levels;
};
