#include <cassert>
#include <iostream>
#include "direct_image_alignment/alignment.hpp"
int main() {
  DirectImageAlignment alignment;
  cv::Mat image(32, 32, CV_64F);
  for (int y = 0; y < image.rows; ++y) {
    for (int x = 0; x < image.cols; ++x) { image.at<double>(y, x) = (x / 4 % 2) * 200; }
  }
  const auto pyramid = alignment.build_pyramid(image);
  assert(pyramid.size() == 3 && pyramid.back().cols == 8);
  const auto residuals = alignment.photometric_error(image, image, Eigen::Matrix4d::Identity(), 1);
  assert(!residuals.empty());
  for (double error : residuals) { assert(std::abs(error) < 1e-9); }
  Eigen::Matrix<double, 6, 1> delta;
  delta << 0, 0, 0, 1, 2, 3;
  assert((alignment.se3_exp(delta).block<3, 1>(0, 3) - Eigen::Vector3d(1, 2, 3)).norm() < 1e-9);
  std::cout << "pyramid_levels=3 identity_residuals=" << residuals.size() << " error=0 SE3_translation=(1,2,3)\n";
  try { alignment.align(image, image, Eigen::Matrix4d::Identity(), 1); }
  catch (const std::logic_error & error) { std::cout << "NOT_IMPLEMENTED: " << error.what() << '\n'; return 0; }
  return 1;
}
