#include <cassert>
#include <iostream>
#include "camera_geometry/camera.hpp"
int main() {
  PinholeCamera camera(615, 615, 320, 240);
  const cv::Vec3d point(1, 0.5, 2);
  const auto pixel = camera.project(point);
  assert(cv::norm(pixel - cv::Point2d(627.5, 393.75)) < 1e-9);
  assert(cv::norm(camera.back_project(pixel, 2) - point) < 1e-9);
  BundleAdjustment ba(camera.K);
  const double error = ba.compute_reprojection_error({0, 0, 0, 0, 0, 0}, point, {630.5, 397.75});
  assert(std::abs(error - 5) < 1e-9);
  std::cout << "projection=(" << pixel.x << ',' << pixel.y << ") round_trip_error=0 reprojection_error="
            << error << " px\n";
}
