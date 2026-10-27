#include <iomanip>
#include <iostream>
#include "slam_teaching_cpp/hector.hpp"
int main() {
  using namespace slam_teaching;
  MatrixXd map(3, 3); map << 0, 0.1, 0.2, 0.2, 0.3, 0.4, 0.4, 0.5, 0.6;
  const double value = HectorScanMatcher::interpolate(map, Vector2d(0.5, 0.5));
  OccupancyGridMap grid(20.0, 20.0, 0.05);
  const Points points{Vector2d(1.0, 0.0)};
  for (int i = 0; i < 5; ++i) { grid.update_with_scan(points, Vector3d::Zero()); }
  auto probability = grid.get_probability_map();
  auto endpoint = grid.world_to_grid(Vector2d(1, 0)); auto start = grid.world_to_grid(Vector2d::Zero());
  MultiResolutionMap pyramid; pyramid.build_pyramid(probability);
  const auto pose = HectorScanMatcher().match(points, MatrixXd::Constant(4, 4, 0.5), 0.05, Vector3d::Zero());
  std::cout << std::fixed << std::setprecision(6)
    << "bilinear=" << value << " occupied=" << probability(endpoint.y(), endpoint.x())
    << " free=" << probability(start.y(), start.x()) << " unknown=" << probability(0, 0) << '\n'
    << "levels=" << pyramid.maps.at(0.20).rows() << ',' << pyramid.maps.at(0.10).rows() << ',' << pyramid.maps.at(0.05).rows()
    << " flat_map_pose=" << pose.transpose() << '\n';
  const bool pass = std::abs(value - 0.15) < 1e-12 &&
    std::abs(probability(endpoint.y(), endpoint.x()) - 1.0/(1.0+std::exp(-5.0))) < 1e-12 &&
    std::abs(probability(start.y(), start.x()) - 1.0/(1.0+std::exp(1.5))) < 1e-12 && pose.norm() == 0;
  std::cout << "pass=" << std::boolalpha << pass << '\n'; return pass ? 0 : 1;
}
