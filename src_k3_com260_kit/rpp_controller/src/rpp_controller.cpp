#include <iostream>
#include "rpp_controller/pursuit.hpp"

int main() {
  const Path path{{0, 0}, {0.5, 0}, {1, 0}, {1.5, 0.5}, {2, 1}};
  for (std::array<double, 3> pose : {std::array<double, 3>{0, 0, 0}, {1, 0, 0}, {2, 1, 0}}) {
    const auto command = pure_pursuit(pose, path);
    std::cout << "x=" << pose[0] << " y=" << pose[1] << " v=" << command.v
      << " w=" << command.w << " curvature=" << command.curvature << '\n';
  }
}
