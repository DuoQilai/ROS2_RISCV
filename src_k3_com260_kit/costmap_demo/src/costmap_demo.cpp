#include <fstream>
#include <iostream>
#include "costmap_demo/costmap.hpp"

int main() {
  int checks = 0;
  auto check = [&checks](bool value, const char * name) {
    if (!value) { throw std::runtime_error(name); }
    ++checks;
    std::cout << "PASS " << name << '\n';
  };
  try {
    check(occupancy(0) == 0 && occupancy(1) == 1 && occupancy(252) == 98 &&
      occupancy(253) == 99 && occupancy(254) == 100 && occupancy(255) == -1, "代价编码边界");
    Costmap2D map(10, 10);
    map.set_cost(0, 0, 300);
    check(map.get_cost(0, 0) == 255, "代价值上界裁剪");
    map.set_cost(0, 0, -1);
    check(map.get_cost(0, 0) == 0, "代价值下界裁剪");
    map.set_cost(-5.01, -5, 254);
    check(map.data.front() == 255, "负边界不截断到零");
    auto [x, y] = map.grid_to_world(0, 0);
    check(std::abs(x + 4.975) < 1e-9 && std::abs(y + 4.975) < 1e-9, "栅格中心");
    std::fill(map.data.begin(), map.data.end(), 0);
    map.set_cost(0, 0, 254);
    InflationLayer inflation;
    auto inflated = inflation.inflate(map);
    check(inflated.get_cost(0, 0) == 254 && inflated.get_cost(0.1, 0) == 253, "致命与内切区分");
    check(inflation.compute_cost(0.3) == 198 && inflation.compute_cost(0.6) == 0, "膨胀衰减与范围");
    Costmap2D unknown(10, 10);
    auto empty = inflation.inflate(unknown);
    check(empty.data == unknown.data, "未知不作为障碍物膨胀");
    check(merge_layers({unknown, inflated}).get_cost(0, 0) == 254, "未知与已知图层融合");
    check(footprint_cost(inflated, 0, 0, 0.3) == 254, "圆形足迹代价");
    check(recommend_inflation_radius(0.3, "narrow_corridor") == 0.3 * 1.5, "半径教学建议");
    map.reset();
    check(map.to_occupancy_grid().data.front() == -1, "重置与未知消息编码");
    std::ofstream csv("inflation_curves.csv");
    csv << "distance,scaling_1,scaling_3,scaling_5,scaling_10\n";
    for (int i = 0; i < 100; ++i) {
      double distance = i / 99.0;
      csv << distance;
      for (double factor : {1.0, 3.0, 5.0, 10.0}) {
        csv << ',' << static_cast<int>(InflationLayer{0.55, factor, 0.22}.compute_cost(distance));
      }
      csv << '\n';
    }
    if (!csv) { throw std::runtime_error("无法写入 inflation_curves.csv"); }
    std::cout << "CHECKS_PASSED=" << checks << " CSV=inflation_curves.csv\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
