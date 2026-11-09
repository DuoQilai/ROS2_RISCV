#include <fstream>
#include <iostream>
#include "costmap_demo/costmap.hpp"

int main() {
  std::ofstream svg("inflation_curves.svg");
  svg << "<svg xmlns='http://www.w3.org/2000/svg' width='800' height='460' viewBox='0 0 800 460'>"
      << "<rect width='800' height='460' fill='white'/>"
      << "<g font-family='sans-serif' font-size='16' fill='#222'>"
      << "<text x='60' y='28'>Inflation costs (radius=0.55 m, inscribed=0.22 m)</text>";
  for (int i = 0; i <= 5; ++i) {
    const int x = 60 + 140 * i, y = 400 - 68 * i;
    svg << "<path d='M" << x << " 60V400 M60 " << y << "H760' stroke='#ddd'/>"
        << "<text x='" << x - 8 << "' y='425'>" << i / 5.0 << "</text>"
        << "<text x='12' y='" << y + 5 << "'>" << 50 * i << "</text>";
  }
  const double factors[] = {1, 3, 5, 10};
  const char * colors[] = {"#0072b2", "#d55e00", "#009e73", "#cc79a7"};
  for (int line = 0; line < 4; ++line) {
    InflationLayer inflation{0.55, factors[line], 0.22};
    svg << "<polyline fill='none' stroke='" << colors[line] << "' stroke-width='2' points='";
    for (int i = 0; i < 100; ++i) {
      const double distance = i / 99.0;
      svg << 60 + 700 * distance << ',' << 400 - 1.36 * inflation.compute_cost(distance) << ' ';
    }
    svg << "'/><text x='560' y='" << 85 + 25 * line << "' fill='" << colors[line]
        << "'>scaling=" << factors[line] << "</text>";
  }
  svg << "<text x='300' y='452'>Distance from obstacle (m)</text></g></svg>\n";
  if (!svg) { std::cerr << "无法写入 inflation_curves.svg\n"; return 1; }
  std::cout << "inflation_curves.svg: 4 curves, 100 samples each\n";
}
