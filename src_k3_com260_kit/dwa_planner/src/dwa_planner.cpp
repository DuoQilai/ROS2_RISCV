#include <iostream>
#include "dwa_planner/dwa.hpp"

int main() {
  const Motion state{0, 0, 0, 0, 0};
  for (const Path obstacles : {Path{}, Path{{0.6, 0}}, Path{{0, 0}}}) {
    const auto result = dwa_plan(state, {2, 0}, obstacles, {{0, 0}, {0.5, 0}, {1, 0}, {2, 0}});
    std::cout << "obstacles=" << obstacles.size() << " v=" << result.command.v
      << " w=" << result.command.w << " score=" << result.score
      << " samples=" << result.trajectory.size() << '\n';
  }
}
