# Dijkstra 与 A* 栅格规划

`grid_path_planner` 比较同一张 5 × 5 m 栅格地图上的 Dijkstra/A*，输出代价、扩展数、耗时及 SVG。`grid_planner_node` 订阅实际 `/global_costmap/costmap`，按地图原点与分辨率转换，默认规划 (0, 0) → (3, 2)，发布 `/planned_path`。

从仓库根目录构建并运行：

```bash
bash setup_course_k3_com260_kit.sh --build-ch18
source ~/.config/ros2-course-com260-ch18-ch19/env.bash
ros2 run grid_path_planner grid_path_planner
```

构建入口会执行本包 `known_inputs` 测试。示例使用 C++17，栅格类型由 `costmap_demo` 提供。Gazebo 的完整操作、截图和视频见[第 18～19 章教案示例运行说明](../../lab_manuals_k3_com260_kit/nav2_planning_setup.md)。
