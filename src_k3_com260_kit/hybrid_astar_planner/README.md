# Hybrid-A* 运动基元搜索

在 (x, y, yaw) 状态空间中搜索前后运动基元。检查每个子步的圆形足迹；位置按地图分辨率离散，朝向分为 24 格。默认起终点 (-1, -1, 0) → (1, 1, 0)，目标位置误差 < 0.5 m、朝向误差 ≤ 15°。这是教学实现，不包含 Smac 的解析 Reeds–Shepp 直连。

从仓库根目录构建并运行：

```bash
bash setup_course_k3_com260_kit.sh --build-ch18
source ~/.config/ros2-course-com260-ch18-ch19/env.bash
ros2 run hybrid_astar_planner hybrid_astar_planner
```

构建入口会执行本包 `known_inputs` 测试。示例使用 C++17，栅格类型由 `costmap_demo` 提供。Gazebo 的完整操作、截图和视频见[第 18～19 章教案示例运行说明](../../lab_manuals_k3_com260_kit/nav2_planning_setup.md)。
