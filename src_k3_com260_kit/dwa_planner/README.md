# 动态窗口规划与闭环节点

`dwa_planner` 演示速度采样、轨迹预测、方向/障碍/速度/路径评分。`dwa_controller` 订阅 `/odom`、`/scan` 和 `/plan`，用 TF 统一到 odom 坐标系，并发布 `/cmd_vel`；输入失联 0.5 s 后停止，目标误差 ≤ 0.1 m 时结束，取消/超时也发送零速度。规划节点应在控制器启动后发布路径。

从仓库根目录构建并运行：

```bash
bash setup_course_k3_com260_kit.sh --build-ch19
source ~/.config/ros2-course-com260-ch18-ch19/env.bash
ros2 run dwa_planner dwa_planner
```

构建入口会执行本包 `known_inputs` 测试。示例使用 C++17，栅格类型由 `costmap_demo` 提供。Gazebo 的完整操作、截图和视频见[第 18～19 章教案示例运行说明](../../lab_manuals_k3_com260_kit/nav2_planning_setup.md)。
