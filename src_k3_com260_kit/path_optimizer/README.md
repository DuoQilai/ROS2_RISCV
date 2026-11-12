# 路径平滑、修剪与重采样

包含梯度平滑、夹持三次 B 样条、Douglas–Peucker 修剪、按弧长重采样和法线方向代价优化。平滑/修剪/优化都检查新连线，输入应是可行路径；重采样保持原折线路径。`max_shift` 是相对原点位移上限，不代表障碍净距。

从仓库根目录构建并运行：

```bash
bash setup_course_k3_com260_kit.sh --build-ch18
source ~/.config/ros2-course-com260-ch18-ch19/env.bash
ros2 run path_optimizer path_optimizer
```

构建入口会执行本包 `known_inputs` 测试。示例使用 C++17，栅格类型由 `costmap_demo` 提供。Gazebo 的完整操作、截图和视频见[第 18～19 章教案示例运行说明](../../lab_manuals_k3_com260_kit/nav2_planning_setup.md)。
