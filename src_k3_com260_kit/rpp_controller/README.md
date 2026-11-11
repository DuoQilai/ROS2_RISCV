# 调节纯追踪原理

用当前路径进度选择前瞻点，计算曲率，并按曲率/代价调节速度。示例输出直行、转弯与到达三种输入；`tuning.hpp` 保留速度和前瞻调参练习。此包是原理示例，Gazebo 中的 Nav2 RPP 使用镜像内的官方插件。

从仓库根目录构建并运行：

```bash
bash setup_course_k3_com260_kit.sh --build-ch19
source ~/.config/ros2-course-com260-ch18-ch19/env.bash
ros2 run rpp_controller rpp_controller
```

构建入口会执行本包 `known_inputs` 测试。示例使用 C++17，栅格类型由 `costmap_demo` 提供。Gazebo 的完整操作、截图和视频见[第 18～19 章教案示例运行说明](../../lab_manuals_k3_com260_kit/nav2_planning_setup.md)。
