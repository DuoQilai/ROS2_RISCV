# C++ / RISC-V K3 课程源码

本次交付第一章的两个包：

| 包 | 执行端 | 用途 |
|---|---|---|
| `lifecycle_demo_cpp` | K3 | 生命周期状态迁移、速度发布与停止 |
| `robot_sim_demo` | x86 Humble 容器 | Burger 仿真、桥接、相机内参和 RViz 显示 |

在仓库根目录运行 `bash setup_course_k3.sh`，将本目录同步到 `~/ros2_course_k3_ws/src/course/` 后构建。x86 按 [运行支持说明](../course_support/k3/README.md)构建。

原版源码位于 `../src/`。两个版本使用相同的 ROS 包名，应分别构建和加载。手工构建 K3 版时使用 `colcon build --base-paths src_k3_pico_itx`，不要在仓库根目录无参数递归发现两套包。切换版本时使用新的终端，仅加载目标版本的工作区。
