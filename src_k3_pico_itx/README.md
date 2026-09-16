# C++ / RISC-V K3 课程源码

第一至四章按以下包组织：

| 包 | 执行端 | 用途 |
|---|---|---|
| `lifecycle_demo_cpp` | K3 | 生命周期状态迁移、速度发布与停止 |
| `robot_sim_demo` | x86 Humble 容器 | Burger 仿真、桥接、相机内参和 RViz 显示 |
| `hello_pkg_cpp`、`name_demo_cpp` | K3 | 第 2 章节点、日志、命名空间及里程计 |
| `topic_demo_cpp`、`topic_demo_interfaces`、`sensor_interfaces`、`sensor_pub_cpp` | K3 | 第 3 章话题、自定义消息、QoS、执行器及方形运动 |
| `service_demo_cpp`、`service_demo_interfaces`、`service_demo_lab_cpp`、`weather_interfaces`、`weather_srv`、`speed_interfaces`、`speed_control` | K3 | 第 4 章服务、超时重试、天气及速度控制 |

在仓库根目录运行 `bash setup_course_k3.sh`，将本目录同步到 `~/ros2_course_k3_ws/src/course/` 后构建。x86 按 [运行支持说明](../course_support/k3/README.md)构建。

原版源码位于 `../src/`。两个版本使用相同的 ROS 包名，应分别构建和加载。手工构建 K3 版时使用 `colcon build --base-paths src_k3_pico_itx`，不要在仓库根目录无参数递归发现两套包。切换版本时使用新的终端，仅加载目标版本的工作区。
