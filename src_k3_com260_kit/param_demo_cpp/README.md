# COM260 参数与 Launch 示例

节点使用 C++17／rclcpp；Python 文件仅用于 ROS 2 Launch。

| 入口 | 用途与运行端 |
|---|---|
| `param_demo` | COM260：原课程参数生命周期例程，第二轮删除 param5，共执行三轮 |
| `param_node` | COM260：参数 CRUD、范围与枚举校验、YAML 加载 |
| `speed_ctrl` | COM260：10 Hz 动态调速，禁用及退出时发布零速度 |
| `patrol_driver` | x86 Humble：按原时间段巡航，由 `gazebo_ch06.launch.py` 条件启动 |
| `demo.launch.py` | x86 Humble：官方 C++ talker/listener 与可选 RViz |
| `param_with_yaml.launch.py` | COM260：启动参数节点并加载 `/param_demo` YAML |
| `combined_sim_nav.launch.py` | x86 Humble：Burger 仿真、Humble Nav2 与 RViz 的组合启动 |

`param_demo.cpp` 改编自原课程 `src/param_demo_cpp`，保留其 BSD 许可声明；适配 Humble 的动态参数删除要求。地图与导航 RViz 配置来自原课程仿真资源，Nav2 参数以 Humble 安装包为基线。

构建与双端配置见[公共环境](../../lab_manuals_k3_com260_kit/ch00_common_setup.md#ch06)，逐步建包与运行见[第六章实验](../../lab_manuals_k3_com260_kit/ch06_lab.md)。
