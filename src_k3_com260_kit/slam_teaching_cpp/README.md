# SLAM C++17 教学示例

对应 COM260 教案第10～15章。Eigen 数值示例展示原课程数学步骤；ROS 节点提供传感器、扫描、地图及定位观测。它们与实验第10章实际使用的 `slam_lab`、官方 slam_toolbox、Cartographer、Nav2 AMCL 分别登记。

## 构建与运行

使用[第10章实验](../../lab_manuals_k3_com260_kit/ch10_lab.md)的独立环境和构建入口。依赖包括 Eigen3、rclcpp、TF2、标准消息、Humble Nav2 1.1.20 的 `nav2_msgs` 与固定版本的 `cartographer_ros_msgs`。包中程序统一使用 C++17，两个 Python 文件仅作为 Launch。

COM260：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run slam_teaching_cpp bayes_demo
ros2 run slam_teaching_cpp icp_demo
ros2 run slam_teaching_cpp hector_demo
ros2 run slam_teaching_cpp particle_demo
ros2 run slam_teaching_cpp graph_demo
ros2 run slam_teaching_cpp analysis_demo
```

程序输出实际数值；内置已知输入检查失败时非零退出。带噪声配准、性能计时和退化走廊输出用于观察，不以“必须比另一个算法快若干倍”作为结论。C++随机分布与NumPy不保证逐点相同序列。

## 程序与边界

| 入口 | 教学内容 | 输入与检查 |
|---|---|---|
| `bayes_demo` | 贝叶斯、EKF、加性图优化、线性信息更新 | 已知后验均值、路标初始化、图残差修正 |
| `icp_demo` | SVD ICP、点线ICP | 原随机点/墙面实验及无噪声刚体变换 |
| `hector_demo` | 双线性插值、地图更新、金字塔 | 0.15插值值、占据/空闲log-odds、100/200/400像素层 |
| `particle_demo` | RBPF、FastSLAM、提议分布、MCL/KLD | Neff=4/1、运动dt=0.1、似然/距离场/重采样 |
| `graph_demo` | SE(2)位姿图、体素、外推、运动筛选 | 有限差分雅可比、图优化残差、阈值 |
| `analysis_demo` | 初值/噪声/走廊分析、鲁棒ICP、教学片段 | 已知刚体变换、平坦地图零梯度、IMU权重、连续异常检测 |
| `sensor_monitor` | 激光/IMU/里程计 | `/scan`、`/imu`、`/odom`，每种日志2 s节流 |
| `tf_monitor` | TF查询及点转换 | map→base_link、laser_link中的(1,0,0)，1 s周期 |
| `scan_matcher` | 相邻激光匹配 | 第6帧初始化参考、0.5 s周期、相对里程计初值 |
| `hector_slam` | 独立教学扫描地图匹配 | `/scan`与TF，输出`/map`、`/slam_out_pose`、map→odom |
| `hector_localization` | 已知图匹配 | `/map`、`/scan`，仅输出激光位姿日志 |
| `map_quality_monitor` | 栅格统计 | 每10张地图统计空闲、占据、未知 |
| `cartographer_map_quality` | 地图/子图统计 | `/map`及`SubmapList.submap` |
| `amcl_interface` | 初始位姿与定位质量 | `/amcl_pose`、`/particle_cloud`，可传`x y yaw_rad` |
| `amcl_monitor` | 定位统计 | 最近100项粒子散布、位置方差及位置协方差迹 |

动态ROS示例：

```bash
source ~/.config/ros2-course-com260-ch09-ch10/env.bash
ros2 run slam_teaching_cpp sensor_monitor --ros-args -p use_sim_time:=true
```

仿真、建图或定位系统按实验手册另开终端启动；节点以 Ctrl+C 停止。`hector_slam`发布地图和map→odom，必须与其他建图/定位系统分开运行。先验证实际消息和TF，再评价算法，启动成功不代表几何精度通过。

原教案中的 `CartographerCore` 缺少历史子图约束及时间状态实现；`GlobalSLAM` 原回环引用产生自环；多传感器AMCL缺少扫描/里程计回调；自动恢复缺少可靠地图位姿的定义。这些保留为明确的流程/设计练习，未包装为完整产品。FastSLAM、gmapping与LocalSLAM类也保留各自简化范围，详见对应教案说明。数学和接口修正与原程序差异逐项写在代码旁的正文中。

目标平台构建与运行结果统一见[第10章运行证据](../../lab_manuals_k3_com260_kit/runtime_evidence.md#ch10)。
