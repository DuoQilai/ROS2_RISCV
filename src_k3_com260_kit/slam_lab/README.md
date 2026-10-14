# slam_lab

第 10 章 C++17 实验包，保留原包名及四个入口：

- `slam_monitor`：使用可靠、Transient Local QoS 统计地图占据、空闲、未知单元。
- `set_initial_pose X Y YAW_DEG`：等待订阅者最多 3 秒，发布三次初始位姿，协方差沿用原值。
- `amcl_evaluator`：在同一 TF 参考系比较 AMCL 与 `/ground_truth/odom`；不把轮里程计当真值。
- `save_cartographer_state TRAJECTORY_ID SERVER_ABSOLUTE_PATH`：依次调用 FinishTrajectory 和 WriteState。失败退出非零；输出路径属于运行 Cartographer 的主机，须预先在该主机创建目录。

`online_mapping.launch.py`、`amcl_localization.launch.py` 和 `cartographer_mapping.launch.py` 在 x86 运行后端。AMCL 必须显式传入本章生成的 `map` 文件。Cartographer 基础 Lua 文件在构建时从已安装的 `cartographer` 包复制，不依赖另一章的地图包。
