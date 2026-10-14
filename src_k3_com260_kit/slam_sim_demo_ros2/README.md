# slam_sim_demo_ros2

第 10 章在线建图入口与 C++ 地图增长检查。`slam_demo.launch.py` 在 x86 启动 Gazebo、slam_toolbox、RViz，`slam_map_runner` 在 COM260 运行。

运动循环保留原值：前 8 秒 0.18 m/s，随后 4 秒原地 0.5 rad/s，至第 20 秒恢复 0.18 m/s。超过两个地图与激光样本、离起点大于 0.15 m、已知单元增加大于 20 时输出 `slam-map-updated`。默认时限 60 秒；成功、超时与 SIGINT/SIGTERM 退出都发送零速度。

检查结果只适用于 slam_toolbox，不能代替 Hector、gmapping 或 Cartographer 算法验证。
