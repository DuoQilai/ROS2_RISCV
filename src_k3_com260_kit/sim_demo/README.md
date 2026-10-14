# sim_demo

COM260 Kit 第 9 章独立仿真练习包。`sensor_reader` 在 COM260 运行；Gazebo Harmonic、RViz 和 ros2_control 在 x86 Humble 容器运行。

- `sim_bringup.launch.py`：默认空世界；`world` 可指定本包 Museum 或 Campus 世界。
- `sensor_profile:=lab`：360 点、180° LiDAR；`teacher`：360 点、360° LiDAR。两者保留相机 640×480 和有效距离上限 12 m。
- `sensor_reader`：订阅 `/scan`、`/camera/image_raw`，输出原练习中的距离摘要；相机日志每 2 秒最多一条。
- `/cmd_vel` 为 `geometry_msgs/msg/Twist`，交给真实 `diff_drive_controller`；轮距 0.16 m、轮半径 0.033 m 对应 Burger，超时 1 秒停止。
- `/ground_truth/odom` 来自 Gazebo 实体位姿；`/odom` 来自轮编码器控制器。二者不能互相冒充。

传感器采用 `SensorDataQoS`，与 Best Effort 发布者兼容。模型与控制器配置均为独立副本，网格和场景纹理复用 `robot_sim_demo` 的现有安装资源。
