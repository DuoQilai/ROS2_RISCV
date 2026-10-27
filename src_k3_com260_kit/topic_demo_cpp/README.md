# topic_demo_cpp

COM260 Kit / ROS 2 Humble 的 C++17 话题通信示例。独立源码位于 `src_k3_com260_kit/`。

| 入口 | 话题与行为 |
| --- | --- |
| `talker` / `listener` | 核心 `Gps` 示例，`/gps_info`；从 `(1.0, 2.0)` 先发布再按 1.03 / 1.01 缩放，周期 1 秒 |
| `publisher` / `gps_subscriber` | 实验 `/gps_position`，Point 从 `(0, 1, 0)` 开始，`y=2*x+1`，周期 1 秒 |
| `qos_publisher` | 每秒发布 RELIABLE 与 BEST_EFFORT 消息，用于四组兼容性实验 |
| `executor_single` / `executor_multi` | 两发布者、两订阅者；回调等待 0.8 秒，多线程使用 4 个工作线程 |
| `teacher_talker` / `teacher_listener` | 教案 String 完整示例；`/chatter` 从 `Hello ROS 2: 0` 开始，周期 0.5 秒 |
| `cmd_vel_subscriber` | 教案练习：订阅 `/cmd_vel`，打印线速度和角速度 |
| `scan_subscriber` | 教案仿真练习：以 SensorDataQoS 订阅 `/scan`，打印角度范围、点数和 frame |
| `square_driver` | 直行 0.2 m/s × 5 秒、左转 1.57 rad/s × 1 秒，循环四边，结束发布零速度 |

在 COM260 的仓库目录执行：

```bash
bash setup_course_k3_com260_kit.sh --build-ch03
source ~/.config/ros2-course-com260/env.bash
ros2 run topic_demo_cpp publisher
```

另开 COM260 终端加载相同环境后运行 `gps_subscriber`；Ctrl+C 停止各节点。

方形练习在 x86 仿真已启动且 `/clock` 可见后执行 `ros2 run topic_demo_cpp square_driver --ros-args -p use_sim_time:=true`。其直行与转弯时长采用仿真时钟；启动仍保留 5 秒墙钟等待。没有仿真时钟时拒绝启动；暂停仿真时动作计时同时暂停。开始前检查 `/cmd_vel` 没有其他控制发布者，Ctrl+C／SIGTERM 提前中止时先发送零速度，再退出 1，不报告完成；暂停仿真时也能中止。强制终止或断连后须另开终端显式发布零速度。

核心节点名 `pytalker` 沿用原接口，实际实现为 C++；GPS 和方形日志标记沿用 Pico 参考实现，便于对应采集记录。参考代码和文档只改变语言、平台与已确认的时钟行为。
