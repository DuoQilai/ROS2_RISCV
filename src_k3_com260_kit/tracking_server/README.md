# COM260 Tracking 动作

三个入口均使用 `tracking_interfaces/action/Tracking`，动作名称为 `/tracking`。两个服务端分别运行，同一时刻只启动一个。

| 入口 | 行为 |
|---|---|
| `server` | 基础通信练习：计算目标向量的三维长度，以 0.25 m/s 推进反馈，不控制底盘 |
| `server_gazebo` | 订阅真实 `/odom`，在 odom 坐标系向目标 x/y 移动；拒绝非零 z |
| `client x y [z]` | 发送目标并打印反馈与最终结果；默认 z=0 |

Gazebo 入口中，`current_position` 是接受目标后按相邻里程计位置累计的实走距离，`distance` 是当前位置到目标的平面直线距离。位置容差 0.10 m，最大线速度 0.25 m/s、角速度 1.0 rad/s。目标超时 120 秒，里程计超过 1 秒未更新则中止；取消和进程中断均先发布零速度。

运行步骤与基础入口的区别见[第五章实验](../../lab_manuals_k3_com260_kit/ch05_lab.md)。
