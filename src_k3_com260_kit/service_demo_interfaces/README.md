# service_demo_interfaces

## 简介

本包是 ROS 2 服务（Service）通信示例的接口定义包，基于 `ament_cmake` 构建。

它定义了服务示例中使用的自定义服务接口 `srv/Greeting.srv`，用于在客户端与服务端之间传递问候请求与反馈。

本包不包含任何可执行节点，仅作为接口由本版 `service_demo_cpp`（C++ 实现）依赖。

## 接口定义

### srv/Greeting.srv

描述一次问候交互：客户端发送姓名与年龄，服务端返回问候反馈。

```
string name       # 请求：姓名
int32 age         # 请求：年龄
---
string feedback   # 响应：问候反馈
```

## 构建命令

> 前提：ROS 2 Humble 已安装并完成环境配置。

```bash
cd ~/ROS2_RISCV_COM260
bash setup_course_k3_com260_kit.sh --build-ch04
```

## 验证命令

构建并 source 环境后，执行以下命令查看接口定义：

```bash
source ~/.config/ros2-course-com260/env.bash
ros2 interface show service_demo_interfaces/srv/Greeting
```
