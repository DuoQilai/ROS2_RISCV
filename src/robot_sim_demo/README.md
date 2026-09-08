# robot_sim_demo

ISCAS Museum Gazebo Sim 仿真包：使用 TurtleBot3 Burger 机器人模型在 ISCAS Museum 场景中进行巡航仿真。

## 目录结构

```text

src/robot_sim_demo/

├── launch/                    Gazebo 启动文件

│   ├── gazebo2.launch.py      博物馆主启动入口

│   └── campus_pucrs.launch.py Campus PUCRS 启动入口

├── config/

│   └── gazebo2_bridge.yaml    ROS-Gazebo 话题桥配置

├── gui/

│   ├── museum.gui.config      博物馆 Gazebo GUI 配置

│   └── campus_pucrs.gui.config Campus PUCRS GUI 配置

├── models/

│   ├── turtlebot3_burger/      TurtleBot3 Burger SDF 模型（官方 STL 网格）

│   ├── ISCAS_Museum/           博物馆场景模型（含 DAE 网格和纹理）

│   ├── ISCAS_groundplane/      地面模型

│   └── campus_patrol_robot/    备用巡逻机器人模型（同 TurtleBot3 Burger 几何）

├── worlds/

│   ├── museum.sdf             ISCAS Museum 世界文件

│   └── campus_pucrs.world.sdf Campus PUCRS 世界文件

├── rviz/

│   ├── museum.rviz            博物馆 RViz 配置（默认不启动）

│   └── campus_pucrs.rviz      Campus PUCRS RViz 配置

├── urdf/

│   ├── turtlebot3_burger.urdf  TF 树与 RViz 模型描述

│   └── campus_patrol_robot.urdf

├── src/

│   └── camera_info_publisher.cpp  相机内参发布节点（C++17）

└── robot_sim_demo/

    └── patrol_driver.py          巡航驱动节点
```

## 环境

- ROS 2 Humble（openEuler 24.03 RISC-V）；主机端仿真可用 Ubuntu 22.04/24.04 WSL2
- Gazebo Sim（ros_gz）

```bash
sudo dnf install -y ros-humble-ros-gz ros-humble-robot-state-publisher \
  ros-humble-rviz2 python3-colcon-common-extensions
```

## 构建

```bash

cd robot_sim_demo

source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select robot_sim_demo

source install/setup.bash
```

## 启动

### 默认启动（GUI + 自动巡航）

```bash
ros2 launch robot_sim_demo gazebo2.launch.py
```

启动 Gazebo 3D Scene 窗口、TurtleBot3 Burger 机器人、传感器桥和自动巡航。

### Campus PUCRS 启动（黄色标志中心）

```bash

ros2 launch robot_sim_demo campus_pucrs.launch.py
```

该入口使用 `worlds/campus_pucrs.world.sdf`，将 TurtleBot3 Burger 初始位置固定在世界中
黄色 X 标志的中心 `(x=20.0, y=0.0, z=0.010)`。为避免车辆在启动后离开标志，
该入口默认关闭自动巡航；需要运动时显式设置 `drive:=true`。原有
`gazebo2.launch.py` 仍使用 `museum.sdf`，其行为和默认位置不变。

### 常用选项

```bash
# 无 GUI（headless）
ros2 launch robot_sim_demo gazebo2.launch.py gui:=false

# 不启动 RViz（默认已关闭）
ros2 launch robot_sim_demo gazebo2.launch.py rviz:=false

# 不自动巡航（手动控制 /cmd_vel）
ros2 launch robot_sim_demo gazebo2.launch.py drive:=false

# 自定义生成位置
ros2 launch robot_sim_demo gazebo2.launch.py spawn_x:=0.0 spawn_y:=0.0 spawn_z:=0.010 spawn_yaw:=0.0

# 自定义巡航速度
ros2 launch robot_sim_demo gazebo2.launch.py drive_linear_speed:=5.0 drive_angular_speed:=1.5
```

### Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gui` | `true` | 启动 Gazebo GUI |
| `rviz` | `false` | 启动 RViz2 |
| `spawn_robot` | `true` | 在场景中生成机器人 |
| `drive` | `true` | 启动自动巡航节点 |
| `drive_linear_speed` | `5.0` | 巡航线速度 (m/s) |
| `drive_angular_speed` | `1.5` | 巡航角速度 (rad/s) |
| `drive_loop` | `true` | 循环巡航 |
| `world` | `museum.sdf` | 世界文件路径 |
| `spawn_x/y/z/yaw` | `0/0/0.010/0` | 机器人生成位姿（z=0.010 为 TurtleBot3 Burger 准确接地高度） |
| `use_sim_time` | `true` | 使用仿真时钟 |
| `gz_partition` | `robot_sim_demo` | Gazebo 分区名 |

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS→Gazebo | 底盘速度命令 |
| `/odom` | `nav_msgs/Odometry` | Gazebo→ROS | 里程计 |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo→ROS | 激光雷达 |
| `/tf` | `tf2_msgs/TFMessage` | 双向 | 坐标变换 |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo→ROS | 仿真时钟 |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo→ROS | 相机图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | ROS 节点 | 相机内参（320x180, FOV 60°） |

## 手动控制

```bash
# 启动仿真（不自动巡航）
ros2 launch robot_sim_demo gazebo2.launch.py drive:=false

# 发送速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \

  "{linear: {x: 0.15}, angular: {z: 0.0}}"

# 查看里程计
ros2 topic echo /odom --once
```

## 验证

```bash
ros2 topic echo /clock --once
ros2 topic echo /scan --once
ros2 topic echo /camera/camera_info --once
ros2 topic hz /camera/image_raw
```

## 测试

```bash

cd src/robot_sim_demo

python3 -m pytest test/ -v
```

10 项测试全部通过：检查必需文件存在性、SDF/URDF 格式正确性、世界文件引用、Launch 引用、相机内参匹配、DiffDrive 插件配置、TurtleBot3 Burger 模型网格引用，以及轮子碰撞几何与关节轴一致性（Y 轴、双轮差速、最高 10 m/s）。

## 机器人模型

TurtleBot3 Burger 使用 Gazebo Sim 原生 `DiffDrive` 系统，网格取自
ROBOTIS 官方 `turtlebot3_description`（humble 分支），质量/惯量按 5 m/s
高速巡航调校：
- 初始位置：场景中心开放区域 `(0, 0, 0.010)`（准确接地高度）
- 坐标系：`base_footprint` → `base_link`（底盘）→ `laser_link`（激光雷达）→ `camera_link`（课程附加前向相机）
- 双轮差速驱动：`wheel_left_joint` / `wheel_right_joint`（轴距 0.16 m，轮半径 0.033 m），后部万向球
- 质量/惯量：底盘 `3.0 kg`（质心降低、惯量加大以保证高速平稳），驱动轮 `0.15 kg/只`
- 性能参数：巡航速度 `5.0 m/s`、线加速度限幅 `2.5 m/s²`、最高角速度 `3.0 rad/s`（DiffDrive 上限 `10.0 m/s`）
- 轮子碰撞体为沿 Y 轴圆柱（半径 0.033 m），与关节轴一致，滚动摩擦 `mu=1.5`、侧向 `mu2=1.0` 防止高速侧滑
- 外观：金属黑 PBR 材质（`metallic=1.0`、`roughness≈0.35`）底座 `burger_base.stl`、黑色轮胎与 LDS 激光雷达网格
- 两个原始门洞已用与相邻墙面对齐的墙体封闭
