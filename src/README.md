# ROS 2 仿真与教学

本工作区包含 46 个可由 colcon 发现和构建的 ROS 2 包，涵盖话题通信、服务通信、动作通信、参数系统、TF 坐标变换、URDF 建模、Gazebo 仿真、SLAM 建图、Nav2 自主导航和 xArm6 机械臂仿真，以及一个完整的 ISCAS Museum 仿真场景。`lab_code/ch16_lab/` 是不带 `package.xml` 的纯文件示例。

## 环境

| 组件 | 版本 |
|------|------|
| 操作系统 | Ubuntu 24.04 (WSL2) |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Sim Harmonic (v8) |
| 构建工具 | colcon + ament |

## 目录结构

```text

├── src/

│   ├── robot_sim_demo/              Gazebo 仿真核心包

│   │   ├── launch/gazebo2.launch.py 主启动入口（museum.sdf）

│   │   ├── launch/campus_pucrs.launch.py Campus PUCRS 启动入口

│   │   ├── models/                  机器人、博物馆、地面模型

│   │   ├── worlds/museum.sdf       仿真世界

│   │   ├── config/                  ROS-Gazebo 桥配置

│   │   ├── gui/                     Gazebo GUI 配置

│   │   └── urdf/                    TurtleBot3 Burger TF/模型描述

│   ├── xarm/                        xArm6 + Gazebo Harmonic + MoveIt 2 仿真

│   ├── xarm_description/             xArm6 URDF、mesh 和 ros2_control 描述

│   ├── navigation_sim_demo_ros2/    Nav2 导航仿真

│   ├── slam_sim_demo_ros2/          SLAM 建图仿真

│   ├── tf_follower_ros2/            TF 跟随控制器

│   ├── topic_demo_py/               话题通信 (Python)

│   ├── topic_demo_cpp/              话题通信 (C++)

│   ├── topic_demo_interfaces/       话题消息接口

│   ├── service_demo_py/             服务通信 (Python)

│   ├── service_demo_cpp/            服务通信 (C++)

│   ├── service_demo_interfaces/     服务接口

│   ├── action_demo_py/              动作通信 (Python)

│   ├── action_demo_cpp/             动作通信 (C++)

│   ├── action_demo_interfaces/      动作接口

│   ├── param_demo_py/               参数系统 (Python)

│   ├── param_demo_cpp/              参数系统 (C++)

│   ├── tf_demo_py/                  TF 坐标变换 (Python)

│   ├── tf_demo_cpp/                 TF 坐标变换 (C++)

│   ├── name_demo_cpp/               命名空间与参数 (C++)

│   ├── msgs_demo_interfaces/       综合消息接口

│   ├── urdf_demo_ros2/            URDF/Xacro 建模演示

│   ├── course_lab_interfaces/      课程实验共享接口

│   ├── course_lab_utils/           课程实验共享实现

│   └── lab_code/                    教学实验包（Ch01-Ch21）

│       ├── ch01_lab/lifecycle_demo/  生命周期节点

│       ├── ch02_lab/hello_pkg/      节点与日志

│       ├── ch03_lab/topic_demo/      话题通信

│       ├── ch03_lab/sensor_pub/     自定义消息

│       ├── ch03_lab/sensor_interfaces/ 传感器接口

│       ├── ch04_lab/service_demo/   服务通信

│       ├── ch05_lab/action_demo/    动作通信

│       ├── ch06_lab/param_demo/     参数系统

│       ├── ch07_lab/tf_demo/        TF 坐标变换

│       ├── ch08_lab/urdf_demo/      URDF 建模

│       ├── ch09_lab/sim_demo/       Gazebo 仿真

│       ├── ch10_lab/slam_lab/       SLAM 建图

│       ├── ch11_lab/navigation_lab/  Nav2 导航

│       ├── ch12_lab/realsense_lab/  RealSense 相机

│       ├── ch13_lab/slam_bringup_lab/ SLAM 一键建图

│       ├── ch14_lab/nav_bringup_lab/  Nav2 一键导航

│       ├── ch15_lab/arm_joint_pub_lab/ xArm 关节发布

│       ├── ch17_lab/moveit_fk_ik_lab/  MoveIt FK/IK

│       ├── ch18_lab/moveit_pick_place_lab/ MoveIt 抓取放置

│       ├── ch19_lab/vision_detection_lab/ 视觉检测

│       └── ch21_lab/vision_pickup_lab/   视觉引导抓取

└── README.md                        本文件
```

## 安装

### 1. 配置 ROS 2 官方软件源

Ubuntu 24.04 使用 ROS 2 Jazzy，Ubuntu 22.04 使用 ROS 2 Humble。当前仓库的 Gazebo、MoveIt 2
和 xArm6 主机仿真优先使用 Jazzy。参考 [ROS 2 Jazzy 官方安装指南](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)：

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update
sudo apt install -y software-properties-common curl ca-certificates
sudo add-apt-repository universe
sudo curl -L https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
```

### 2. 安装 ROS 2 和开发工具

根据 Ubuntu 版本选择一个发行版，不要同时安装两套 ROS 2：

```bash
# Ubuntu 24.04 LTS
export ROS_DISTRO=jazzy
sudo apt install -y ros-jazzy-desktop

# Ubuntu 22.04 LTS：改用上一段后不要重复执行
# export ROS_DISTRO=humble
# sudo apt install -y ros-humble-desktop
```

```bash
sudo apt install -y \
  build-essential cmake git \
  python3-pip python3-rosdep python3-colcon-common-extensions python3-vcstool \
  python3-numpy python3-opencv python3-yaml ffmpeg
```

### 3. 初始化环境

```bash
sudo rosdep init
rosdep update
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. 克隆工作区并安装源码依赖

```bash
git clone https://github.com/YunxiangLuo/ROS2.git
cd ROS2
rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -r -y
```

### 5. 编译全部包

```bash

source /opt/ros/${ROS_DISTRO}/setup.bash
colcon list --base-paths src --names-only | sort -u | wc -l  # 应为 46
colcon build --base-paths src --symlink-install \
  --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
colcon test --base-paths src --executor sequential --return-code-on-test-failure
colcon test-result --verbose
```

源码树当前应发现并安装 46 个可构建包；构建后可检查：

```bash
find -L install -path '*/share/ament_index/resource_index/packages/*' -type f | wc -l  # 应为 46
test -f install/setup.bash && echo 'ROS 2 workspace ready'
```

## 包清单

### 仿真核心

| 包名 | 类型 | 说明 |
|------|------|------|
| `robot_sim_demo` | Python | ISCAS Museum / Campus PUCRS Gazebo 仿真：TurtleBot3 Burger 机器人、传感器桥、巡航驱动 |
| `xarm_ros2_arm_only` | Python | xArm6 纯机械臂仿真：Gazebo Harmonic、ros2_control、MoveIt 2 和 RViz |
| `xarm_description` | CMake | xArm6 URDF、mesh 和 ros2_control 描述 |
| `navigation_sim_demo_ros2` | Python | Nav2 导航栈：地图、AMCL、规划、控制 |
| `slam_sim_demo_ros2` | Python | slam_toolbox 在线建图 |
| `tf_follower_ros2` | Python | TF 跟随控制器：基于坐标变换的速度控制 |

### 通信示例

| 包名 | 语言 | 说明 |
|------|------|------|
| `topic_demo_py` | Python | 话题通信（GPS 数据发布/订阅） |
| `topic_demo_cpp` | C++ | 话题通信（GPS 数据发布/订阅） |
| `service_demo_py` | Python | 服务通信（Greeting 请求/响应） |
| `service_demo_cpp` | C++ | 服务通信（Greeting 请求/响应） |
| `action_demo_py` | Python | 动作通信（DoDishes 洗碗任务） |
| `action_demo_cpp` | C++ | 动作通信（DoDishes 洗碗任务） |

### 接口包

| 包名 | 说明 |
|------|------|
| `topic_demo_interfaces` | `msg/Gps`（state, x, y） |
| `service_demo_interfaces` | `srv/Greeting`（name, age → feedback） |
| `action_demo_interfaces` | `action/DoDishes`（dishwasher_id → total_dishes_cleaned, percent_complete） |
| `msgs_demo_interfaces` | ROS 1 迁移的综合消息接口（24 个 msg/srv/action） |
| `sensor_interfaces` | `msg/SensorData`（temperature, humidity, pressure, device_id） |

### 教学支持包

| 包名 | 类型 | 说明 |
|------|------|------|
| `course_lab_interfaces` | C++ | 课程实验共享接口：`srv/ValidateRecipe`、`srv/ReadLabel`、`action/Pipeline`、`msg/MarkerPose` |
| `course_lab_utils` | Python | 课程实验共享实现：MoveIt 演示、视觉检测、AR 抓取、机械臂控制（lab_code ch17-ch21 依赖） |

### 其他示例

| 包名 | 语言 | 说明 |
|------|------|------|
| `param_demo_py` | Python | 参数声明、修改、删除、回调 |
| `param_demo_cpp` | C++ | 参数系统 |
| `tf_demo_py` | Python | TF2 广播、监听、坐标变换 |
| `tf_demo_cpp` | C++ | TF2 广播、监听、四元数转换 |
| `name_demo_cpp` | C++ | 命名空间、节点名、参数 |
| `urdf_demo_ros2` | Python | URDF/Xacro 建模演示：mybot 模型、四种 RViz 显示 launch |

### 教学实验包（lab_code，21 章）

**ROS 2 基础与机器人仿真（ch01-ch21，标准 ROS 2 包）**

| 章节 | 包名 | 说明 |
|------|------|------|
| Ch01 | `lifecycle_demo` | 生命周期节点、`/cmd_vel` 发布 |
| Ch02 | `hello_pkg` | 节点创建、日志分级 |
| Ch03 | `topic_demo` | 话题发布/订阅/QoS/正方形轨迹 |
| Ch03 | `sensor_pub` | 自定义 SensorData 消息发布 |
| Ch03 | `sensor_interfaces` | SensorData.msg 定义 |
| Ch04 | `service_demo` | AddTwoInts 服务 |
| Ch05 | `action_demo` | DoDishes 动作（异步执行） |
| Ch06 | `param_demo` | 参数声明/Launch 配置 |
| Ch07 | `tf_demo` | TF2 广播/监听 |
| Ch08 | `urdf_demo` | URDF/Xacro 建模 + RViz |
| Ch09 | `sim_demo` | Gazebo 仿真启动（委托 `robot_sim_demo`） |
| Ch10 | `slam_lab` | SLAM/Cartographer/AMCL |
| Ch11 | `navigation_lab` | Nav2 导航/航点/恢复 |
| Ch12 | `realsense_lab` | RealSense 深度相机启动 |
| Ch13 | `slam_bringup_lab` | SLAM 一键建图（`robot_sim_demo` + slam_toolbox） |
| Ch14 | `nav_bringup_lab` | Nav2 一键导航（`robot_sim_demo` + Nav2） |
| Ch15 | `arm_joint_pub_lab` | xArm 关节状态发布 |
| Ch17 | `moveit_fk_ik_lab` | MoveIt FK/IK 规划（MoveItPy） |
| Ch18 | `moveit_pick_place_lab` | MoveIt 抓取/避障/附着演示 |
| Ch19 | `vision_detection_lab` | 相机/cv_bridge/颜色/AR 码检测 |
| Ch21 | `vision_pickup_lab` | 视觉引导抓取（AR + xArm） |

Ch16、Ch20 为占位章节（仅 README）。

lab_code 详细说明见 [lab_code/README.md](src/lab_code/README.md)。

## 快速开始

### 1. 启动仿真

```bash

ros2 launch robot_sim_demo gazebo2.launch.py
```

Gazebo 打开后，TurtleBot3 Burger 机器人在 ISCAS Museum 场景中心自动巡航。

### 2. 检查话题

```bash
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

### 3. 运行 SLAM 建图

```bash
# 终端 1
ros2 launch robot_sim_demo gazebo2.launch.py gui:=false rviz:=false drive:=false

# 终端 2
ros2 launch slam_sim_demo_ros2 slam_demo.launch.py use_gazebo:=false use_rviz:=true

# 终端 3
ros2 run slam_sim_demo_ros2 slam_map_runner --ros-args -p use_sim_time:=true
```

### 4. 运行 Nav2 导航

```bash
# 终端 1
ros2 launch robot_sim_demo gazebo2.launch.py gui:=false rviz:=false drive:=false

# 终端 2
ros2 launch navigation_sim_demo_ros2 nav2_demo.launch.py use_gazebo:=false use_rviz:=true

# 终端 3
ros2 run navigation_sim_demo_ros2 nav_goal_runner --ros-args -p use_sim_time:=true -p goal_x:=1.0 -p goal_y:=0.0
```

## 测试

仓库级测试命令会覆盖核心包和实验包；`course_lab_utils` 的部分测试需 `moveit_py`：

```bash

python3 -m pytest -q
# 当前无 ROS 2 的静态测试环境：189 passed, 43 subtests passed

# 核心与示例包
for pkg in action_demo_py topic_demo_py service_demo_py param_demo_py tf_demo_py \

  tf_follower_ros2 navigation_sim_demo_ros2 slam_sim_demo_ros2 robot_sim_demo \

  urdf_demo_ros2 xarm; do

  echo "=== $pkg ==="

  (cd src/$pkg && python3 -m pytest test/ -q)

done

# lab_code 各章（详见 src/lab_code/README.md）
for pkg in ch01_lab ch02_lab/hello_pkg ch03_lab/sensor_pub ch03_lab/topic_demo \

  ch04_lab/service_demo ch05_lab/action_demo ch06_lab/param_demo ch07_lab/tf_demo \

  ch08_lab/urdf_demo ch09_lab/sim_demo ch10_lab/slam_lab ch11_lab/navigation_lab \

  ch12_lab/realsense_lab ch13_lab/slam_bringup_lab ch14_lab/nav_bringup_lab \

  ch15_lab/arm_joint_pub_lab ch17_lab/moveit_fk_ik_lab ch18_lab/moveit_pick_place_lab \

  ch19_lab/vision_detection_lab ch21_lab/vision_pickup_lab; do

  echo "=== $pkg ==="

  (cd src/lab_code/$pkg && python3 -m pytest test/ -q)

done
```

### 仿真核心与支持包

| 包名 | 测试数 | 结果 |
|------|--------|------|
| `robot_sim_demo` | 8 | 全通过 |
| `xarm_ros2_arm_only` | 9 | 全通过 |
| `tf_follower_ros2` | 7 | 全通过 |
| `slam_sim_demo_ros2` | 6 | 全通过 |
| `navigation_sim_demo_ros2` | 5 | 全通过 |
| `urdf_demo_ros2` | 1 | 通过 |

### 示例包

| 包名 | 测试数 | 结果 |
|------|--------|------|
| `action_demo_py` | 1 | 通过 |
| `topic_demo_py` | 1 | 通过 |
| `service_demo_py` | 1 | 通过 |
| `param_demo_py` | 1 | 通过 |
| `tf_demo_py` | 1 | 通过 |

### lab_code 教学实验包

| 包名 | 测试数 | 结果 |
|------|--------|------|
| `lifecycle_demo` | 1 | 通过 |
| `hello_pkg` | 2 | 通过 |
| `sensor_pub` | 1 | 通过 |
| `topic_demo` | 2 | 通过 |
| `service_demo` | 2 | 通过 |
| `action_demo` | 3 | 通过 |
| `param_demo` | 5 | 通过 |
| `tf_demo` | 4 | 通过 |
| `urdf_demo` | 5 | 通过 |
| `sim_demo` | 4 | 通过 |
| `slam_lab` | 5 | 通过 |
| `navigation_lab` | 6 | 通过 |
| `realsense_lab` | 3 | 通过 |
| `slam_bringup_lab` | 5 | 通过 |
| `nav_bringup_lab` | 4 | 通过 |
| `arm_joint_pub_lab` | 5 | 通过 |
| `moveit_fk_ik_lab` | 6 | 通过 |
| `moveit_pick_place_lab` | 3 | 通过 |
| `vision_detection_lab` | 4 | 通过 |
| `vision_pickup_lab` | 5 | 通过 |

运行截图和预期现象见各章节实验手册；当前源码目录不包含固定的 Nav2 GIF 资源。
