# 第 0 章：COM260 公共双端环境

本版第一、二章已在 COM260 完成实测；第二章新增的里程计速度输出也已通过板端构建、已知输入验证及 Gazebo 静止／运动／停止联动补验。具体环境、结果与素材见[实际运行证据](runtime_evidence.md)。课程实现使用 C++17／rclcpp，仿真使用 TurtleBot3 Burger 与 Museum。原版及 Pico 版保留。

## 设备与职责

| 执行端 | 本轮已核对的环境 | 职责 |
|---|---|---|
| 本机访问端 | macOS | 编辑、同步、Remote-SSH 和素材归档 |
| COM260 板端 | SpacemiT K3 Com260 IFX；Bianbu 4.0.6；riscv64；用户 `com260` | ROS 2 Humble 课程节点、C++ 构建和调试 |
| x86 宿主 | Ubuntu 24.04.4；用户 `duomaomao`；rootless Podman | 图形会话和课程容器 |
| x86 课程容器 | Ubuntu 22.04.5／ROS 2 Humble；Gazebo Harmonic、RViz | 仿真、传感器桥和图形界面 |

`com260`、`duomaomao` 是本机已有的 SSH 别名，读者需使用自己的地址与账号。下文 `~` 始终表示命令执行端的主目录。

【本机访问端】

```bash
ssh com260
ssh duomaomao
```

两台设备分别使用 `~/ROS2_RISCV_COM260` 接收课程文件。板端构建目录是 `~/ros2_course_com260_ws`，x86 构建目录是 `~/ros2_course_com260_humble_ws`。首次使用前检查目录用途，保留已有工作。

## 第一章源码与同步

第一章原样复用仓库中的 `src_k3_pico_itx/lifecycle_demo_cpp` 和 `src_k3_pico_itx/robot_sim_demo`。目录名表示源码来源，不表示 COM260 已通过验证。安装器只指定这两个包，避免递归发现原版与其他版本中的同名包。

本机仓库是唯一编辑源；需要平台修正时在 COM260 独立目录适配。同步前检查 itemized dry-run 清单，同步后核对源文件 SHA-256。

COM260 尚无 rsync 时，先传送独立安装入口。以下命令在目标文件尚不存在时使用：

【本机访问端，仓库根目录】

```bash
ssh com260 'mkdir -p ~/ROS2_RISCV_COM260; test ! -e ~/ROS2_RISCV_COM260/setup_course_k3_com260_kit.sh' && \
scp setup_course_k3_com260_kit.sh com260:ROS2_RISCV_COM260/
```

## COM260 安装、构建与环境加载

【COM260 板端】

```bash
cd ~/ROS2_RISCV_COM260
bash setup_course_k3_com260_kit.sh --dry-run
bash setup_course_k3_com260_kit.sh --install-deps
```

`--dry-run` 只解析依赖；`--install-deps` 调用 Bianbu 已配置软件源并交互请求 sudo。第一章不安装后续 SLAM、导航和机械臂组件，也不在板端安装 Gazebo/RViz。安装成功后同步源码及支持文件：

【本机访问端，仓库根目录】

```bash
rsync -anvi --relative --exclude=__pycache__ --exclude='*.pyc' \
  setup_course_k3_com260_kit.sh \
  src_k3_pico_itx/lifecycle_demo_cpp src_k3_pico_itx/robot_sim_demo \
  course_support/k3_com260_kit com260:ROS2_RISCV_COM260/
# 检查清单后执行同一范围的同步。
rsync -avi --relative --exclude=__pycache__ --exclude='*.pyc' \
  setup_course_k3_com260_kit.sh \
  src_k3_pico_itx/lifecycle_demo_cpp src_k3_pico_itx/robot_sim_demo \
  course_support/k3_com260_kit com260:ROS2_RISCV_COM260/
```

【COM260 板端】

```bash
cd ~/ROS2_RISCV_COM260
bash setup_course_k3_com260_kit.sh --build
source ~/.config/ros2-course-com260/env.bash
```

Bianbu 4.0.6 的 `python3-colcon-core` 提供模块入口，但不自带单独的 `colcon` 命令。本版板端使用等价的 `python3 -m colcon`，建包练习也使用这一入口。

安装器遇到主机、系统版本、依赖或源码不符合要求时，会说明原因并非零退出。两端工作区沿用首次建立时的 `.com260-ch01-workspace` 标记，后续章节继续使用；它标识课程创建的受管工作区，不表示只能构建第一章。

构建入口使用 Debug 配置并生成编译数据库，随后执行测试。构建成功后才生成环境文件与 GDB 包装入口；每个实验终端分别加载环境。板端构建结果以证据索引为准。

## 第二章源码与构建

第二章使用独立的 `src_k3_com260_kit/hello_pkg_cpp` 和 `name_demo_cpp`，仿真继续复用第一章的 `robot_sim_demo`。先完成第一章环境，然后同步本章源码；不会覆盖第一章练习目录。

【本机访问端，仓库根目录】

```bash
rsync -anvi --relative setup_course_k3_com260_kit.sh src_k3_com260_kit/hello_pkg_cpp src_k3_com260_kit/name_demo_cpp com260:ROS2_RISCV_COM260/
# 核对清单后执行。
rsync -avi --relative setup_course_k3_com260_kit.sh src_k3_com260_kit/hello_pkg_cpp src_k3_com260_kit/name_demo_cpp com260:ROS2_RISCV_COM260/
```

【COM260 板端】

```bash
cd ~/ROS2_RISCV_COM260
bash setup_course_k3_com260_kit.sh --build-ch02
source ~/.config/ros2-course-com260/env.bash
```

该入口只构建两个第二章包，不重复统计第一章测试。第二章验收通过实际节点、日志过滤、传感器订阅与仿真运动完成。独立练习另用 `~/my_hello_com260_ws`；教案示例分别使用 `~/my_robot_com260_ws` 和 `~/my_scan_com260_ws`。

下方 x86 启动方式相同，第二章将运行编号后缀改为 `com260-ch02`，整章使用同一仿真实例。在容器的独立终端先加载 ROS 环境，再运行 `rqt_graph` 或 `ros2 run rqt_console rqt_console`；本轮基础镜像已安装这两个工具。

## x86 独立构建与仿真

本轮复用宿主已安装的 `localhost/ros2-course-humble:gazebo` 镜像，来源为原课程的 `course_support/k3/containers/`。需要新建镜像的设备先完成镜像准备；以下命令不负责安装宿主或容器。

【本机访问端，仓库根目录】

```bash
rsync -anvi --relative --exclude=__pycache__ --exclude='*.pyc' \
  src_k3_pico_itx/robot_sim_demo course_support/k3_com260_kit \
  duomaomao:ROS2_RISCV_COM260/
# 检查清单后执行同步。
rsync -avi --relative --exclude=__pycache__ --exclude='*.pyc' \
  src_k3_pico_itx/robot_sim_demo course_support/k3_com260_kit \
  duomaomao:ROS2_RISCV_COM260/
ssh duomaomao 'bash ~/ROS2_RISCV_COM260/course_support/k3_com260_kit/scripts/x86-gazebo.bash build'
```

【x86 宿主】

```bash
run_id=$(date -u +%Y%m%dT%H%M%SZ)-com260-ch01
bash ~/ROS2_RISCV_COM260/course_support/k3_com260_kit/scripts/x86-gazebo.bash start "$run_id"
# 需要独立观察终端时，使用同一个 run_id。
podman exec -it "ros2-com260-$run_id" bash
```

【x86 课程容器，观察终端】

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 node list --no-daemon --spin-time 10
ros2 topic echo /clock --once
```

脚本使用当前宿主的 GDM X11 `:1` 会话和 rootless Podman，只启动本轮命名的容器。找不到会话或无法访问时会给出错误提示；先登录 x86 桌面，核对 `/tmp/.X11-unix/X1` 和 `/run/user/$(id -u)/gdm/Xauthority`，图形会话不同需先按实际路径调整脚本。每轮容器最长运行一小时，退出后自动移除；`stop` 对已不存在的实例报告 `X86_GAZEBO_ALREADY_ABSENT` 并成功返回，标签不匹配或停止失败仍报错。验证结束后退出容器终端，在启动它的 x86 终端停止同一实例：

```bash
bash ~/ROS2_RISCV_COM260/course_support/k3_com260_kit/scripts/x86-gazebo.bash stop "$run_id"
```

## DDS 与 IDE

双端目标环境为 Humble／CycloneDDS，`ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`。运行前检查课程域是否被其他任务使用；同域收发、异域隔离与恢复均需实测。宿主的原生 Jazzy 环境不加入课程仿真。

在 Mac 使用 RuyiSDK VSCode 插件和 Remote-SSH 连接 `com260`，打开 `/home/com260/ROS2_RISCV_COM260`。C++ 补全使用 clangd，断点调试使用 Native Debug 与板端 GDB。独立配置模板为 `course_support/k3_com260_kit/ide/launch.json`，GDB 包装入口由板端构建步骤安装至 `~/.config/ros2-course-com260/gdb.bash`。已有 `.vscode` 配置先核对再合并。

本轮跨机查询使用 `ros2 node list --no-daemon --spin-time 10` 和 `ros2 param get --no-daemon --spin-time 10 <节点> <参数>`。短发现窗口曾漏报远端节点；新启动的订阅者也应等待发现，跨域恢复观察窗口使用 20 秒。

首次配置远程课程目录时，先检查 `.vscode/` 是否已有个人配置；不存在对应文件时再复制模板，已有文件只合并本课程配置：

【COM260 板端】

```bash
cd ~/ROS2_RISCV_COM260
mkdir -p .vscode
cp -n course_support/k3_com260_kit/ide/launch.json .vscode/launch.json
cp -n course_support/k3_com260_kit/ide/settings.json .vscode/settings.json
```

clangd 的编译数据库链接与具体调试步骤见[练习 1.6](ch01_lab.md#练习-16vs-code--ros-2-插件编程约-15-分钟)。本轮 Remote-SSH、clangd 语义补全、GDB 断点及 Watch 0→1 均已实测。RuyiSDK VSCode 扩展版本为 0.1.6；本轮使用板端原生 GCC/GDB，未安装 RuyiSDK CLI，界面中的 `<No RuyiSDK>` 对应这一实际状态。
