# 第 0 章：COM260 公共双端环境

COM260 运行 C++17／rclcpp 课程节点，x86 运行 Gazebo 和 RViz。按所学章节的工作区与 DDS 配置启动两端，再打开对应的教案或实验手册。各章的运行结果和素材见[运行证据](runtime_evidence.md)。

## 设备与职责

| 执行端 | 课程环境 | 职责 |
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

<a id="第一章源码与同步"></a>
<a id="ch01"></a>

## 第 1 章 源码与同步

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

<a id="ch02"></a>

## 第 2 章 源码与构建

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

<a id="ch03"></a>

## 第 3 章 源码与构建

第三章在 COM260 独立目录提供 `topic_demo_cpp`、`topic_demo_interfaces`、`sensor_interfaces` 和 `sensor_pub_cpp`。先完成上述公共依赖；第三章额外需要 ROSIDL 消息生成器与运行库，更新后的安装器已包含这两项依赖。

【本机访问端，仓库根目录】

```bash
rsync -anvi --relative setup_course_k3_com260_kit.sh \
  src_k3_com260_kit/topic_demo_cpp src_k3_com260_kit/topic_demo_interfaces \
  src_k3_com260_kit/sensor_interfaces src_k3_com260_kit/sensor_pub_cpp \
  com260:ROS2_RISCV_COM260/
# 核对清单后执行。
rsync -avi --relative setup_course_k3_com260_kit.sh \
  src_k3_com260_kit/topic_demo_cpp src_k3_com260_kit/topic_demo_interfaces \
  src_k3_com260_kit/sensor_interfaces src_k3_com260_kit/sensor_pub_cpp \
  com260:ROS2_RISCV_COM260/
```

【COM260 板端】

```bash
cd ~/ROS2_RISCV_COM260
bash setup_course_k3_com260_kit.sh --dry-run
# 尚未具备 ROSIDL 依赖时执行 --install-deps。
bash setup_course_k3_com260_kit.sh --build-ch03
source ~/.config/ros2-course-com260/env.bash
```

本入口只构建第三章四个包。`sensor_interfaces` 注册一个 pytest 用例；colcon 汇总中的两条结果包含该用例及其 CTest 包装，不能计为两个独立测试，也不计入旧章节测试。独立建包练习使用 `~/my_topics_com260_ws`，按[第三章实验](ch03_lab.md)逐步创建。

x86 按下一节同步 `robot_sim_demo` 与课程支持目录并构建仿真。本章使用 `start-ch03 RUN_ID` 启动，加载 `ch03_square.rviz`：Fixed Frame 为 `odom`，Odometry 保留运动历史。第一章的 `museum.rviz` 继续保留。GUI 观察终端加载容器环境后运行 `rqt_graph`；选择 `Nodes/Topics (all)`，取消 `Hide: Leaf topics`，点击刷新，可看到 GPS 发布／订阅及 SensorData 话题。CLI 观察节点在 `Hide: Debug` 勾选时会隐藏。

<a id="ch04"></a>
## 第 4 章 源码与构建

第四章新增七个独立包：`service_demo_interfaces`、`service_demo_cpp`、`service_demo_lab_cpp`、`weather_interfaces`、`weather_srv`、`speed_interfaces`、`speed_control`。先完成第一章环境；板端节点使用 C++17，独立建包练习使用 `~/my_services_com260_ws`。

【本机访问端，仓库根目录】对两端分别核对同步清单并同步：

```bash
for host in com260 duomaomao; do
  rsync -anvi --relative setup_course_k3_com260_kit.sh \
    src_k3_com260_kit/service_demo_interfaces src_k3_com260_kit/service_demo_cpp \
    src_k3_com260_kit/service_demo_lab_cpp src_k3_com260_kit/weather_interfaces \
    src_k3_com260_kit/weather_srv src_k3_com260_kit/speed_interfaces \
    src_k3_com260_kit/speed_control course_support/k3_com260_kit \
    "$host:ROS2_RISCV_COM260/"
done
# 核对后将同一命令的 -anvi 改成 -avi 执行。
```

【COM260 板端】

```bash
cd ~/ROS2_RISCV_COM260
bash setup_course_k3_com260_kit.sh --dry-run
# 缺少 example_interfaces 或 ROSIDL 依赖时先执行 --install-deps。
bash setup_course_k3_com260_kit.sh --build-ch04
source ~/.config/ros2-course-com260/env.bash
```

本入口仅构建上述七包；这些包未注册单元测试，本章通过真实服务调用、失败分支及仿真反馈验收，不引用旧章节测试总数。

【x86 主机】先按下一节构建共享仿真，再在同一 Humble 工作区构建第四章服务包，用于双向跨机请求：

```bash
podman run --rm --network none --cap-drop all --security-opt no-new-privileges \
  --volume "$HOME/ROS2_RISCV_COM260/src_k3_com260_kit:/course-src:ro" \
  --volume "$HOME/ros2_course_com260_humble_ws:/workspace:rw" --workdir /workspace \
  localhost/ros2-course-humble:gazebo bash --noprofile --norc -c '
    set -eo pipefail
    pkgs=(service_demo_interfaces service_demo_cpp service_demo_lab_cpp weather_interfaces weather_srv speed_interfaces speed_control)
    sources=(); for package in "${pkgs[@]}"; do sources+=("/course-src/$package"); done
    python3 -m colcon build --base-paths "${sources[@]}" --packages-select "${pkgs[@]}" \
      --executor sequential --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  '
```

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

脚本可选透传当前终端的 `CYCLONEDDS_URI`（下节仅使用内联 XML；宿主文件路径不会自动映射入容器）。脚本使用当前宿主的 GDM X11 `:1` 会话和 rootless Podman，只启动本轮命名的容器。找不到会话或无法访问时会给出错误提示；先登录 x86 桌面，核对 `/tmp/.X11-unix/X1` 和 `/run/user/$(id -u)/gdm/Xauthority`，图形会话不同需先按实际路径调整脚本。每轮容器最长运行一小时，退出后自动移除；`stop` 对已不存在的实例报告 `X86_GAZEBO_ALREADY_ABSENT` 并成功返回，标签不匹配或停止失败仍报错。验证结束后退出容器终端，在启动它的 x86 终端停止同一实例：

```bash
bash ~/ROS2_RISCV_COM260/course_support/k3_com260_kit/scripts/x86-gazebo.bash stop "$run_id"
```

## DDS 与 IDE

<a id="dds-unicast"></a>
### 无线网络发现失败时的单播复核

如果两机 SSH 可达、容器内部已有 `/odom`，但 COM260 的 `ros2 node list --no-daemon --spin-time 10` 看不到仿真节点，先核对 Humble、RMW、Domain 和网络。需要复核自动发现时，可以仅为课程进程指定对端地址；不要据此关闭防火墙或改动系统网络。

【COM260 每个课程终端】先加载 `env.bash`，再将 `DDS_PEER` 设为 x86 实际地址；【x86 启动仿真的终端】将其设为 COM260 实际地址。下面地址仅对应本轮局域网：

```bash
# COM260 终端：
DDS_PEER=192.168.1.4
# x86 终端改用：DDS_PEER=192.168.1.88
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address=\"127.0.0.1\"/><Peer Address=\"$DDS_PEER\"/></Peers></Discovery></Domain></CycloneDDS>"
```

两端都设置后，在 x86 停止原 RUN_ID 并启动新一轮。`x86-gazebo.bash` 将该可选环境变量传入容器；容器内的观察终端会继承它。每个 COM260 课程终端都需要相同配置，`env.bash` 本身不写入设备地址。只需在当前终端 `unset CYCLONEDDS_URI` 即可恢复默认；已启动节点需重新启动才会读取变化。

```bash
# COM260；避免沿用先前配置启动的 ROS CLI daemon。
ros2 daemon stop
ros2 node list --no-daemon --spin-time 10
ros2 topic echo /odom nav_msgs/msg/Odometry --once --no-daemon --spin-time 10
```


<a id="dds-wired"></a>
### 多网卡时选择有线路径

单播配置用于解决发现问题，不能消除运动指令与 `/clock` 的传输延迟。方形运动属于开环实验；若速度、转弯时长正确而轨迹仍明显失真，先检查网络。实测无线链路的 25 次 ping 平均约 266 ms、峰值 743 ms，方形轨迹未达到验收范围；有线连接平均约 0.33 ms、峰值 0.39 ms，轨迹验收通过。

在两端执行 `ip -4 route get 对端地址` 核对接口和源地址，再用 `ping -c 25 对端地址` 观察延迟。本轮 COM260 的 `end1` 为 `192.168.1.88`，x86 有线接口为 `192.168.1.172`；地址属于本轮实际配置，下次仍须核对，不视为固定地址。

同时保留 Wi-Fi 时，在当前课程终端显式选择用于 DDS 的本机地址：

```bash
# COM260：先 source ~/.config/ros2-course-com260/env.bash
DDS_LOCAL=192.168.1.88
DDS_PEER=192.168.1.172
# x86 启动仿真的终端改用：DDS_LOCAL=192.168.1.172; DDS_PEER=192.168.1.88
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"$DDS_LOCAL\"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address=\"127.0.0.1\"/><Peer Address=\"$DDS_PEER\"/></Peers></Discovery></Domain></CycloneDDS>"
```

两个地址都必须替换为实际值；所有 COM260 课程终端使用相同配置，x86 停止旧 RUN_ID 后用新 RUN_ID 启动仿真。停止先前 ROS CLI daemon，再查询话题。此配置只作用于本轮进程，不修改系统网络、SSH 别名或持久环境文件。网络条件变化后，应重新验证轨迹和停止反馈，不沿用历史数值。


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

<a id="ch05"></a>
## 第 5 章 动作通信环境

第五章节点在 COM260 执行，Gazebo 在 x86；先完成第一章公共环境。十个独立包如下，两个洗碗接口的字段不同，不混入原版同名包。

【本机访问端，仓库根目录】先核对同步清单：

```bash
pkgs=(action_demo_interfaces action_demo_cpp action_demo_lab_interfaces action_demo_lab_cpp dishes_action_interfaces dishes_action_lab tracking_interfaces tracking_server pose_nav_interfaces pose_nav_action)
sources=(); for package in "${pkgs[@]}"; do sources+=("src_k3_com260_kit/$package"); done
rsync -anvi --relative "${sources[@]}" setup_course_k3_com260_kit.sh \
  course_support/k3_com260_kit com260:ROS2_RISCV_COM260/
# 核对后将 -anvi 改为 -avi 执行。
```

【COM260】

```bash
cd ~/ROS2_RISCV_COM260
bash setup_course_k3_com260_kit.sh --dry-run
# 确认缺失依赖后再运行 --install-deps。
bash setup_course_k3_com260_kit.sh --build-ch05
source ~/.config/ros2-course-com260/env.bash
```

入口显式构建上述十包；动作验收包括反馈、结果、取消、忙碌拒绝与异常停止，不以编译成功替代运行成功。Gazebo 的 `server_gazebo` 与基础计算 `server` 使用相同 `/tracking` 名称，两者只能择一启动。学生建包步骤见[第五章实验](ch05_lab.md)。

<a id="ch06"></a>
## 第 6 章 参数与 Launch 环境

本章的 `param_demo_cpp` 包含 C++ 参数生命周期、参数验证、速度控制和巡航节点。Python 文件仅组织 Launch。COM260 不安装图形界面或 Nav2；含 RViz、Gazebo、Nav2 的 Launch 在 x86 独立容器中运行。

【本机访问端，仓库根目录】

```bash
for host in com260 duomaomao; do
  rsync -anvi --relative src_k3_com260_kit/param_demo_cpp \
    setup_course_k3_com260_kit.sh course_support/k3_com260_kit \
    "$host:ROS2_RISCV_COM260/"
done
# 核对后将 -anvi 改为 -avi 执行。
```

【COM260】

```bash
cd ~/ROS2_RISCV_COM260
bash setup_course_k3_com260_kit.sh --build-ch06
source ~/.config/ros2-course-com260/env.bash
ros2 run param_demo_cpp param_demo
```

核心例程循环三次，在第二轮删除 param5。详细参数 CRUD、YAML 与速度节点见[第六章实验](ch06_lab.md)。

【x86】在已完成第一章共享仿真构建的基础上，增加本章独立 Nav2 镜像：

```bash
cd ~/ROS2_RISCV_COM260
CH06=course_support/k3_com260_kit/scripts/x86-ch06.bash
bash "$CH06" build-image
bash "$CH06" build
RUN_ID=$(date -u +%Y%m%dT%H%M%SZ)-ch06-demo
bash "$CH06" start "$RUN_ID" demo.launch.py use_rviz:=false
bash "$CH06" exec "$RUN_ID" ros2 node list
bash "$CH06" stop "$RUN_ID"
```

镜像 `localhost/ros2-course-com260:ch06-nav2` 基于原有 Humble/Gazebo 镜像派生，安装 Humble Navigation2 与 Nav2 bringup；原镜像保留。本章脚本复用独立 Humble 工作区，只构建 `param_demo_cpp`。`start` 仅接受本章四个 Launch 入口，`exec` 在对应容器内加载 Humble 工作区后执行查询，`stop` 核对 RUN_ID 标签后停止容器。图形实验期间保持 x86 桌面会话，不要注销；注销会使窗口与 rootless 容器退出。

`CYCLONEDDS_URI` 的接口绑定不能代替系统路由检查。即使两端已接有线，仍应确认 `ip -4 route get 对端地址` 的接口和源地址都是有线；若回程仍走 Wi-Fi，先调整课程网络，再复验。不要为通过导航验收而放宽 1 秒里程计超时。

组合启动会同时创建多个 DDS participant。上面的 CycloneDDS 配置将自动分配索引上限设为 100；两端应使用同一设置。若已有自定义配置，也需合并此项并重启相关节点，否则可能出现 `Failed to find a free participant index`，导致部分 Nav2 节点启动失败。第六章脚本未收到自定义配置时采用相同上限和默认网络发现；跨机有线实验仍需显式设置接口和对端地址。

<a id="ch07-ch08"></a>
## 第 7～8 章 TF 与建模环境

第七章业务节点使用 C++17；第八章为 XML/Xacro、配置和 Python Launch，无需把模型文件转换成 C++。COM260 运行 TF 广播、监听及查询；x86 运行 Xacro、RViz、关节 GUI 和 Gazebo。两章教学模型与 Gazebo 分轮运行，不能同时向同一 DDS 域发布同名机器人 frame。

【本机访问端，仓库根目录】先预览两端同步清单，核对后将 `-anvi` 改为 `-avi`：

```bash
for host in com260 duomaomao; do
  rsync -anvi --relative src_k3_com260_kit/tf_demo_cpp \
    src_k3_com260_kit/tf_demo_lab_cpp src_k3_com260_kit/urdf_demo \
    src_k3_com260_kit/urdf_demo_ros2 setup_course_k3_com260_kit.sh \
    course_support/k3_com260_kit "$host:ROS2_RISCV_COM260/"
done
```

【COM260】

```bash
cd ~/ROS2_RISCV_COM260
bash setup_course_k3_com260_kit.sh --build-ch07
source ~/.config/ros2-course-com260/env.bash
ros2 run tf_demo_lab_cpp tf_broadcaster
```

确认广播器启动后按 `Ctrl+C` 停止，再继续 x86 模型入口，避免同时发布相同的 `base_link`。

TF 包依赖 rclcpp、geometry_msgs、tf2、tf2_ros、tf2_geometry_msgs、tf2_tools。第八章建模和 GUI 依赖只在 x86 容器准备。`--build-ch08` 仅安装模型资源，不表示板端已安装图形程序。

【x86】复用第六章镜像，并派生独立的第七、八章镜像：

```bash
cd ~/ROS2_RISCV_COM260
CH08=course_support/k3_com260_kit/scripts/x86-ch07-ch08.bash
bash "$CH08" build-image
bash "$CH08" build
RUN_ID=$(date -u +%Y%m%dT%H%M%SZ)-ch08
bash "$CH08" start "$RUN_ID" urdf_demo display.launch.py
bash "$CH08" exec "$RUN_ID" ros2 topic list
bash "$CH08" stop "$RUN_ID"
```

第八章独立建包需可写且持久保存的练习目录。以下命令在 x86 主机启动带桌面访问的练习容器；已有目录先核对内容，所有编辑和生成物保存在主机的同名练习目录。进入后执行手册的建包步骤：

```bash
mkdir -p ~/my_urdf_com260_ws
podman run --rm -it --network host --cap-drop all --security-opt no-new-privileges \
  --env DISPLAY=:1 --env XAUTHORITY=/tmp/course.xauth --env QT_X11_NO_MITSHM=1 \
  --env LIBGL_ALWAYS_SOFTWARE=1 --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --volume "/run/user/$(id -u)/gdm/Xauthority:/tmp/course.xauth:ro" \
  --volume "$HOME/my_urdf_com260_ws:/root/my_urdf_com260_ws:rw" \
  --volume "$HOME/ROS2_RISCV_COM260/src_k3_com260_kit:/course-src:ro" \
  localhost/ros2-course-com260:ch07-ch08 bash
```

原 XML/Xacro 对照源码在容器 `/course-src/urdf_demo/`。完成后退出容器；再次使用相同挂载可继续之前的练习。公共图形会话要求和有线 DDS 设置沿用前文。

## 第 9～15 章 Gazebo 与 SLAM 环境

[实验 9 的双端准备](ch09_lab.md#双端准备)包含 Gazebo 镜像和工作区配置。在此基础上，按[实验 10](ch10_lab.md)开头的命令启动 SLAM 容器。

<a id="nav2-prerequisites"></a>

## Nav2 与视觉示例的基础依赖

COM260 使用 ROS 2 Humble、C++ 编译器、colcon、`libyaml-cpp-dev`，以及实验 10 安装到 `~/ros2_course_com260_ch09_ch10_deps_ws` 的 `nav2_msgs` 接口。

x86 使用 rootless Podman 和 X11 桌面 `:1`。缺少第 6 章 Nav2 基础镜像时，在仓库根目录执行：

```bash
bash course_support/k3_com260_kit/scripts/x86-ch06.bash build-image
```

将仓库放到两台设备上本章命令所示的位置。两端使用同一个 DDS 域，并将命令中的 IP 替换为实际通信地址。COM260 的构建命令将 DDS 设置写入本章环境文件。x86 在设置好 DDS 的同一终端中启动仿真；另开终端时需重新执行本章的两个 `export` 命令。

<a id="ch16-ch17"></a>
<a id="ch16"></a>

## 第 16 章 Nav2 架构与核心组件环境

首次配置请先准备[基础依赖](#nav2-prerequisites)。

板端同时构建 [navigation_lab_cpp](../src_k3_com260_kit/navigation_lab_cpp/README.md)，用于 [Nav2 自主导航实验](ch11_lab.md)。

<a id="ch16-ch17-packages"></a>
<a id="ch16-packages"></a>

| 示例包 | 可执行文件 |
|---|---|
| [custom_nav_component](../src_k3_com260_kit/custom_nav_component/README.md) | `custom_nav_component` |
| [navigation_client_cpp](../src_k3_com260_kit/navigation_client_cpp/README.md) | `bt_navigator_client`、`multi_goal_navigation` |
| [global_planner_client](../src_k3_com260_kit/global_planner_client/README.md) | `global_planner_client` |

**COM260**

在 COM260 构建本章示例。

```bash
cd ~/ROS2_RISCV_COM260_CH16_CH17
export ROS_DOMAIN_ID=181
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.88"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.172"/></Peers></Discovery></Domain></CycloneDDS>'
bash setup_course_k3_com260_kit.sh --build-ch16
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
```

**x86**

在 x86 构建 Nav2 仿真环境。

```bash
cd ~/ROS2_RISCV_COM260_CH16_CH17
export ROS_DOMAIN_ID=181
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.172"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.88"/></Peers></Discovery></Domain></CycloneDDS>'
bash course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash build-image
bash course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash build
```

x86 工作区为 `~/ros2_course_com260_ch16_ch17_humble_ws`。

<a id="ch17"></a>

## 第 17 章 全局代价地图环境

首次配置请先准备[基础依赖](#nav2-prerequisites)。

板端同时构建 [navigation_lab_cpp](../src_k3_com260_kit/navigation_lab_cpp/README.md)，用于 [Nav2 自主导航实验](ch11_lab.md)。

<a id="ch17-packages"></a>

| 示例包 | 可执行文件 |
|---|---|
| [costmap_demo](../src_k3_com260_kit/costmap_demo/README.md) | `costmap_demo`、`inflation_plot` |
| [map_loader](../src_k3_com260_kit/map_loader/README.md) | `map_loader` |
| [obstacle_layer_processor](../src_k3_com260_kit/obstacle_layer_processor/README.md) | `obstacle_layer_processor` |
| [costmap_debugger](../src_k3_com260_kit/costmap_debugger/README.md) | `costmap_debugger` |

**COM260**

在 COM260 构建本章示例。

```bash
cd ~/ROS2_RISCV_COM260_CH16_CH17
export ROS_DOMAIN_ID=181
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.88"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.172"/></Peers></Discovery></Domain></CycloneDDS>'
bash setup_course_k3_com260_kit.sh --build-ch17
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
```

**x86**

在 x86 构建 Nav2 仿真环境。

```bash
cd ~/ROS2_RISCV_COM260_CH16_CH17
export ROS_DOMAIN_ID=181
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.172"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.88"/></Peers></Discovery></Domain></CycloneDDS>'
bash course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash build-image
bash course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash build
```

x86 工作区为 `~/ros2_course_com260_ch16_ch17_humble_ws`。

<a id="ch18-ch19"></a>
<a id="ch18"></a>

## 第 18 章 全局路径规划环境

首次配置请先准备[基础依赖](#nav2-prerequisites)。

示例使用 `costmap_demo` 的栅格类型和 `global_planner_client`、`navigation_client_cpp` 的 Action 客户端。

| 示例包 | 用途 |
|---|---|
| `grid_path_planner` | Dijkstra/A*、SVG、真实代价地图订阅 |
| `hybrid_astar_planner` | 带朝向和足迹检查的运动基元搜索 |
| `path_optimizer` | 平滑、修剪、重采样、代价优化 |

**COM260**

在 COM260 构建本章示例并运行已注册的测试。

```bash
cd ~/ROS2_RISCV_COM260_CH18_CH19
export ROS_DOMAIN_ID=182
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.88"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.172"/></Peers></Discovery></Domain></CycloneDDS>'
bash setup_course_k3_com260_kit.sh --build-ch18
source ~/.config/ros2-course-com260-ch18-ch19/env.bash
```

在 COM260 查看测试结果：`python3 -m colcon test-result --test-result-base ~/ros2_course_com260_ch18_ch19_ws/build --verbose`。

**x86**

在 x86 构建 Museum 地图、Burger 模型和 Nav2 插件使用的仿真环境。

```bash
cd ~/ROS2_RISCV_COM260_CH18_CH19
export ROS_DOMAIN_ID=182
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.172"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.88"/></Peers></Discovery></Domain></CycloneDDS>'
bash course_support/k3_com260_kit/scripts/x86-ch18-ch19.bash build-image
bash course_support/k3_com260_kit/scripts/x86-ch18-ch19.bash build
```

x86 工作区为 `~/ros2_course_com260_ch18_ch19_humble_ws`。

<a id="ch19"></a>

## 第 19 章 局部路径规划环境

首次配置请先准备[基础依赖](#nav2-prerequisites)。

示例使用 `costmap_demo` 的栅格类型和 `global_planner_client`、`navigation_client_cpp` 的 Action 客户端。

| 示例包 | 用途 |
|---|---|
| `dwa_planner` | 动态窗口、评分、COM260 闭环控制 |
| `rpp_controller` | 纯追踪原理、速度与前瞻调参 |

**COM260**

在 COM260 构建本章示例并运行已注册的测试。

```bash
cd ~/ROS2_RISCV_COM260_CH18_CH19
export ROS_DOMAIN_ID=182
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.88"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.172"/></Peers></Discovery></Domain></CycloneDDS>'
bash setup_course_k3_com260_kit.sh --build-ch19
source ~/.config/ros2-course-com260-ch18-ch19/env.bash
```

在 COM260 查看测试结果：`python3 -m colcon test-result --test-result-base ~/ros2_course_com260_ch18_ch19_ws/build --verbose`。

**x86**

在 x86 构建 Museum 地图、Burger 模型和 Nav2 插件使用的仿真环境。

```bash
cd ~/ROS2_RISCV_COM260_CH18_CH19
export ROS_DOMAIN_ID=182
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.172"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.88"/></Peers></Discovery></Domain></CycloneDDS>'
bash course_support/k3_com260_kit/scripts/x86-ch18-ch19.bash build-image
bash course_support/k3_com260_kit/scripts/x86-ch18-ch19.bash build
```

x86 工作区为 `~/ros2_course_com260_ch18_ch19_humble_ws`。

<a id="ch20-ch21"></a>
<a id="ch20"></a>

## 第 20 章 行为树与恢复行为环境

首次配置请先准备[基础依赖](#nav2-prerequisites)。

示例包为 `behavior_tree_demo_cpp`、`recovery_demo_cpp` 和 `multi_robot_scheduler`。

**COM260**

在 COM260 安装依赖，构建本章示例并加载工作区。

```bash
cd ~/ROS2_RISCV_COM260_CH20_CH21
bash setup_course_k3_com260_kit.sh --install-deps-ch20-ch21
export ROS_DOMAIN_ID=202
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.88"/></Interfaces><AllowMulticast>false</AllowMulticast><MaxMessageSize>1400B</MaxMessageSize><FragmentSize>1280B</FragmentSize></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.4"/></Peers></Discovery></Domain></CycloneDDS>'
bash setup_course_k3_com260_kit.sh --build-ch20
source ~/.config/ros2-course-com260-ch20-ch21/env.bash
```

**x86**

x86 需要 Nav2 镜像 `localhost/ros2-course-com260:ch16-ch17-nav2`。缺少该镜像时，在当前仓库运行 `bash course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash build-image`，再构建本章的仿真环境。

```bash
cd ~/ROS2_RISCV_COM260_CH20_CH21
export ROS_DOMAIN_ID=202
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.4"/></Interfaces><AllowMulticast>false</AllowMulticast><MaxMessageSize>1400B</MaxMessageSize><FragmentSize>1280B</FragmentSize></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.88"/></Peers></Discovery></Domain></CycloneDDS>'
bash course_support/k3_com260_kit/scripts/x86-ch20-ch21.bash build-image
bash course_support/k3_com260_kit/scripts/x86-ch20-ch21.bash build
```

x86 工作区为 `~/ros2_course_com260_ch20_ch21_humble_ws`。

<a id="wired-dds"></a>

### 三车导航与视觉 SLAM 的有线通信

三车导航、RGB-D、EuRoC 与 LVI-SAM 的跨机通信共用有线 IPv6 和 DDS 域 203。在 COM260 运行 `ip -6 addr show dev end1 scope global`，在 x86 运行 `ip -6 addr show dev enx80691a14e68b scope global`。若接口名不同，请换成本机有线接口。各选一个有效、未标记为 temporary 或 deprecated 的全局地址。

在**两台设备上分别**执行以下命令，将两个示例地址替换为实际地址。第一个参数始终填本机，第二个填对端。更换网络或地址后重新执行。

```bash
cd ~/ROS2_RISCV_COM260_CH20_CH21
bash course_support/k3_com260_kit/scripts/configure-vision-dds.bash 2001:db8::1 2001:db8::2
```

COM260 每个运行这些示例的新终端先加载工作区，再加载通信配置：

```bash
source ~/.config/ros2-course-com260-ch20-ch21/env.bash
source ~/.config/ros2-course-com260-ch20-ch21/vision-dds.bash
```

在 x86 的每个运行这些示例的新终端中加载通信配置：

```bash
source ~/.config/ros2-course-com260-ch20-ch21/vision-dds.bash
```

<a id="ch21"></a>

## 第 21 章 视觉 SLAM 导论环境

首次配置请先准备[基础依赖](#nav2-prerequisites)。

本章的相机、ORB 特征、直接法、RGB-D 和 IMU 数值示例使用域 202。RGB-D 建图、EuRoC 轨迹显示和 LVI-SAM 另用域 203，配置见[有线 DDS 配置](#wired-dds)。

**COM260**

在 COM260 安装依赖，构建本章示例并加载工作区。

```bash
cd ~/ROS2_RISCV_COM260_CH20_CH21
bash setup_course_k3_com260_kit.sh --install-deps-ch20-ch21
export ROS_DOMAIN_ID=202
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.88"/></Interfaces><AllowMulticast>false</AllowMulticast><MaxMessageSize>1400B</MaxMessageSize><FragmentSize>1280B</FragmentSize></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.4"/></Peers></Discovery></Domain></CycloneDDS>'
bash setup_course_k3_com260_kit.sh --build-ch21
source ~/.config/ros2-course-com260-ch20-ch21/env.bash
```

**x86**

x86 需要 Nav2 镜像 `localhost/ros2-course-com260:ch16-ch17-nav2`。缺少该镜像时，在当前仓库运行 `bash course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash build-image`，再构建本章的仿真环境。

```bash
cd ~/ROS2_RISCV_COM260_CH20_CH21
export ROS_DOMAIN_ID=202
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.4"/></Interfaces><AllowMulticast>false</AllowMulticast><MaxMessageSize>1400B</MaxMessageSize><FragmentSize>1280B</FragmentSize></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.88"/></Peers></Discovery></Domain></CycloneDDS>'
bash course_support/k3_com260_kit/scripts/x86-ch20-ch21.bash build-image
bash course_support/k3_com260_kit/scripts/x86-ch20-ch21.bash build
```

x86 工作区为 `~/ros2_course_com260_ch20_ch21_humble_ws`。

上述配置将 UDP 负载限制为 1400 B、DDS 分片设为 1280 B；两端使用相同配置，图像保持原始分辨率。参数含义见 [CycloneDDS 配置说明](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html)。

<a id="ch21-vision"></a>

### 第 21 章 RGB-D 建图与 EuRoC 轨迹显示

在 COM260 准备依赖并构建：

```bash
cd ~/ROS2_RISCV_COM260_CH20_CH21
source ~/.config/ros2-course-com260-ch20-ch21/env.bash
bash setup_course_k3_com260_kit.sh --prepare-ch21-slam
bash setup_course_k3_com260_kit.sh --build-ch21-slam
source ~/.config/ros2-course-com260-ch20-ch21/env.bash
```

依赖准备使用固定版本的 ORB-SLAM3 与 Pangolin，下载到 `~/ros2_course_com260_ch21_slam`；Boost 等补充库解包到该目录的 `sysroot`，不替换系统库。首次原生编译需要较长时间。`visual_slam_eval` 使用 C++17，读取 rosbag 的原始图像时间戳以及两帧之间的 IMU 样本，调用 ORB-SLAM3 的视觉惯性接口。

跨机显示先完成[有线 DDS 配置](#wired-dds)。

<a id="ch21-frameworks"></a>

### 第 21 章完整 DSO 与 VINS-Mono 环境

先完成 COM260 的[第 21 章基础依赖](#ch21)。两个框架使用独立工作区 `~/ros2_course_com260_ch21_frameworks`。构建脚本下载固定版本源码，应用课程补丁，在工作区内解包依赖并编译 Ceres 2.1、DSO 和 VINS-Mono。

**COM260**　在课程仓库根目录执行。首次构建需要下载源码和依赖，保持联网，等待命令正常返回。

```bash
sudo apt-get install --no-install-recommends --no-remove --no-upgrade \
  bubblewrap curl patch python3-numpy python3-opencv
cd ~/ROS2_RISCV_COM260_CH20_CH21
bash setup_course_k3_com260_kit.sh --build-ch21-frameworks
```

如果板端下载较慢，可在电脑的课程仓库根目录执行以下命令，将同版本源码放入 COM260 的 `~/Downloads/`，构建脚本会读取缓存。括号内的目录切换仅在子 shell 中生效。

```bash
(
  mkdir -p /tmp/com260-framework-sources
  cd /tmp/com260-framework-sources
  curl -fL --retry 3 https://api.github.com/repos/dongbo19/VINS-MONO-ROS2/tarball/4c3cf08fa5d9b57f3be9bd49241915e968ee2e1e \
    -o vins-4c3cf08fa5d9b57f3be9bd49241915e968ee2e1e.tar.gz
  curl -fL --retry 3 https://api.github.com/repos/ceres-solver/ceres-solver/tarball/2.1.0 \
    -o ceres-2.1.0.tar.gz
  ssh com260 'mkdir -p ~/Downloads'
  rsync -av *.tar.gz com260:Downloads/
)
```

**电脑端**　从课程仓库根目录生成相机和 IMU 数据。以下工具在电脑端运行，生成的图像、标定和时间戳传给 COM260。

```bash
python3 -m venv /tmp/com260-synthetic-tools
/tmp/com260-synthetic-tools/bin/python -m pip install numpy scipy opencv-python-headless
/tmp/com260-synthetic-tools/bin/python \
  course_support/k3_com260_kit/visual_slam/synthetic_sequence.py /tmp/com260-synthetic-sequence
rsync -av /tmp/com260-synthetic-sequence/ com260:ros2_course_com260_ch21_frameworks/sequence/
```

生成目录须为空目录路径。重复生成时换一个目录名即可。数据包括 601 张 640×480 单目图像、6001 条 IMU、相机标定、时间戳和相机中心的世界坐标真值。相机中心与 IMU 原点重合。具体运行与导出步骤见第 21 章的 DSO 和 VINS-Mono 示例。

<a id="ch21-lvi"></a>

### 第 21 章 LVI-SAM 与手持录包

先完成上方的[双端有线 DDS 配置](#wired-dds)，并准备好第 21 章的 x86 工作区。LVI-SAM 的七个 C++ 节点在 COM260 上运行，RViz 使用 x86 的课程容器。

**准备录包。** 在能访问下载站的电脑上，从课程仓库根目录执行。需要 Python 3.10 或更高版本。以下命令从[原作者的 handheld 录包](https://github.com/TixiaoShan/LVI-SAM)下载前约 60 秒的完整数据块，转换时会校验 SHA-256，保留原始传感器时间戳。

```bash
data="$HOME/ros2_course_data"
mkdir -p "$data"
curl -fL --retry 3 --range 0-388342707 --max-filesize 388342708 \
  'https://drive.usercontent.google.com/download?id=1lE32SwMqvLpAI39zIlGe1QzLqS3d0CTA&export=download&confirm=t' \
  -o "$data/handheld-first60.part"
python3 -m venv "$data/lvi-tools"
"$data/lvi-tools/bin/python" -m pip install 'rosbags==0.11.9' 'Pillow>=10'
"$data/lvi-tools/bin/python" course_support/k3_com260_kit/lvi_sam/prepare-handheld.py \
  "$data/handheld-first60.part" "$data/handheld"
ssh com260 'mkdir -p ~/ros2_course_data'
scp -r "$data/handheld" com260:ros2_course_data/
```

**在 COM260 构建。** 脚本获取[固定版本的 ROS 2 源码](https://github.com/M-MohsinKhan/LVI-SAM-ROS2/tree/12187f80d8c955241d0f28f03d567a5092af2703)，校验后应用 Humble 适配。Bianbu 依赖解包到用户工作区，构建和运行均使用 Bubblewrap 的只读系统挂载。

```bash
cd ~/ROS2_RISCV_COM260_CH20_CH21
bash setup_course_k3_com260_kit.sh --build-ch21-lvi
~/ros2_course_com260_ch21_lvi/run ros2 bag info ~/ros2_course_data/handheld
```

话题列表应包含 596 帧 `/points_raw`、1,802 帧 `/camera/image_raw`、30,039 条 `/imu_raw` 和 30,040 条 `/imu_correct`。本例使用 `/imu_raw`，按录包标定与激光、相机对齐。

若板端下载源码较慢，可在联网电脑预取同一文件，再执行上述构建命令：

```bash
data="$HOME/ros2_course_data"
revision=12187f80d8c955241d0f28f03d567a5092af2703
archive="lvi-sam-$revision.tar.gz"
curl -fL --retry 3 "https://api.github.com/repos/M-MohsinKhan/LVI-SAM-ROS2/tarball/$revision" -o "$data/$archive"
ssh com260 'mkdir -p ~/Downloads'
scp "$data/$archive" com260:Downloads/
```

<a id="ch22"></a>

## 第 22 章 多传感器融合环境

本章对应[第12章实验指导书](ch12_lab.md)。COM260 运行 C++ 示例，x86 的独立课程容器运行 RGB-D、IMU、Gazebo、Cartographer 和 RViz。先完成[基础依赖](#nav2-prerequisites)以及第 9～10、20～21 章的镜像准备。

两端都使用包含本章内容的仓库副本 `~/ROS2_RISCV_COM260_CH22_CH23`。COM260 需要 Eigen、OpenCV、`message_filters`、`image_transport` 和 PNG 传输插件；第21章基础依赖已包含这些包。COM260 的构建入口为：

```bash
cd ~/ROS2_RISCV_COM260_CH22_CH23
bash setup_course_k3_com260_kit.sh --build-ch22
```

x86 使用普通用户构建课程容器：

```bash
cd ~/ROS2_RISCV_COM260_CH22_CH23
bash course_support/k3_com260_kit/scripts/x86-ch22-ch23.bash build-image
bash course_support/k3_com260_kit/scripts/x86-ch22-ch23.bash build
```

镜像基于 `localhost/ros2-course-com260:ch09-ch10`，并使用 `ch20-ch21` 的 RViz 关闭支持。两者缺失时分别执行 `x86-ch09-ch10.bash build-image`、`x86-ch20-ch21.bash build-image`。x86 工作区为 `~/ros2_course_com260_ch22_ch23_humble_ws`，COM260 工作区为 `~/ros2_course_com260_ch22_ch23_ws`。

两端接有线网络。按[有线地址的检查方法](#wired-dds)选取有效的 IPv6 地址，然后在两端分别运行配置脚本；第一个参数为本机地址，第二个为对端地址。以下文档示例地址需要替换：

```bash
cd ~/ROS2_RISCV_COM260_CH22_CH23
bash course_support/k3_com260_kit/scripts/configure-acquisition-dds.bash 2001:db8::1 2001:db8::2
```

**COM260 每个新终端：**

```bash
source ~/.config/ros2-course-com260-ch22-ch23/env.bash
source ~/.config/ros2-course-com260-ch22-ch23/dds.bash
```

**x86 每个新终端：**

```bash
cd ~/ROS2_RISCV_COM260_CH22_CH23
source ~/.config/ros2-course-com260-ch22-ch23/dds.bash
```

DDS 域为 222，两端使用 CycloneDDS 和有线 IPv6 单播。更换地址后重新生成配置，并重启使用旧配置的节点和容器。x86 需保持 X11 桌面登录；启动脚本检查 `:1` 显示会话。可访问 Intel 渲染设备时使用硬件渲染，否则使用软件渲染。

彩色、深度与点云在 x86 本地观察。COM260 同步示例接收无损 PNG 彩色图，避免百兆链路上原始 RGB-D 和点云堵塞其他传感器。不要把小于实时速度的 Gazebo 墙钟帧率当作相机标称采样率。

<a id="ch23"></a>

## 第 23 章 SLAM 与导航综合实训环境

本章依次使用[第13章建图实验](ch13_lab.md)和[第14章导航实验](ch14_lab.md)。先完成[第22章双端环境](#ch22)，再在 COM260 构建：

```bash
cd ~/ROS2_RISCV_COM260_CH22_CH23
bash setup_course_k3_com260_kit.sh --build-ch23
source ~/.config/ros2-course-com260-ch22-ch23/env.bash
source ~/.config/ros2-course-com260-ch22-ch23/dds.bash
```

C++ 包包括 `slam_bringup_lab`、`nav_bringup_lab`、`auto_explorer`、`map_tools_cpp`、`navigation_console` 和 `fleet_scheduler`。x86 的 `build` 入口同时构建这些包；后续更新源码后重跑同一入口即可。

教案综合项目的 `start-explore` 和 `start-project` 使用扩大后的 Museum 场景；建图与导航实验的 `start-slam`、`start-nav` 使用原尺寸场景。场景与地图必须配套，切换入口前先停止上一容器。

每次仿真指定一个 `RUN_ID`，例如 `ch13`。x86 新终端通过下列命令进入该容器；容器自动加载 ROS、工作区和 DDS 环境：

```bash
bash course_support/k3_com260_kit/scripts/x86-ch22-ch23.bash exec ch13 bash
```

容器内 `/workspace` 对应 x86 的 `~/ros2_course_com260_ch22_ch23_humble_ws`，`/maps` 对应 x86 的 `~/maps`。所有地图、录包和输出都应写入这两个持久目录。不同仿真入口共用话题和 TF，切换前先停止当前容器：

```bash
bash course_support/k3_com260_kit/scripts/x86-ch22-ch23.bash stop ch13
```
