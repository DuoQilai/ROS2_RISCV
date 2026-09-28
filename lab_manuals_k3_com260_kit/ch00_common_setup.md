# 第 0 章：COM260 公共双端环境

本版第一至四章已在 COM260 完成构建与运行验证。具体环境、结果与素材见[实际运行证据](runtime_evidence.md)。课程实现使用 C++17／rclcpp，仿真使用 TurtleBot3 Burger 与 Museum。原版及 Pico 版保留。

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

## 第三章源码与构建

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
## 第四章源码与构建

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
## 第五章动作通信环境

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
## 第六章参数与 Launch 环境

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
