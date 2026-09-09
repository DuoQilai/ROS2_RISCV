# 第 0 章：公共双端环境

本章集中说明所有实验共用的 SSH、环境脚本、Remote-SSH 和工具命名。各章节只需链接本页，不重复复制前置步骤。

[K3 与 x86 网络和 ROS 拓扑源码](images/topology_network.mmd)

[双端开发流程源码](images/topology_devflow.mmd)

## 设备与运行环境

本课程使用三端分工：

| 名称 | 设备和环境 | 用途 |
|---|---|---|
| 本机（访问端） | 本次验证使用 macOS，安装 VS Code/VSCodium | 编辑课程文件，通过 SSH 访问两台 Linux 设备 |
| K3 板端 | K3 Pico-ITX，Bianbu >= 4.0.1 / riscv64 / ROS 2 Humble | 构建、运行和调试课程节点 |
| x86 仿真宿主 | Ubuntu 24.04 电脑，运行 rootless Podman | 承载 Ubuntu 22.04 / Humble / Gazebo Harmonic / RViz 课程容器 |

### SSH 别名

`pico` 和 `duomaomao` 是本次验证环境在访问端 `~/.ssh/config` 中设置的别名：

- `ssh pico`：登录 K3 Pico-ITX 板端；本次板端用户名为 `pico`。
- `ssh duomaomao`：登录 x86 Ubuntu 24.04 仿真宿主。随后用 `podman exec` 进入该宿主上的课程容器。

在自己的访问端配置下面两项，将占位内容替换为实际设备地址和登录用户名：

```sshconfig
Host pico
    HostName YOUR_K3_ADDRESS
    User YOUR_K3_USER

Host duomaomao
    HostName YOUR_X86_ADDRESS
    User YOUR_X86_USER
```

SSH 别名与登录用户名可以不同。文中的 `~` 指命令执行端当前用户的主目录；`/home/pico` 是本次 K3 账号的主目录，使用其他账号时按实际路径替换。通过 SSH 执行仿真脚本时，命令运行在 x86 宿主；`podman exec` 后的命令运行在课程容器。

<a id="dds-contract"></a>

## DDS 通信约定

当前已验证组合为 K3 原生 Humble 与 x86 rootless Ubuntu 22.04/Humble 容器。
x86 容器使用 Gazebo Harmonic 8.15.0、RViz 11.2.28。宿主 Ubuntu 24.04/Jazzy 保留，
运行课程时不要让宿主 Jazzy 节点或 daemon 参与同一 DDS 域。

两端加载各自课程环境后，使用以下共同参数：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
```

两端节点同时存活时，已验证双向节点发现、参数和生命周期查询、基础消息收发、错误域隔离与恢复，以及 Gazebo `/clock` 向 K3 流转。
原生 Humble/Jazzy 曾通过基础消息收发，但节点发现出现反序列化错误，因此不作为当前课程组合。
各章行为及媒介交付状态见[K3 实际运行证据](runtime_evidence.md)。

## 连接与环境

【本机（访问端）】

```bash
ssh pico
```

【K3 板端】

```bash
source /etc/os-release
test "$ID" = bianbu
dpkg --compare-versions "$VERSION_ID" ge 4.0.1
test "$(uname -m)" = riscv64
source ~/.config/ros2-course-k3/env.bash
cd ~/ros2_course_k3_ws
```

K3 版只构建 `src_k3_pico_itx/`，默认工作区为 `~/ros2_course_k3_ws`；原版 `src/` 由原安装器使用。两个版本分别在独立终端中加载。

课程环境默认入口是 `~/.config/ros2-course-k3/env.bash`。首次通过 `ssh pico` 连接后必须核对实际路径、Bianbu >= 4.0.1 和 `riscv64`；版本比较命令返回 0 才满足基线，检查通过后继续。

## 本地编辑、同步与 Remote-SSH

本机 `/path/to/ROS2_RISCV` 是课程文件的唯一编辑源（请替换为自己的本地仓库路径）。在 VS Code 或
VSCodium 中打开该本地仓库，并使用 RuyiSDK VSCode 插件。修改完成后把 `setup_course_k3.sh`、`src_k3_pico_itx/` 和所需的 `course_support/k3/ide/`
从本机单向同步到 K3 的 `~/ROS2_RISCV`。正式同步前先保存 itemized dry-run 清单；出现
删除项或意外覆盖项时停止并确认，清单符合预期后才执行正式同步，最后核对 Git HEAD 与内容 SHA-256：

【本机（访问端）】

```bash
ssh pico 'test -d "$HOME/ROS2_RISCV/.git" && mkdir -p "$HOME/ROS2_RISCV/src_k3_pico_itx"'
sync_stamp=$(date -u +%Y%m%dT%H%M%SZ)
sync_audit="${TMPDIR:-/tmp}/ros2-course-sync-audit/$sync_stamp"
mkdir -p "$sync_audit"
rsync -ani setup_course_k3.sh pico:~/ROS2_RISCV/setup_course_k3.sh \
  | tee "$sync_audit/setup-dry-run.txt"
rsync -ani --delete \
  --exclude=.git/ --exclude=.venv/ --exclude=__pycache__/ \
  --exclude='*.pyc' --exclude=build/ --exclude=install/ --exclude=log/ \
  src_k3_pico_itx/ pico:~/ROS2_RISCV/src_k3_pico_itx/ \
  | tee "$sync_audit/src-dry-run.txt"
# 检查上面两份清单；确认无意外删除或覆盖后再执行以下两条命令。
rsync -ai setup_course_k3.sh pico:~/ROS2_RISCV/setup_course_k3.sh \
  | tee "$sync_audit/setup-applied.txt"
rsync -ai --delete \
  --exclude=.git/ --exclude=.venv/ --exclude=__pycache__/ \
  --exclude='*.pyc' --exclude=build/ --exclude=install/ --exclude=log/ \
  src_k3_pico_itx/ pico:~/ROS2_RISCV/src_k3_pico_itx/ \
  | tee "$sync_audit/src-applied.txt"
bash course_support/k3/scripts/check-course-sync.bash
```

只有 `SOURCE_SYNC_MATCH=true` 才能在板端同步受管课程工作区并构建。不要用板端 `git pull`
替代本机未提交内容，也不要直接在板端形成另一份课程源码。

Remote-SSH 用于打开 K3 终端、构建、运行和调试，不把板端工作副本作为独立编辑源。发现需修复时，
回到本机修改，再重新同步并核对哈希。Gazebo/RViz GUI 放在 x86 主机端；
插件安装、源码同步和远程连接结果需要在对应运行证据中记录。

VSCodium 的 Remote-SSH 目标选择 `pico`，远程文件夹打开 `/home/pico/ROS2_RISCV`。

### C++ 补全与调试工具

【本机访问端】本地安装 RuyiSDK 与 Open Remote - SSH；连接 K3 后，在扩展面板的
`SSH: pico` 端安装 Native Debug（`webfreak.debug`）和 clangd
（`llvm-vs-code-extensions.vscode-clangd`）。本轮使用 Native Debug 0.27.0、clangd 扩展 0.6.0。

【K3 板端】clangd 后端须为 riscv64。本轮从 Bianbu 包源安装到用户目录，避免使用扩展自动下载的 x86 后端；目录已存在时先核对 `--version`，不要覆盖其他安装。

```bash
mkdir -p ~/.cache/ros2-course/clangd ~/.local/share/ros2-course/clangd-21
cd ~/.cache/ros2-course/clangd
apt-get download clangd-21 libclang-common-21-dev
for package in ./*.deb; do
  dpkg-deb -x "$package" ~/.local/share/ros2-course/clangd-21
done
~/.local/share/ros2-course/clangd-21/usr/lib/llvm-21/bin/clangd --version
```

本次验证使用 Bianbu clangd 21.1.8，后端解包在用户目录，运行时共享库已在 K3 上具备。若提示缺少共享库，先按实际报错处理依赖。
在 VSCodium 的远程设置中设置 `clangd.path` 为
`/home/pico/.local/share/ros2-course/clangd-21/usr/lib/llvm-21/bin/clangd`，重载窗口。
各章再生成对应包的 `compile_commands.json`，使补全使用实际构建参数。

若重开窗口仍恢复迁移前的主机或主目录，重新选择该目标与路径，不从旧的“最近打开”条目恢复。
首次进入新路径时只信任课程文件夹；左下角应显示 `SSH: pico`，文件树应能正常展开。

## 运行支持文件放置

仓库内的运行支持集中在 [course_support/k3](../course_support/k3/README.md)。本机运行同步检查；仿真脚本通过 SSH 标准输入在 x86 执行。设备与 SSH 别名见本章开头。

首次准备 IDE 调试时，将仓库中的调试支持同步到 K3：

【本机访问端，课程仓库根目录】

```bash
ssh pico 'mkdir -p ~/ROS2_RISCV/course_support/k3/ide'
rsync -ani course_support/k3/ide/ pico:~/ROS2_RISCV/course_support/k3/ide/
# 核对清单后同步，不删除目标目录中的其他文件。
rsync -ai course_support/k3/ide/ pico:~/ROS2_RISCV/course_support/k3/ide/
```

【K3 板端，安装器已生成 env.bash 且课程已构建】

```bash
cd ~/ROS2_RISCV
mkdir -p ~/.config/ros2-course-k3
# 首次安装；已有脚本内容不同时先比较，保留原配置后再替换。
if [ ! -e ~/.config/ros2-course-k3/gdb.bash ]; then
  install -m 0755 course_support/k3/ide/gdb.bash ~/.config/ros2-course-k3/gdb.bash
else
  diff -u ~/.config/ros2-course-k3/gdb.bash course_support/k3/ide/gdb.bash
fi
~/.config/ros2-course-k3/gdb.bash --version
```

[launch.json 模板](../course_support/k3/ide/launch.json)在 ch01 指导书中给出；合并到 K3 工作副本的 `.vscode/launch.json`，保留已有其他调试配置。

## 最小环境检查

【K3 板端】

```bash
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unset}"
ros2 doctor --report
ros2 topic list
```

【x86 主机，进入 Humble 节点发现验证容器】

```bash
podman run --rm -it --network host --cap-drop all \
  --security-opt no-new-privileges localhost/ros2-course-humble:graph bash
```

【x86 容器内】

```bash
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unset}"
ros2 doctor --report
ros2 topic list --no-daemon
```

镜像入口已设置 Humble、CycloneDDS、域 0、localhost-only 0。宿主的
`~/.config/ros2-course/x86-env.bash` 是 Jazzy 环境，不能用于本轮 Humble 课程参与者。
安装与仿真派生镜像说明见 [容器环境](../course_support/k3/containers/README.md)。

两端的 ROS 发行版、RMW、`ROS_DOMAIN_ID` 和可见 Topic 必须以实际输出记录。Stage1 安装和构建成功不等于章节跨机验收完成。
`ros2 doctor` 原始输出仅供私下查看；正式录制使用 `asciinema/ros2/` 的脱敏场景，不直接录入地址或默认登录提示符。

## 启动与进入课程仿真容器

完成 [容器安装](../course_support/k3/containers/README.md) 并把本机 `src_k3_pico_itx/robot_sim_demo/` 按先 dry-run、后正式同步的方式
同步到 x86 同名课程路径后，从本机课程仓库执行以下命令。[仿真脚本](../course_support/k3/scripts/x86-gazebo.bash)在 x86 独立工作区构建，源码只读挂载。

【本机（访问端），首次或仿真源码更新后】

```bash
cd /path/to/ROS2_RISCV  # 替换为本机课程仓库路径
ssh -T duomaomao 'bash --noprofile --norc -s -- build' < course_support/k3/scripts/x86-gazebo.bash
```

【本机（访问端），手动实验】

```bash
run_id=$(date -u +%Y%m%dT%H%M%SZ)-manual
printf 'RUN_ID=%s\n' "$run_id"
ssh -T duomaomao "bash --noprofile --norc -s -- start '$run_id'" < course_support/k3/scripts/x86-gazebo.bash
ssh -t duomaomao "podman exec -it 'ros2-course-$run_id' bash --noprofile --norc"
```

【x86 容器内】加载环境后执行各章标为“x86 容器内”的命令：

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
printf 'ROS=%s RMW=%s DOMAIN=%s\n' "$ROS_DISTRO" "$RMW_IMPLEMENTATION" "$ROS_DOMAIN_ID"
```

原课程入口为 `robot_sim_demo/gazebo2.launch.py`，使用仓库自带的 Museum 与 TurtleBot3 Burger；
helper 启动 Gazebo/RViz，显式 `drive:=false`，运动指令来自当章 K3 节点。x86 必须已登录 X11 桌面；
GUI 使用软件渲染及课程容器内 Qt DPI 设置。章节自动录制已启动容器时不要重复启动。

【本机（访问端），ch01 生命周期验收】K3 的 `/hello_ros2_lifecycle` 已激活并发布速度后，在保留同一 `run_id` 的本机终端执行：

```bash
ssh -T duomaomao "bash --noprofile --norc -s -- observe-lifecycle '$run_id'" < course_support/k3/scripts/x86-gazebo.bash
```

该模式只观察已有课程容器，检查 `/cmd_vel` 中的 `x: 0.1`、生命周期 active 状态及节点可见性，是 ch01 专用验收。

【本机（访问端），实验结束后】

```bash
ssh -T duomaomao "bash --noprofile --norc -s -- stop '$run_id'" < course_support/k3/scripts/x86-gazebo.bash
```

helper 仅停止同 RUN_ID 标签匹配的课程容器；正常停止后保留镜像与独立构建工作区。

## 章节引用

后续章节在公共环境部分使用：`参见 [公共双端环境](ch00_common_setup.md)`。
