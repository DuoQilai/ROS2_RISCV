# K3 课程运行支持

本目录与课程源码一起保留在 `ROS2_RISCV` 仓库中。原录制仓库中的采集脚本仍用于制作素材；读者运行课程使用本目录提供的支持文件。

| 文件 | 放在哪里 | 在哪里执行、用途 |
|---|---|---|
| [scripts/check-course-sync.bash](scripts/check-course-sync.bash) | 本机课程仓库，保持本目录结构 | 本机执行；经 `ssh pico` 只读比较本机与 K3 的 HEAD 和 `setup_course_k3.sh`、`src_k3_pico_itx/` 内容指纹，并检查 K3 平台 |
| [scripts/x86-gazebo.bash](scripts/x86-gazebo.bash) | 本机课程仓库；正文通过 SSH 标准输入传送，无需另存到 x86 | 在 x86 执行 `build`、`start RUN_ID`、`observe-lifecycle RUN_ID`、`stop RUN_ID`；构建与管理课程仿真容器 |
| [ide/gdb.bash](ide/gdb.bash) | 源文件保留在仓库；安装到 K3 `~/.config/ros2-course-k3/gdb.bash`，权限 0755 | K3 上由调试扩展调用；加载课程环境后执行 `/usr/bin/gdb`，参数原样传递 |
| [ide/launch.json](ide/launch.json) | 合并到 K3 `~/ROS2_RISCV/.vscode/launch.json` | Mac VS Code/VSCodium 通过 Remote-SSH 打开 K3 工作副本后使用 |
| [containers/](containers/README.md) | 本机及 x86 `~/ROS2_RISCV/course_support/k3/containers/` | 供根目录 `setup_course_x86_humble_container.sh` 构建课程镜像 |

## 使用前提

- 本机的 SSH 别名 `pico` 登录 K3 Pico-ITX 板端，`duomaomao` 登录 Ubuntu 24.04 x86 仿真宿主；配置方法见[设备与 SSH 别名](../../lab_manuals_k3_pico_itx/ch00_common_setup.md#ssh-别名)。首次连接先确认主机指纹。
- K3 源码位于 `~/ROS2_RISCV`，受管构建位于 `~/ros2_course_k3_ws`；根目录 `setup_course_k3.sh` 生成 `~/.config/ros2-course-k3/env.bash`。先安装、构建，再启用 GDB 启动脚本。新终端中执行 `source ~/.config/ros2-course-k3/env.bash` 加载 K3 环境；安装器在 `.bashrc` 提供 `k3env` 别名。
- x86 源码位于 `~/ROS2_RISCV/src_k3_pico_itx/robot_sim_demo`，独立构建目录为 `~/ros2_course_humble_ws`。按[容器说明](containers/README.md)准备 `localhost/ros2-course-humble:gazebo` 镜像。
- 仿真脚本沿用已验证的 Ubuntu 24.04 物理主机、普通用户 rootless Podman、GDM X11 桌面 `DISPLAY=:1` 与 `/run/user/UID/gdm/Xauthority`。需要 `xdpyinfo`；先从 x86 的已登录桌面核对条件，其他桌面配置不可直接假定兼容。
- `observe-lifecycle RUN_ID` 是 ch01 验收专用：读取该运行容器中的 `/cmd_vel`，检查 `x: 0.1`，并检查 `/hello_ros2_lifecycle` 为 active 且节点可见。应在 K3 生命周期节点激活后执行。
- `start` 使用软件渲染、Museum/Burger、`drive:=false`，由章节节点控制运动；单次运行最多 3600 秒。`stop` 只操作 RUN_ID 标签匹配的课程容器。

## 放置与执行步骤

安装 GDB 启动脚本、合并调试配置、同步检查及仿真构建/启停的完整命令见[公共双端环境](../../lab_manuals_k3_pico_itx/ch00_common_setup.md#运行支持文件放置)和[仿真容器操作](../../lab_manuals_k3_pico_itx/ch00_common_setup.md#启动与进入课程仿真容器)。

从本机任意目录调用 `check-course-sync.bash` 时，默认按脚本位置找到本仓库；也可通过 `ROS2_RISCV_LOCAL_REPO` 指定本机课程副本。退出 0 且输出 `SOURCE_SYNC_MATCH=true` 表示指定范围一致；不一致时核对同步清单。
