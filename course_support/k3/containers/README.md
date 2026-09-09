# x86 Humble 隔离验证环境

课程容器运行在 Ubuntu 24.04 x86 宿主上，使用普通用户的 rootless Podman。节点发现镜像提供 Ubuntu 22.04 / ROS 2 Humble，Gazebo/RViz 使用其派生镜像。设备分工及 SSH 配置见[公共环境](../../../lab_manuals_k3_pico_itx/ch00_common_setup.md#ssh-别名)。

## 安装与构建

从访问端运行 `ssh duomaomao`，登录 x86 Ubuntu 24.04 仿真宿主；在宿主的交互式终端执行一次系统包安装：

```bash
sudo apt-get update
sudo apt-get install -y --no-remove --no-install-recommends \
  podman uidmap slirp4netns fuse-overlayfs
```

本机将仓库根目录的 `setup_course_x86_humble_container.sh` 和 `course_support/k3/containers/` 目录同步到 x86 的同名课程路径后，由普通用户执行：

```bash
cd ~/ROS2_RISCV
bash setup_course_x86_humble_container.sh --dry-run
bash setup_course_x86_humble_container.sh
bash setup_course_x86_humble_container.sh --verify
```

脚本使用 [ROS 官方镜像](https://github.com/docker-library/official-images/blob/master/library/ros)
`docker.io/library/ros:humble-ros-base-jammy`，拉取后记录实际仓库 digest，并用该 digest
构建 `localhost/ros2-course-humble:graph`。包安装只发生在镜像内；当前 Dockerfile 来自
[官方 ros-base](https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/ros-base/Dockerfile)
的派生。

若 x86 不能访问 Docker Hub，可在能访问官方仓库的 Mac 上用现有 Docker 拉取同一
`linux/amd64` 镜像，记录官方 repo digest、image ID，使用 `docker save` 导出，再经 SSH
转存到 x86。两端核对归档 SHA-256，`podman load` 后核对 image ID 和架构一致，才可执行
`bash setup_course_x86_humble_container.sh --offline`。不换第三方镜像或关闭 TLS 校验。
离线导入可能重新生成 manifest digest，因此离线模式始终用已核对的不可变 image ID 构建；原官方 digest
仍须记录在转存证据中，不能把 tar 文件摘要或 image ID 写成官方 repo digest。
`--offline` 仅跳过基础镜像拉取，镜像构建中的 apt 依然需要联网。

## Humble + Harmonic 派生镜像

双端 Humble 节点发现通过后，在同一普通用户终端执行：

```bash
cd ~/ROS2_RISCV
bash setup_course_x86_humble_container.sh --gazebo
bash setup_course_x86_humble_container.sh --verify-gazebo
```

该模式使用已构建的 graph 镜像不可变 ID，派生 `localhost/ros2-course-humble:gazebo`，
只在容器内添加 [Gazebo 官方 OSRF 仓库](https://gazebosim.org/docs/harmonic/install_ubuntu/)，
公开签名密钥同时使用 HTTPS 和固定 SHA-256 校验。按
[官方 Humble/Harmonic 配对说明](https://gazebosim.org/docs/harmonic/ros_installation/)
安装 `ros-humble-ros-gzharmonic`，不安装 Humble 默认的 Fortress `ros-humble-ros-gz`。
Gazebo/RViz 与课程模型的运行记录见[实际运行证据](../../../lab_manuals_k3_pico_itx/runtime_evidence.md)。

## 验证边界

### 运行范围

- 宿主保持 Ubuntu 24.04 原生运行；容器内单独核验 Ubuntu 22.04 / amd64 / Humble。
- 容器 rootless、host 网络，因而可以通过真实局域网 DDS 与 K3 通信。
- 同一次跨机验收中，不运行宿主 Jazzy 的 ROS 节点或 daemon。
- 双端沿用 CycloneDDS、域 0、localhost-only 0；必须让 K3 节点与容器节点同时存活，
  双向检查节点名，并从容器查询 K3 的参数和生命周期。
- 当前状态：graph 镜像和双端节点发现正式通过；Harmonic 派生镜像构建、新组合基础 DDS 已通过。Gazebo/RViz 模型、传感器和各章配对结果见实际运行证据。

[Podman 官方安装说明](https://podman.io/docs/installation#ubuntu) 提供 Ubuntu 软件包路径。
模拟安装已确认新增 11 包、0 升级、0 删除；实际安装前仍以 apt 的最新清单为准。
