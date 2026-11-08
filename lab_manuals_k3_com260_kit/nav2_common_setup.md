# Nav2 公共环境与第 16～17 章操作

本页供教案第 16～17 章使用。它覆盖原第 11 章导航实验的组件、单目标、五点任务与代价地图基础；完整第 11 章还包含后续路径规划、控制、恢复与综合巡逻内容。

## 1. 双端准备

先完成[公共双端环境](ch00_common_setup.md)、第 6 章 Nav2 镜像与第 10 章接口 underlay。将当前仓库放到两端的 `~/ROS2_RISCV_COM260_CH16_CH17`。本章构建独立工作区，不覆盖前面章节的安装目录。

- COM260：ROS 2 Humble、C++ 编译器、colcon 模块、`libyaml-cpp-dev`；使用已有 `~/ros2_course_com260_ch09_ch10_deps_ws` 中固定版本的 `nav2_msgs`。
- x86：rootless Podman、Humble Nav2 镜像 `localhost/ros2-course-com260:ch06-nav2`，以及可访问的 X11 `:1` 桌面。镜像缺失时，在仓库内执行 `bash course_support/k3_com260_kit/scripts/x86-ch06.bash build-image`。
- 两端接有线网络。下例地址为 COM260 `192.168.1.88`、x86 `192.168.1.172`，换机时改为实际有线地址。DDS 域使用 181，两端必须相同。

<a id="packages"></a>

源码按实例和功能组织；第 16 章与第 17 章分别通过总 setup 构建。

| 章节 | 源码包 | 可执行文件 |
|---|---|---|
| 16 | [custom_nav_component](../src_k3_com260_kit/custom_nav_component/README.md) | `custom_nav_component` |
| 16 | [navigation_client_cpp](../src_k3_com260_kit/navigation_client_cpp/README.md) | `bt_navigator_client`、`multi_goal_navigation` |
| 16 | [global_planner_client](../src_k3_com260_kit/global_planner_client/README.md) | `global_planner_client` |
| 17 | [costmap_demo](../src_k3_com260_kit/costmap_demo/README.md) | `costmap_demo`、`inflation_plot` |
| 17 | [map_loader](../src_k3_com260_kit/map_loader/README.md) | `map_loader` |
| 17 | [obstacle_layer_processor](../src_k3_com260_kit/obstacle_layer_processor/README.md) | `obstacle_layer_processor` |
| 17 | [costmap_debugger](../src_k3_com260_kit/costmap_debugger/README.md) | `costmap_debugger` |

`navigation_client_cpp` 内的两个程序共享小型 Action 客户端；`obstacle_layer_processor` 复用 `costmap_demo` 的栅格数据结构。x86 的章节脚本负责容器构建、启动与停止。

COM260：

```bash
cd ~/ROS2_RISCV_COM260_CH16_CH17
export ROS_DOMAIN_ID=181
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.88"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.172"/></Peers></Discovery></Domain></CycloneDDS>'
bash setup_course_k3_com260_kit.sh --build-ch16
bash setup_course_k3_com260_kit.sh --build-ch17
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
```

x86：

```bash
cd ~/ROS2_RISCV_COM260_CH16_CH17
export ROS_DOMAIN_ID=181
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="192.168.1.172"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/><Peer Address="192.168.1.88"/></Peers></Discovery></Domain></CycloneDDS>'
bash course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash build
export RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-nav2"
bash course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash start "$RUN_ID"
```

此入口启动 Burger／Museum、Nav2 和 RViz。保留当前终端的 `RUN_ID`，停止时使用相同值；每次重新启动生成新编号，日志与媒体按该编号分别保存。仿真运行期间，COM260 各终端先加载本章 `env.bash`，ROS 节点使用 `use_sim_time:=true`。不要同时向 `/cmd_vel` 发送遥控或巡逻指令。

## 2. 组件与生命周期

COM260 查询远端组件：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 node list
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /global_costmap/global_costmap
ros2 action list
```

独立练习状态转换，终端一运行：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 run custom_nav_component custom_nav_component --ros-args -p use_sim_time:=true
```

终端二依次运行：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 lifecycle set /custom_nav_component configure
ros2 lifecycle set /custom_nav_component activate
ros2 lifecycle set /custom_nav_component deactivate
ros2 lifecycle set /custom_nav_component cleanup
ros2 lifecycle set /custom_nav_component configure
ros2 lifecycle set /custom_nav_component activate
ros2 lifecycle set /custom_nav_component shutdown
```

Active 时记录激光样本，Inactive 时停止处理；cleanup 后可再次 configure。Finalized 不意味着进程已经退出，结束示例终端可按 Ctrl+C。

## 3. 规划、导航和取消

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 run global_planner_client global_planner_client --ros-args -p use_sim_time:=true
ros2 run navigation_client_cpp bt_navigator_client --ros-args -p use_sim_time:=true
```

默认目标 `(3,2)` 靠近 Museum 的障碍区，NavFn 可选择容差内的可达终点。本章独立启动文件将 `GridBased.tolerance` 设为 0.15 m，并将目标检查器与 DWB 的 `xy_goal_tolerance` 同步设为 0.10 m，减少规划终点偏移与控制器提前停止带来的累积误差；末段进度检测距离使用 0.10 m，超时仍为 10 秒，避免 0.5 m 的旧距离要求在终点转向前过早中止；第 6 章原参数不变。验收仍要求距原目标不超过 0.25 m，并核对 Action 状态和 `/odom` 停止反馈；实际结果见[证据页](runtime_evidence.md#ch16-ch17)。五点练习使用空闲区域：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 run navigation_client_cpp multi_goal_navigation --ros-args -p use_sim_time:=true
```

取消练习应在机器人距目标仍有足够距离时进行：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 run navigation_client_cpp bt_navigator_client --ros-args \
  -p use_sim_time:=true -p x:=3.0 -p y:=2.0 -p cancel_after:=6.0
```

看到 CANCELED 后还要检查 `/odom` 速度接近零。Ctrl+C／SIGTERM 走同一取消流程；程序退出码 1 表示没有完成目标，在主动取消场景是预期结果。

行为树路径由 x86 上的 Nav2 读取。例如 COM260 可以传入 x86 容器中的绝对路径：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 run navigation_client_cpp bt_navigator_client --ros-args \
  -p use_sim_time:=true -p x:=1.0 -p y:=0.0 \
  -p behavior_tree:=/workspace/install/navigation_client_cpp/share/navigation_client_cpp/behavior_trees/teaching_navigation.xml
```

## 4. 代价地图

确定输入与曲线生成：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
mkdir -p ~/nav2_teaching_output
cd ~/nav2_teaching_output
ros2 run costmap_demo costmap_demo
ros2 run costmap_demo inflation_plot
```

输出 `inflation_curves.csv` 和 `inflation_curves.svg`。SVG 可复制到有浏览器的电脑查看。

真实地图统计与教学激光处理器分别在两个终端运行：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 run costmap_debugger costmap_debugger --ros-args -p use_sim_time:=true
```

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 run obstacle_layer_processor obstacle_layer_processor --ros-args -p use_sim_time:=true
```

RViz 中的全局／局部 Costmap 已配置。可另加 Map 显示 `/teaching_obstacles`，使用 Reliable／Transient Local；这是教学处理器输出，不能当作 Nav2 的生产地图。

停止导航后，可比较膨胀梯度，最后恢复原值：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 1.0
ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 10.0
ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 3.0
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap '{}'
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap '{}'
```

每次设置后等待地图实际更新再执行下一条，比较颜色与原始值。清除后静态地图和有效传感器输入会重新写入；不能将“服务返回”理解为所有障碍永久消失。

地图加载教学节点使用重映射，避免覆盖导航地图：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 run map_loader map_loader --ros-args -p use_sim_time:=true \
  -r /map:=/teaching_map -r /map_server/load_map:=/teaching/load_map
```

另一终端调用，路径由 COM260 读取：

```bash
source ~/.config/ros2-course-com260-ch16-ch17/env.bash
ros2 service call /teaching/load_map nav2_msgs/srv/LoadMap \
  "{map_url: '$HOME/ROS2_RISCV_COM260_CH16_CH17/src_k3_com260_kit/param_demo_cpp/maps/Software_Museum.yaml'}"
```

## 5. 结束

先结束 COM260 客户端和观察节点，再在 x86 停止本轮仿真：

```bash
cd ~/ROS2_RISCV_COM260_CH16_CH17
bash course_support/k3_com260_kit/scripts/x86-ch16-ch17.bash stop "$RUN_ID"
```

理论、例程与实际证据分别见[第 16 章教案](../teaching_docs_k3_com260_kit/ch16_Nav2架构与核心组件.md)、[第 17 章教案](../teaching_docs_k3_com260_kit/ch17_全局代价地图.md)和[运行证据](runtime_evidence.md#ch16-ch17)。
