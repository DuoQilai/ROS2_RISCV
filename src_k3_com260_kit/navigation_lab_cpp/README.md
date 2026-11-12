# navigation_lab_cpp

本包提供 Nav2 自主导航的 C++17 示例，运行步骤见[实验指导书](../../lab_manuals_k3_com260_kit/ch11_lab.md)。

| 入口 | 功能 |
|---|---|
| `go_to_pose_demo` | 四次 NavigateToPose：(3,0)、(3,2)、(0,2)、(0,0) |
| `follow_waypoints_demo` | 一个 FollowWaypoints 任务：(2,0)、(2,2)、(0,2)、(0,0) |
| `recovery_demo` | 向 (4,0) 导航，观察恢复行为及反馈 |
| `scan_injector` | 10 Hz 合成 LaserScan 和圆形轨迹 |
| `nav_monitor` | `/plan`、`/local_plan`、`/cmd_vel`、`/odom` 监控 |
| `waypoint_patrol` | 四个航点、两圈巡逻，失败时停止 |

通过 `setup_course_k3_com260_kit.sh` 的 `--build-ch16`、`--build-ch17`、`--build-ch18` 或 `--build-ch19` 构建；运行时加载对应工作区环境。实验指导书使用 `~/ros2_course_com260_ch18_ch19_ws`。

导航程序使用 C++ Action API；`navigator.hpp` 负责初始定位和两个 Action 的等待/取消逻辑。退出 0 要求所有目标完成且无遗漏航点；失败、主动取消或未就绪返回 1。

`scan_injector` 使用 Burger 的 `laser_link` 坐标系，轨迹相对传感器；它不是世界坐标中的移动实体。`nav_monitor` 根据路径首尾点变化计数，`/odom` 回调不处理数据；路径计数本身不能证明障碍导致重规划。

真实激光发布到 `/scan`，合成扫描发布到 `/scan_injected`；启动文件为代价地图配置两个独立观测源，AMCL 只使用真实激光。注入观测缓存保留 0.2 秒；停止后缓存过期，残留栅格由后续真实扫描清除。练习 3 的可移除障碍见实验指导书。

六个练习使用 `x86-ch18-ch19.bash start-lab RUN_ID` 启动配套导航场景和地图；`start` 入口使用 Museum。配套文件位于 `course_support/k3_com260_kit/config/navigation-lab*`。
