# multi_robot_scheduler — 多机器人任务派发

第 20 章 20.6的 C++17 示例。

在 COM260 仓库根目录运行 `bash setup_course_k3_com260_kit.sh --build-ch20`，然后 `source ~/.config/ros2-course-com260-ch20-ch21/env.bash`。

可执行程序：`multi_robot_scheduler`。

任务文件用 task_file 参数指定；任务含 robot、x、y、recovery_strategy。只派发并确认请求接受，不负责启动多机器人或等待到达。

双端运行步骤见[第 20 章《行为树与恢复行为》](../../teaching_docs_k3_com260_kit/ch20_行为树与恢复行为.md)。
