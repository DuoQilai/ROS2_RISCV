# behavior_tree_demo_cpp — 行为树客户端与条件树

第 20 章 20.2、20.4的 C++17 示例。

在 COM260 仓库根目录运行 `bash setup_course_k3_com260_kit.sh --build-ch20`，然后 `source ~/.config/ros2-course-com260-ch20-ch21/env.bash`。

可执行程序：`bt_client / custom_behavior_tree`。

条件树从 /battery_status 读取 Float32 电量；大于 0.2 才导航。XML 路径属于运行 Nav2 的 x86 主机。

双端运行步骤见[第 20 章《行为树与恢复行为》](../../teaching_docs_k3_com260_kit/ch20_行为树与恢复行为.md)。
