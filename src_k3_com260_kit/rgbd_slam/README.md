# rgbd_slam — RGB-D 点云、ICP 与 TSDF

第 21 章 21.5的 C++17 示例。

在 COM260 仓库根目录运行 `bash setup_course_k3_com260_kit.sh --build-ch21`，然后 `source ~/.config/ros2-course-com260-ch20-ch21/env.bash`。

可执行程序：`rgbd_slam`。

入口使用已知彩色/深度输入检查点云融合和 ICP，不能代替真实相机建图。逐帧配准保留教学简化，未补全位姿图、回环或全局一致性。

双端运行步骤见[第 21 章《视觉SLAM导论》](../../teaching_docs_k3_com260_kit/ch21_视觉SLAM导论.md)。
