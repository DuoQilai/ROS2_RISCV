# camera_processor — 相机数据处理

第 21 章 21.2的 C++17 示例。

在 COM260 仓库根目录运行 `bash setup_course_k3_com260_kit.sh --build-ch21`，然后 `source ~/.config/ros2-course-com260-ch20-ch21/env.bash`。

可执行程序：`camera_processor`。

RGB-D 采样使用对齐的彩色图、32FC1 米制深度和相机内参。步长 5、有效深度 0～5 m；RGB-only 输入不会产生深度点云。

双端运行步骤见[第 21 章《视觉SLAM导论》](../../teaching_docs_k3_com260_kit/ch21_视觉SLAM导论.md)。
