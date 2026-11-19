# orb_feature_extractor — ORB 特征提取与匹配

第 21 章 21.3的 C++17 示例。

在 COM260 仓库根目录运行 `bash setup_course_k3_com260_kit.sh --build-ch21`，然后 `source ~/.config/ros2-course-com260-ch20-ch21/env.bash`。

可执行程序：`orb_feature_extractor`。

输入 /camera/image_raw，输出 /orb/matches；保留 1000 特征、1.2 尺度、8 层及 0.75 比率测试。只匹配相邻帧，不估计 SLAM 位姿。

双端运行步骤见[第 21 章《视觉SLAM导论》](../../teaching_docs_k3_com260_kit/ch21_视觉SLAM导论.md)。
