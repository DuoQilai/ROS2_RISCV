# urdf_demo

COM260 第8章原教学建模练习的独立副本，使用 ament_cmake 安装 URDF/Xacro、RViz 配置和 Python Launch，无业务节点。

- `simple_robot.urdf` / `simple_robot.xacro`：实验8.1/8.2的蓝色底盘(0.4×0.3×0.15 m、5 kg)、双连续轮与万向轮。
- `marked_robot.xacro`：实验4红色半径0.03 m球标记，位于base_link上方0.3 m。
- `teaching_robot.xacro`：教案8.1～8.6的10 kg惯性、四轮宏与use_lidar条件。轮子用圆柱展示四轮宏位置，未模拟麦克纳姆滚子动力学。
- `display.launch.py`：可选model路径；use_gui=false启用无GUI关节状态发布，use_rviz=false省略RViz。

轮子圆柱体的绕X旋转放在visual/collision，joint轴保持Y，保证GUI改变角度时绕实际轮轴转动。
