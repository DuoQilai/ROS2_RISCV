# tf_demo_lab_cpp

COM260 第 7 章 C++17/Humble 练习。所有运行终端先 `source ~/.config/ros2-course-com260/env.bash`。

- `tf_broadcaster`：20 Hz，半径 1 m，角步长 0.05 rad；发布五个静态传感器 TF。原正文与源码角步长不一致，此实现采用实验正文 0.05。教学 10 Hz 使用 `--ros-args -p rate_hz:=10.0`。
- `tf_listener`：每秒输出 Laser→Base 和激光点 (1,0.5,0) 在相机系的位置。
- `tf_lookup`：每两秒查询仿真真实 `laser_link` 和 `camera_link`。
- `static_tf_pub`：默认 x=0.2,z=0.1；教案练习使用 `--ros-args -p x:=0.3 -p z:=0.15`。
- `time_query`：最新/历史时间查询、点(1,0,0)变换，最多三次、每次两秒等待；成功退出0，超时退出1。历史示例使用 `--ros-args -p target:=odom -p source:=base_link -p history_sec:=2.0`。

教学 TF 与 Gazebo 使用相同底盘 frame 名，必须分轮运行，不能同时发布同名父子关系。静态 broadcaster 发送一次后保持运行，允许后加入的监听器通过 transient-local 收到数据。
