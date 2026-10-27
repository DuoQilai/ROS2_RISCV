# sensor_interfaces

第 3 章使用的自定义消息，由 `sensor_pub_cpp` 发布和订阅。

- `SensorData.msg`：temperature、humidity、pressure、device_id。
- `Person.msg`：name（string）、age（int32）、height（float64）。

在 K3 的课程工作区构建和检查：

```bash
source ~/.config/ros2-course-k3/env.bash
cd ~/ros2_course_k3_ws
colcon build --base-paths src/course --packages-select sensor_interfaces sensor_pub_cpp
source install/setup.bash
ros2 interface show sensor_interfaces/msg/SensorData
ros2 interface show sensor_interfaces/msg/Person
```

发布、订阅与验证步骤见[第 3 章实验](../../lab_manuals_k3_pico_itx/ch03_lab.md)。
