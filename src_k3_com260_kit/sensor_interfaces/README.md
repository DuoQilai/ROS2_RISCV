# sensor_interfaces

第 3 章使用的自定义消息，由 `sensor_pub_cpp` 发布和订阅。

- `SensorData.msg`：temperature、humidity、pressure、device_id。
- `Person.msg`：name（string）、age（int32）、height（float64）。

在 COM260 的仓库目录构建和检查：

```bash
bash setup_course_k3_com260_kit.sh --build-ch03
source ~/.config/ros2-course-com260/env.bash
ros2 interface show sensor_interfaces/msg/SensorData
ros2 interface show sensor_interfaces/msg/Person
```

发布、订阅与验证步骤见[第 3 章实验](../../lab_manuals_k3_com260_kit/ch03_lab.md)。
