# 第3章 实验指导书：话题通信编程与实践

连接和环境加载见[公共双端环境](ch00_common_setup.md)。发布者、订阅者和控制节点在 K3 运行，Gazebo、rqt、RViz 在 x86 课程容器运行。

## 当前仓库仿真验证：订阅 Gazebo 的 `/scan` 与 `/odom`

### 实验目标

把本实验的 Publisher/Subscriber 知识应用到真实仿真话题，检查消息类型、发布者数量、QoS 和持续数据流。

### 运行步骤

【x86 课程容器】

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py \
  gui:=false rviz:=false drive:=false
```

【K3】另开终端，先停止其他 `/cmd_vel` 发布者：

```bash
source ~/.config/ros2-course-k3/env.bash
source ~/ros2_course_k3_ws/install/setup.bash
ros2 topic info /scan
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
# 观察消息后发送零速停止。
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

### 观察与验收

`/scan` 应为 `sensor_msgs/msg/LaserScan`，`/odom` 应为 `nav_msgs/msg/Odometry`；可将实验订阅节点的输入改为 `/scan`。源码：`src_k3_pico_itx/robot_sim_demo/config/gazebo2_bridge.yaml`。

> **实验课时**：2 课时（90 分钟）
> **实验平台**：K3 Pico-ITX / Bianbu >= 4.0.1 / ROS 2 Humble；x86 Humble/Harmonic 课程容器

---

## 实验目标

完成本实验后，学员应能够：
1. 编写 Publisher 和 Subscriber 节点
2. 理解标准消息类型的使用
3. 创建并使用自定义消息接口
4. 配置测试不同的 QoS 策略

---

## 练习 3.1：Publisher 与 Subscriber 通信（约 30 分钟）

### 目标
实现一个完整的发布-订阅通信链路，发布者周期发送消息，订阅者接收并打印。

### 步骤

**步骤1：创建 topic_demo_cpp 包**
```bash
source ~/.config/ros2-course-k3/env.bash
mkdir -p ~/my_ros2_ws/src
cd ~/my_ros2_ws/src
ros2 pkg create topic_demo_cpp --build-type ament_cmake \
  --dependencies rclcpp std_msgs geometry_msgs
```

**步骤2：编写 Publisher 节点**

在 `~/my_ros2_ws/src/topic_demo_cpp/src/` 下创建 `publisher.cpp`：

```cpp
#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PositionPublisher : public rclcpp::Node
{
public:
  PositionPublisher()
  : Node("gps_publisher"), x_(0.0)
  {
    publisher_ = create_publisher<geometry_msgs::msg::Point>("/gps_position", 10);
    timer_ = create_wall_timer(1s, std::bind(&PositionPublisher::publish, this));
  }

private:
  void publish()
  {
    geometry_msgs::msg::Point message;
    message.x = x_;
    message.y = 2.0 * x_ + 1.0;
    publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "GPS_PUBLISHED x=%.2f y=%.2f", message.x, message.y);
    x_ += 1.0;
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

**步骤3：编写 Subscriber 节点**

在 `topic_demo_cpp/src/` 下创建 `gps_subscriber.cpp`：

```cpp
#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

class GpsSubscriber : public rclcpp::Node
{
public:
  GpsSubscriber()
  : Node("gps_subscriber")
  {
    subscription_ = create_subscription<geometry_msgs::msg::Point>(
      "/gps_position", 10, std::bind(&GpsSubscriber::handle, this, std::placeholders::_1));
  }

private:
  void handle(const geometry_msgs::msg::Point::SharedPtr message)
  {
    const double distance = std::hypot(message->x, message->y);
    RCLCPP_INFO(
      get_logger(), "GPS_RECEIVED x=%.2f y=%.2f distance=%.2f", message->x, message->y, distance);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

**步骤4：配置 CMakeLists.txt 并编译**

编辑 `topic_demo_cpp/CMakeLists.txt`：
```cmake
cmake_minimum_required(VERSION 3.10)
project(topic_demo_cpp)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(publisher src/publisher.cpp)
target_compile_features(publisher PUBLIC cxx_std_17)
ament_target_dependencies(publisher rclcpp geometry_msgs)
install(TARGETS publisher DESTINATION lib/${PROJECT_NAME})
add_executable(gps_subscriber src/gps_subscriber.cpp)
target_compile_features(gps_subscriber PUBLIC cxx_std_17)
ament_target_dependencies(gps_subscriber rclcpp geometry_msgs)
install(TARGETS gps_subscriber DESTINATION lib/${PROJECT_NAME})
ament_package()
```

```bash
cd ~/my_ros2_ws
colcon build --packages-select topic_demo_cpp --symlink-install
source install/setup.bash
```

**步骤5：运行测试**
```bash
# 终端1：启动发布者
ros2 run topic_demo_cpp publisher
# 终端2：启动订阅者
ros2 run topic_demo_cpp gps_subscriber
# 终端3：监控
ros2 topic echo /gps_position
ros2 topic hz /gps_position       # 查看发布频率（期望 ~1Hz）
```

**步骤5：检查运行结果与图示一致**

- 话题消息字段：

![K3 gps-message 对应实测结果](images/ch03/delivery-gps-message.png)

- 截图1：publisher 发布日志

![K3 gps-publisher 对应实测结果](images/ch03/delivery-gps-publisher.png)

- 截图2：subscriber 接收日志

![K3 gps-subscriber 对应实测结果](images/ch03/delivery-gps-subscriber.png)

- 截图3：`ros2 topic hz /gps_position` 输出（频率）

![K3 /gps_position 发布频率约 1 Hz。](images/ch03/delivery-gps-frequency.png)

- 截图4：rqt_graph 通信拓扑

【x86 课程容器】加载公共环境后运行：

```bash
rqt_graph
```

![x86 rqt_graph 显示 K3 GPS 发布者、话题与订阅者。](images/ch03/delivery-rqt-graph.png)

### 参考代码
> 完整参考代码位于 `src_k3_pico_itx/topic_demo_cpp/`

---

## 练习 3.2：自定义消息接口（约 30 分钟）

### 目标
创建自定义消息接口包，定义 `SensorData.msg` 并在 C++17 中使用。

### 步骤

**步骤1：创建 CMake 消息接口包**（先查有没有，有的话就不用创建了）
```bash
cd ~/my_ros2_ws/src
ros2 pkg create sensor_interfaces --build-type ament_cmake \
  --dependencies rosidl_default_generators
```

**步骤2：定义 SensorData.msg**

创建 `sensor_interfaces/msg/SensorData.msg`：
因为没有msg文件夹，所以用绝对路径创建
mkdir -p ~/my_ros2_ws/src/sensor_interfaces/msg
nano ~/my_ros2_ws/src/sensor_interfaces/msg/SensorData.msg
```text
# 传感器数据消息定义
float64 temperature      # 温度 (℃)
float64 humidity         # 湿度 (%)
float64 pressure         # 气压 (hPa)
string device_id         # 传感器ID
```

**步骤3：配置 CMakeLists.txt**
```cmake
cmake_minimum_required(VERSION 3.8)
project(sensor_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
)
ament_package()
```

**步骤4：配置 package.xml**

在生成的 `<package>` 内保留 `ament_cmake` 构建依赖，并补充：

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**步骤5：编译接口包**
```bash
cd ~/my_ros2_ws
colcon build --packages-select sensor_interfaces
source install/setup.bash
ros2 interface show sensor_interfaces/msg/SensorData
```

**步骤6：创建使用自定义消息的 C++17 包**
```bash
cd ~/my_ros2_ws/src
ros2 pkg create sensor_pub_cpp --build-type ament_cmake \
  --dependencies rclcpp sensor_interfaces
```

编写 `sensor_pub_cpp/src/sensor_publisher.cpp`：
```cpp
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_interfaces/msg/sensor_data.hpp"

using namespace std::chrono_literals;

class SensorPublisher : public rclcpp::Node
{
public:
  SensorPublisher()
  : Node("sensor_publisher")
  {
    publisher_ = create_publisher<sensor_interfaces::msg::SensorData>("/sensor_data", 10);
    timer_ = create_wall_timer(1s, std::bind(&SensorPublisher::publish, this));
  }

private:
  void publish()
  {
    sensor_interfaces::msg::SensorData message;
    message.temperature = 25.5F;
    message.humidity = 60.0F;
    message.pressure = 1013.25F;
    message.device_id = "sensor_01";
    publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "SENSOR_PUBLISHED device_id=%s", message.device_id.c_str());
  }

  rclcpp::Publisher<sensor_interfaces::msg::SensorData>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

**步骤7：编译并运行**
在 `sensor_pub_cpp/CMakeLists.txt` 中写入：

```cmake
cmake_minimum_required(VERSION 3.10)
project(sensor_pub_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_interfaces REQUIRED)

add_executable(sensor_publisher src/sensor_publisher.cpp)
target_compile_features(sensor_publisher PUBLIC cxx_std_17)
ament_target_dependencies(sensor_publisher rclcpp sensor_interfaces)
install(TARGETS sensor_publisher DESTINATION lib/${PROJECT_NAME})
ament_package()
```

```bash
source /opt/ros/humble/setup.bash
cd ~/my_ros2_ws

colcon build --packages-select sensor_interfaces sensor_pub_cpp --symlink-install
source ~/my_ros2_ws/install/setup.bash

ros2 interface show sensor_interfaces/msg/SensorData
ros2 pkg executables sensor_pub_cpp
ros2 run sensor_pub_cpp sensor_publisher # 终端1
ros2 topic echo /sensor_data          # 终端2
```

**步骤7：检查运行结果与图示一致**

- 截图1：`ros2 interface show sensor_interfaces/msg/SensorData` 输出消息定义

![K3 生成的 SensorData 消息字段。](images/ch03/delivery-sensor-interface.png)

- 截图2：发布者和 echo 终端输出

![独立 sensor_pub_cpp 包发布的四个消息字段。](images/ch03/delivery-sensor-message.png)

![独立发布者运行日志及正常停止。](images/ch03/delivery-sensor-publisher.png)

- 截图3：rqt_graph 中的自定义消息话题

![x86 rqt_graph 显示 K3 发布的 GPS 与 SensorData 话题](images/ch03/delivery-rqt-graph.png)

### 参考代码
> 完整参考代码位于 `src_k3_pico_itx/sensor_interfaces/` 和 `src_k3_pico_itx/sensor_pub_cpp/`

---

## 练习 3.3：QoS 策略对比实验（约 30 分钟）

### 目标
测试不同 QoS 配置下的通信行为差异。

### 步骤

**步骤1：编写 QoS 测试发布者**

在 `topic_demo_cpp/src/` 下创建 `qos_publisher.cpp`：

```cpp
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class QosPublisher : public rclcpp::Node
{
public:
  QosPublisher()
  : Node("qos_publisher"), count_(0)
  {
    rclcpp::QoS reliable(10);
    reliable.reliable();
    rclcpp::QoS best_effort(10);
    best_effort.best_effort();
    reliable_pub_ = create_publisher<std_msgs::msg::String>("/qos_reliable", reliable);
    best_effort_pub_ = create_publisher<std_msgs::msg::String>("/qos_best_effort", best_effort);
    timer_ = create_wall_timer(1s, std::bind(&QosPublisher::publish, this));
  }

private:
  void publish()
  {
    ++count_;
    std_msgs::msg::String reliable;
    reliable.data = "RELIABLE: " + std::to_string(count_);
    reliable_pub_->publish(reliable);
    std_msgs::msg::String best_effort;
    best_effort.data = "BEST_EFFORT: " + std::to_string(count_);
    best_effort_pub_->publish(best_effort);
    RCLCPP_INFO(get_logger(), "QOS_PUBLISHED count=%zu", count_);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reliable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr best_effort_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QosPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

**步骤2：测试兼容性**

| 实验 | Publisher QoS | Subscriber QoS | 预期结果 |
|------|--------------|----------------|---------|
| A | RELIABLE | RELIABLE | ✓ 正常通信 |
| B | RELIABLE | BEST_EFFORT | ✓ 降级通信 |
| C | BEST_EFFORT | BEST_EFFORT | ✓ 正常通信 |
| D | BEST_EFFORT | RELIABLE | ✗ 无法通信 |

在 `topic_demo_cpp/CMakeLists.txt` 的 `ament_package()` 前添加：

```cmake
find_package(std_msgs REQUIRED)
add_executable(qos_publisher src/qos_publisher.cpp)
target_compile_features(qos_publisher PUBLIC cxx_std_17)
ament_target_dependencies(qos_publisher rclcpp std_msgs)
install(TARGETS qos_publisher DESTINATION lib/${PROJECT_NAME})
```

```bash
source /opt/ros/humble/setup.bash
cd ~/my_ros2_ws
colcon build --packages-select topic_demo_cpp --symlink-install
source ~/my_ros2_ws/install/setup.bash
# 终端1：启动发布者
ros2 run topic_demo_cpp qos_publisher

# 终端2：分别测试不同 QoS 订阅
# 实验A：订阅 /qos_reliable，默认 RELIABLE
ros2 topic echo /qos_reliable

# 实验B：以 BEST_EFFORT 订阅 RELIABLE 话题 → 应成功
ros2 topic echo /qos_reliable --qos-reliability best_effort

# 实验D：以 RELIABLE 订阅 BEST_EFFORT 话题 → 应失败
ros2 topic echo /qos_best_effort --qos-reliability reliable

# 实验C：以 BEST_EFFORT 订阅 BEST_EFFORT 话题 → 应成功
ros2 topic echo /qos_best_effort --qos-reliability best_effort
```

**步骤3：检查运行结果与图示一致**

- 截图1：实验 A（成功）：/qos_reliable 正常输出

![K3 qos-a 对应实测结果](images/ch03/delivery-qos-a.png)

- 截图2：实验 D（失败）：ros2 topic echo 无输出

![K3 BEST_EFFORT 发布者与 RELIABLE 订阅者不兼容，接收超时为预期负例。](images/ch03/delivery-qos-d.png)

- 截图3：实验 C（成功）：/qos_best_effort 正常输出

![K3 BEST_EFFORT 发布者与 BEST_EFFORT 订阅者通信成功。](images/ch03/delivery-qos-c.png)

### 参考代码
> 完整参考代码位于 `src_k3_pico_itx/topic_demo_cpp/src/qos_publisher.cpp`

---

## 本章实验总结

| 练习 | 核心技能 | 关键 API |
|------|---------|---------|
| 练习1 | Publisher + Subscriber 编程 | `create_publisher()`, `create_subscription()` |
| 练习2 | 自定义消息接口定义与使用 | `.msg` 定义, `rosidl_generate_interfaces()` |
| 练习3 | QoS 策略配置与兼容性测试 | `QoSProfile`, `ReliabilityPolicy` |

### 思考题

1. 如果 Publisher 发布频率 100Hz，Subscriber 处理能力仅 50Hz，使用哪种 QoS 策略处理丢包？KEEP_LAST（depth），队列满就丢掉就消息，保证最新数据，如果可以丢包，可以配合BEST_EFFORT
2. KEEP_LAST(1) 和 KEEP_ALL 的使用场景分别是什么？KEEP_LAST(1)保存最新的一条消息，用于/cmd_vel实时状态数据；KEEP_ALL保存所有消息，用于日志
3. BEST_EFFORT + TRANSIENT_LOCAL 组合能否正常通信？为什么？可以，BEST_EFFORT属于可靠性策略，TRANSIENT_LOCAL是持久性策略，是不同的QoS维度，不冲突

---

## 练习3.4：Twist 控制 TurtleBot3 Burger 走方形轨迹（约 15 分钟）

### 目标
发布 `geometry_msgs/Twist` 消息到 `/cmd_vel`，驱动仿真中的 TurtleBot3 Burger 机器人在 Gazebo 中走正方形轨迹。

### 步骤

**步骤1：启动课程仿真**
```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch robot_sim_demo gazebo2.launch.py gui:=true rviz:=true drive:=false
```

**步骤2：创建 square_driver.cpp**
```cpp
#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SquareDriver : public rclcpp::Node
{
public:
  SquareDriver()
  : Node("square_driver")
  {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    motion_clock_ = get_parameter("use_sim_time").as_bool() ?
      get_clock() : std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    RCLCPP_INFO(get_logger(), "SQUARE_DRIVER_ACTIVE waiting=5s");
  }

  bool drive_square()
  {
    std::this_thread::sleep_for(5s);
    if (get_parameter("use_sim_time").as_bool() && now().nanoseconds() == 0) {
      RCLCPP_ERROR(get_logger(), "No simulation clock received; motion not started");
      return false;
    }
    for (int side = 1; side <= 4 && rclcpp::ok(); ++side) {
      RCLCPP_INFO(get_logger(), "SQUARE_SIDE %d straight", side);
      move(0.2, 0.0, 5s);
      RCLCPP_INFO(get_logger(), "SQUARE_SIDE %d turn", side);
      move(0.0, 1.57, 1s);
    }
    move(0.0, 0.0, 500ms);
    RCLCPP_INFO(get_logger(), "SQUARE_DRIVER_DONE");
    return true;
  }

private:
  void move(double linear, double angular, std::chrono::milliseconds duration)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = linear;
    message.angular.z = angular;
    const auto end = motion_clock_->now() + rclcpp::Duration(duration);
    while (rclcpp::ok() && motion_clock_->now() < end) {
      publisher_->publish(message);
      if (!motion_clock_->sleep_until(
          std::min(end, motion_clock_->now() + rclcpp::Duration(100ms)))) {
        break;
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr motion_clock_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SquareDriver>();
  std::thread executor([node]() {rclcpp::spin(node);});
  const bool completed = node->drive_square();
  rclcpp::shutdown();
  executor.join();
  return completed ? 0 : 1;
}
```

在 `topic_demo_cpp/CMakeLists.txt` 的 `ament_package()` 前添加控制器并重新编译：

```cmake
add_executable(square_driver src/square_driver.cpp)
target_compile_features(square_driver PUBLIC cxx_std_17)
ament_target_dependencies(square_driver rclcpp geometry_msgs)
install(TARGETS square_driver DESTINATION lib/${PROJECT_NAME})
```

```bash
cd ~/my_ros2_ws
colcon build --packages-select topic_demo_cpp
source install/setup.bash
```

仿真运动按已确认的适配使用 ROS `/clock` 计时；启动等待仍为 5 秒墙钟，未收到仿真时钟时退出，不开始运动。

**步骤3：运行测试**

【K3】保持 x86 的 Gazebo 仿真运行，在已加载本章工作空间的终端执行：

```bash
source ~/.config/ros2-course-k3/env.bash
source ~/my_ros2_ws/install/setup.bash
ros2 run topic_demo_cpp square_driver --ros-args -p use_sim_time:=true
```

**✓ 验证**：

- 截图1：Gazebo 中 TurtleBot3 Burger 按正方形轨迹运动

![x86 Gazebo 中 Burger 正在执行 K3 方形控制；完整四边运动和停止见同轮连续录像。](images/ch03/delivery-square-gazebo.png)

- 截图2：RViz 中 /odom 轨迹显示正方形图案

![x86 RViz 的 Odometry 箭头显示方形轨迹；本图来自 2026-09-06 独立轨迹补验，固定坐标系为 odom。](images/ch03/run-20260906T050801Z--after-rviz.png)

- 截图3：终端日志输出四边一角的执行进度

![K3 square_driver 完成四次直行与转弯，跨度约 1 m × 1 m，闭合误差 0.0053 m，末条速度为零。](images/ch03/delivery-square-progress.png)

### 参考代码
> 完整参考代码位于 `src_k3_pico_itx/topic_demo_cpp/src/square_driver.cpp`

### 思考题
1. 如何精确控制机器人走 2m×2m 的正方形？调整哪些参数？用直线运动+原地旋转90°循环四次。要调整线速度、角速度、运动时间、控制频率
2. 如果需要在正方形路径上添加圆角过渡，如何实现？在拐角处加入圆弧轨迹。在接近拐角时同时设置线速度和角速度，使机器人沿圆弧过渡。

## 实际运行证据

**本章演示：完整方形与执行阶段。** 固定区域保留完整四边与停止，右侧按 square_driver 日志显示当前直行/转弯阶段。原速展示；5 s 直行和 1 s 转弯按仿真时钟计时。RViz 历史轨迹仍见下方独立截图。

[播放演示（约 77 秒）](https://gitee.com/chuachuaa/ROS2_RISCV/releases/download/k3-ch01-04-media-20260908/ch03--chapter-demo.mp4)。左侧为同轮连续 Gazebo 原片的固定区域放大；右侧为实际按键/日志按 UTC 近似对齐的后期回放，不是原生终端画面。原始记录见统一证据页。


[K3 实际运行证据](runtime_evidence.md#ch03)登记本章可用素材及不同运行的边界。

真实运行的话题发布、订阅、自定义消息接口和消息输出：

![ch03 话题通信运行输出](images/runtime/ch03/delivery-gps-qos.gif)

原始录制：[ch03_topics.cast](https://gitee.com/chuachuaa/ROS2_RISCV/releases/download/k3-ch01-04-media-20260908/ch03--delivery-gps-qos.cast)。完整证据索引见[实际运行证据](runtime_evidence.md)。
